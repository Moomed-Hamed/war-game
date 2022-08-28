#ifndef BT_DISCRETE_DYNAMICS_WORLD_MT_H
#define BT_DISCRETE_DYNAMICS_WORLD_MT_H

#include "btDiscreteDynamicsWorld.h"
#include "btSimulationIslandManagerMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

// btConstraintSolverPoolMt - masquerades as a constraint solver, but really it is a threadsafe pool of them.
//
// Each solver in the pool is protected by a mutex.  When solveGroup is called from a thread,
// the pool looks for a solver that isn't being used by another thread, locks it, and dispatches the
// call to the solver.
// So long as there are at least as many solvers as there are hardware threads, it should never need to spin wait.

class btConstraintSolverPoolMt : public btConstraintSolver
{
public:
	// create the solvers for me
	explicit btConstraintSolverPoolMt(int numSolvers)
	{
		btAlignedObjectArray<btConstraintSolver*> solvers;
		solvers.reserve(numSolvers);
		for (int i = 0; i < numSolvers; ++i)
		{
			btConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
			solvers.push_back(solver);
		}
		init(&solvers[0], numSolvers);
	}

	// pass in fully constructed solvers (destructor will delete them)
	btConstraintSolverPoolMt(btConstraintSolver** solvers, int numSolvers)
	{
		init(solvers, numSolvers);
	}

	virtual ~btConstraintSolverPoolMt()
	{
		// delete all solvers
		for (int i = 0; i < m_solvers.size(); ++i)
		{
			ThreadSolver& solver = m_solvers[i];
			delete solver.solver;
			solver.solver = NULL;
		}
	}

	///solve a group of constraints
	virtual btScalar solveGroup(btCollisionObject** bodies,
								int numBodies,
								btPersistentManifold** manifolds,
								int numManifolds,
								btTypedConstraint** constraints,
								int numConstraints,
								const btContactSolverInfo& info,
								btIDebugDraw* debugDrawer,
								btDispatcher* dispatcher) BT_OVERRIDE
	{
		ThreadSolver* ts = getAndLockThreadSolver();
		ts->solver->solveGroup(bodies, numBodies, manifolds, numManifolds, constraints, numConstraints, info, debugDrawer, dispatcher);
		ts->mutex.unlock();
		return 0.0f;
	}

	virtual void reset() BT_OVERRIDE
	{
		for (int i = 0; i < m_solvers.size(); ++i)
		{
			ThreadSolver& solver = m_solvers[i];
			solver.mutex.lock();
			solver.solver->reset();
			solver.mutex.unlock();
		}
	}
	virtual btConstraintSolverType getSolverType() const BT_OVERRIDE { return m_solverType; }

private:
	const static size_t kCacheLineSize = 128;
	struct ThreadSolver
	{
		btConstraintSolver* solver;
		btSpinMutex mutex;
		char _cachelinePadding[kCacheLineSize - sizeof(btSpinMutex) - sizeof(void*)];  // keep mutexes from sharing a cache line
	};
	btAlignedObjectArray<ThreadSolver> m_solvers;
	btConstraintSolverType m_solverType;

	ThreadSolver* getAndLockThreadSolver()
	{
		int i = 0;
		while (true)
		{
			ThreadSolver& solver = m_solvers[i];
			if (solver.mutex.tryLock())
			{
				return &solver;
			}
			// failed, try the next one
			i = (i + 1) % m_solvers.size();
		}
		return NULL;
	}
	void init(btConstraintSolver** solvers, int numSolvers)
	{
		m_solverType = BT_SEQUENTIAL_IMPULSE_SOLVER;
		m_solvers.resize(numSolvers);
		for (int i = 0; i < numSolvers; ++i)
		{
			m_solvers[i].solver = solvers[i];
		}
		if (numSolvers > 0)
		{
			m_solverType = solvers[0]->getSolverType();
		}
	}
};

static struct UpdaterUnconstrainedMotion : public btIParallelForBody
{
	btScalar timeStep;
	btRigidBody** rigidBodies;

	void forLoop(int iBegin, int iEnd) const BT_OVERRIDE
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btRigidBody* body = rigidBodies[i];
			if (!body->isStaticOrKinematicObject())
			{
				//don't integrate/update velocities here, it happens in the constraint solver
				body->applyDamping(timeStep);
				body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
			}
		}
	}
};

///
/// btDiscreteDynamicsWorldMt -- a version of DiscreteDynamicsWorld with some minor changes to support
///                              solving simulation islands on multiple threads.
///
///  Should function exactly like btDiscreteDynamicsWorld.
///  Also 3 methods that iterate over all of the rigidbodies can run in parallel:
///     - predictUnconstraintMotion
///     - integrateTransforms
///     - createPredictiveContacts
///
ATTRIBUTE_ALIGNED16(class)
btDiscreteDynamicsWorldMt : public btDiscreteDynamicsWorld
{
protected:
	btConstraintSolver* m_constraintSolverMt;

	virtual void solveConstraints(btContactSolverInfo & solverInfo) BT_OVERRIDE
	{
		BT_PROFILE("solveConstraints");

		m_constraintSolver->prepareSolve(getCollisionWorld()->getNumCollisionObjects(), getCollisionWorld()->getDispatcher()->getNumManifolds());

		/// solve all the constraints for this island
		btSimulationIslandManagerMt* im = static_cast<btSimulationIslandManagerMt*>(m_islandManager);
		btSimulationIslandManagerMt::SolverParams solverParams;
		solverParams.m_solverPool = m_constraintSolver;
		solverParams.m_solverMt = m_constraintSolverMt;
		solverParams.m_solverInfo = &solverInfo;
		solverParams.m_debugDrawer = m_debugDrawer;
		solverParams.m_dispatcher = getCollisionWorld()->getDispatcher();
		im->buildAndProcessIslands(getCollisionWorld()->getDispatcher(), getCollisionWorld(), m_constraints, solverParams);

		m_constraintSolver->allSolved(solverInfo, m_debugDrawer);
	}

	virtual void predictUnconstraintMotion(btScalar timeStep) BT_OVERRIDE
	{
		BT_PROFILE("predictUnconstraintMotion");
		if (m_nonStaticRigidBodies.size() > 0)
		{
			UpdaterUnconstrainedMotion update;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];
			int grainSize = 50;  // num of iterations per task for task scheduler
			btParallelFor(0, m_nonStaticRigidBodies.size(), grainSize, update);
		}
	}

	struct UpdaterCreatePredictiveContacts : public btIParallelForBody
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;
		btDiscreteDynamicsWorldMt* world;

		void forLoop(int iBegin, int iEnd) const BT_OVERRIDE
		{
			world->createPredictiveContactsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};
	virtual void createPredictiveContacts(btScalar timeStep) BT_OVERRIDE
	{
		BT_PROFILE("createPredictiveContacts");
		releasePredictiveContacts();
		if (m_nonStaticRigidBodies.size() > 0)
		{
			UpdaterCreatePredictiveContacts update;
			update.world = this;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];
			int grainSize = 50;  // num of iterations per task for task scheduler
			btParallelFor(0, m_nonStaticRigidBodies.size(), grainSize, update);
		}
	}

	struct UpdaterIntegrateTransforms : public btIParallelForBody
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;
		btDiscreteDynamicsWorldMt* world;

		void forLoop(int iBegin, int iEnd) const BT_OVERRIDE
		{
			world->integrateTransformsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};
	virtual void integrateTransforms(btScalar timeStep) BT_OVERRIDE
	{
		BT_PROFILE("integrateTransforms");
		if (m_nonStaticRigidBodies.size() > 0)
		{
			UpdaterIntegrateTransforms update;
			update.world = this;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];
			int grainSize = 50;  // num of iterations per task for task scheduler
			btParallelFor(0, m_nonStaticRigidBodies.size(), grainSize, update);
		}
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btDiscreteDynamicsWorldMt(btDispatcher * dispatcher,
							  btBroadphaseInterface * pairCache,
							  btConstraintSolverPoolMt * solverPool,        // Note this should be a solver-pool for multi-threading
							  btConstraintSolver * constraintSolverMt,      // single multi-threaded solver for large islands (or NULL)
							  btCollisionConfiguration * collisionConfiguration)
		: btDiscreteDynamicsWorld(dispatcher, pairCache, solverPool, collisionConfiguration)
	{
		if (m_ownsIslandManager)
		{
			m_islandManager->~btSimulationIslandManager();
			btAlignedFree(m_islandManager);
		}
		{
			void* mem = btAlignedAlloc(sizeof(btSimulationIslandManagerMt), 16);
			btSimulationIslandManagerMt* im = new (mem) btSimulationIslandManagerMt();
			im->setMinimumSolverBatchSize(m_solverInfo.m_minimumSolverBatchSize);
			m_islandManager = im;
		}
		m_constraintSolverMt = constraintSolverMt;
	}
	virtual ~btDiscreteDynamicsWorldMt() {}

	virtual int stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep) BT_OVERRIDE
	{
		int numSubSteps = btDiscreteDynamicsWorld::stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
		if (btITaskScheduler* scheduler = btGetTaskScheduler())
		{
			// tell Bullet's threads to sleep, so other threads can run
			scheduler->sleepWorkerThreadsHint();
		}
		return numSubSteps;
	}
};

#endif  //BT_DISCRETE_DYNAMICS_WORLD_H
