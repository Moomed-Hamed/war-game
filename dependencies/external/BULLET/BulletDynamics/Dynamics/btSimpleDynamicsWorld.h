#ifndef BT_SIMPLE_DYNAMICS_WORLD_H
#define BT_SIMPLE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

// Make sure this dummy function never changes so that it
// can be used by probes that are checking whether the
// library is actually installed.
//extern "C"
//{
//	void btBulletDynamicsProbe();
//	void btBulletDynamicsProbe() {}
//}

class btDispatcher;
class btOverlappingPairCache;
class btConstraintSolver;

///The btSimpleDynamicsWorld serves as unit-test and to verify more complicated and optimized dynamics worlds.
///Please use btDiscreteDynamicsWorld instead
class btSimpleDynamicsWorld : public btDynamicsWorld
{
protected:
	btConstraintSolver* m_constraintSolver;

	bool m_ownsConstraintSolver;

	void predictUnconstraintMotion(btScalar timeStep)
	{
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body)
			{
				if (!body->isStaticObject())
				{
					if (body->isActive())
					{
						body->applyGravity();
						body->integrateVelocities(timeStep);
						body->applyDamping(timeStep);
						body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
					}
				}
			}
		}
	}

	void integrateTransforms(btScalar timeStep)
	{
		btTransform predictedTrans;
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body)
			{
				if (body->isActive() && (!body->isStaticObject()))
				{
					body->predictIntegratedTransform(timeStep, predictedTrans);
					body->proceedToTransform(predictedTrans);
				}
			}
		}
	}

	btVector3 m_gravity;

public:
	///this btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache and constraintSolver
	btSimpleDynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration)
		: btDynamicsWorld(dispatcher, pairCache, collisionConfiguration),
		  m_constraintSolver(constraintSolver),
		  m_ownsConstraintSolver(false),
		  m_gravity(0, 0, -10)
	{
	}

	virtual ~btSimpleDynamicsWorld()
	{
		if (m_ownsConstraintSolver)
			btAlignedFree(m_constraintSolver);
	}

	///maxSubSteps/fixedTimeStep for interpolation is currently ignored for btSimpleDynamicsWorld, use btDiscreteDynamicsWorld instead
	virtual int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.))
	{
		(void)fixedTimeStep;
		(void)maxSubSteps;

		///apply gravity, predict motion
		predictUnconstraintMotion(timeStep);

		btDispatcherInfo& dispatchInfo = getDispatchInfo();
		dispatchInfo.m_timeStep = timeStep;
		dispatchInfo.m_stepCount = 0;
		dispatchInfo.m_debugDraw = getDebugDrawer();

		///perform collision detection
		performDiscreteCollisionDetection();

		///solve contact constraints
		int numManifolds = m_dispatcher1->getNumManifolds();
		if (numManifolds)
		{
			btPersistentManifold** manifoldPtr = ((btCollisionDispatcher*)m_dispatcher1)->getInternalManifoldPointer();

			btContactSolverInfo infoGlobal;
			infoGlobal.m_timeStep = timeStep;
			m_constraintSolver->prepareSolve(0, numManifolds);
			m_constraintSolver->solveGroup(&getCollisionObjectArray()[0], getNumCollisionObjects(), manifoldPtr, numManifolds, 0, 0, infoGlobal, m_debugDrawer, m_dispatcher1);
			m_constraintSolver->allSolved(infoGlobal, m_debugDrawer);
		}

		///integrate transforms
		integrateTransforms(timeStep);

		updateAabbs();

		synchronizeMotionStates();

		clearForces();

		return 1;
	}

	virtual void setGravity(const btVector3& gravity)
	{
		m_gravity = gravity;
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body)
			{
				body->setGravity(gravity);
			}
		}
	}

	virtual btVector3 getGravity() const
	{
		return m_gravity;
	}

	virtual void addRigidBody(btRigidBody* body)
	{
		body->setGravity(m_gravity);

		if (body->getCollisionShape())
		{
			addCollisionObject(body);
		}
	}

	virtual void addRigidBody(btRigidBody* body, int group, int mask)
	{
		body->setGravity(m_gravity);

		if (body->getCollisionShape())
		{
			addCollisionObject(body, group, mask);
		}
	}

	virtual void removeRigidBody(btRigidBody* body)
	{
		btCollisionWorld::removeCollisionObject(body);
	}

	virtual void debugDrawWorld() {}
	virtual void addAction(btActionInterface* action) {}
	virtual void removeAction(btActionInterface* action) {}

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
	virtual void removeCollisionObject(btCollisionObject* collisionObject)
	{
		btRigidBody* body = btRigidBody::upcast(collisionObject);
		if (body)
			removeRigidBody(body);
		else
			btCollisionWorld::removeCollisionObject(collisionObject);
	}

	virtual void updateAabbs()
	{
		btTransform predictedTrans;
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body)
			{
				if (body->isActive() && (!body->isStaticObject()))
				{
					btVector3 minAabb, maxAabb;
					colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb, maxAabb);
					btBroadphaseInterface* bp = getBroadphase();
					bp->setAabb(body->getBroadphaseHandle(), minAabb, maxAabb, m_dispatcher1);
				}
			}
		}
	}

	virtual void synchronizeMotionStates()
	{
		///@todo: iterate over awake simulation islands!
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body && body->getMotionState())
			{
				if (body->getActivationState() != ISLAND_SLEEPING)
				{
					body->getMotionState()->setWorldTransform(body->getWorldTransform());
				}
			}
		}
	}

	virtual void setConstraintSolver(btConstraintSolver* solver)
	{
		if (m_ownsConstraintSolver)
		{
			btAlignedFree(m_constraintSolver);
		}
		m_ownsConstraintSolver = false;
		m_constraintSolver = solver;
	}

	virtual btConstraintSolver* getConstraintSolver()
	{
		return m_constraintSolver;
	}

	virtual btDynamicsWorldType getWorldType() const
	{
		return BT_SIMPLE_DYNAMICS_WORLD;
	}

	virtual void clearForces()
	{
		///@todo: iterate over awake simulation islands!
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];

			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body)
			{
				body->clearForces();
			}
		}
	}
};

#endif  //BT_SIMPLE_DYNAMICS_WORLD_H
