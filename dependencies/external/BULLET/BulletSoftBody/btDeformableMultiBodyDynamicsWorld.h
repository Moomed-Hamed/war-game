/*
	Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
	Bullet Continuous Collision Detection and Physics Library
	Copyright (c) 2019 Google Inc. http://bulletphysics.org
*/

/* ----- Overview of the Deformable Algorithm -----

	A single step of the deformable body simulation contains the following main components:
	Call internalStepSimulation multiple times, to achieve 240Hz (4 steps of 60Hz).
	1. Deformable maintaintenance of rest lengths and volume preservation. Forces only depend on position: Update velocity to a temporary state v_{n+1}^* = v_n + explicit_force * dt / mass, where explicit forces include gravity and elastic forces.
	2. Detect discrete collisions between rigid and deformable bodies at position x_{n+1}^* = x_n + dt * v_{n+1}^*.
	3a. Solve all constraints, including LCP. Contact, position correction due to numerical drift, friction, and anchors for deformable.
	3b. 5 Newton steps (multiple step). Conjugent Gradient solves linear system. Deformable Damping: Then velocities of deformable bodies v_{n+1} are solved in
	        M(v_{n+1} - v_{n+1}^*) = damping_force * dt / mass,
	   by a conjugate gradient solver, where the damping force is implicit and depends on v_{n+1}.
	   Make sure contact constraints are not violated in step b by performing velocity projections as in the paper by Baraff and Witkin https://www.cs.cmu.edu/~baraff/papers/sig98.pdf. Dynamic frictions are treated as a force and added to the rhs of the CG solve, whereas static frictions are treated as constraints similar to contact.
	4. Position is updated via x_{n+1} = x_n + dt * v_{n+1}.
	
	The algorithm also closely resembles the one in http://physbam.stanford.edu/~fedkiw/papers/stanford2008-03.pdf
*/

#ifndef BT_DEFORMABLE_MULTIBODY_DYNAMICS_WORLD_H
#define BT_DEFORMABLE_MULTIBODY_DYNAMICS_WORLD_H

#include "btSoftMultiBodyDynamicsWorld.h"
#include "btDeformableMultiBodyConstraintSolver.h"
#include "btSoftBodyHelpers.h"
#include "DeformableBodyInplaceSolverIslandCallback.h"

typedef btAlignedObjectArray<btSoftBody*> btSoftBodyArray;

class btDeformableBodySolver;
class btDeformableLagrangianForce;
struct MultiBodyInplaceSolverIslandCallback;
struct DeformableBodyInplaceSolverIslandCallback;
class btDeformableMultiBodyConstraintSolver;

typedef btAlignedObjectArray<btSoftBody*> btSoftBodyArray;

class btDeformableMultiBodyDynamicsWorld : public btMultiBodyDynamicsWorld
{
	typedef btAlignedObjectArray<btVector3> TVStack;
	///Solver classes that encapsulate multiple deformable bodies for solving
	btDeformableBodySolver* m_deformableBodySolver;
	btSoftBodyArray m_softBodies;
	int m_drawFlags;
	bool m_drawNodeTree;
	bool m_drawFaceTree;
	bool m_drawClusterTree;
	btSoftBodyWorldInfo m_sbi;
	btScalar m_internalTime;
	int m_ccdIterations;
	bool m_implicit;
	bool m_lineSearch;
	bool m_useProjection;
	DeformableBodyInplaceSolverIslandCallback* m_solverDeformableBodyIslandCallback;

	typedef void (*btSolverCallback)(btScalar time, btDeformableMultiBodyDynamicsWorld* world);
	btSolverCallback m_solverCallback;

protected:
	virtual void internalSingleStepSimulation(btScalar timeStep)
	{
		BT_PROFILE("internalSingleStepSimulation");
		if (0 != m_internalPreTickCallback)
		{
			(*m_internalPreTickCallback)(this, timeStep);
		}
		reinitialize(timeStep);

		// add gravity to velocity of rigid and multi bodys
		applyRigidBodyGravity(timeStep);

		///apply gravity and explicit force to velocity, predict motion
		predictUnconstraintMotion(timeStep);

		///perform collision detection that involves rigid/multi bodies
		btMultiBodyDynamicsWorld::performDiscreteCollisionDetection();

		btMultiBodyDynamicsWorld::calculateSimulationIslands();

		beforeSolverCallbacks(timeStep);

		///solve contact constraints and then deformable bodies momemtum equation
		solveConstraints(timeStep);

		afterSolverCallbacks(timeStep);

		performDeformableCollisionDetection();

		applyRepulsionForce(timeStep);

		performGeometricCollisions(timeStep);

		integrateTransforms(timeStep);

		///update vehicle simulation
		btMultiBodyDynamicsWorld::updateActions(timeStep);

		updateActivationState(timeStep);
		// End solver-wise simulation step
		// ///////////////////////////////
	}

	virtual void integrateTransforms(btScalar timeStep)
	{
		BT_PROFILE("integrateTransforms");
		positionCorrection(timeStep);
		btMultiBodyDynamicsWorld::integrateTransforms(timeStep);
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				btSoftBody::Node& node = psb->m_nodes[j];
				btScalar maxDisplacement = psb->getWorldInfo()->m_maxDisplacement;
				btScalar clampDeltaV = maxDisplacement / timeStep;
				for (int c = 0; c < 3; c++)
				{
					if (node.m_v[c] > clampDeltaV)
					{
						node.m_v[c] = clampDeltaV;
					}
					if (node.m_v[c] < -clampDeltaV)
					{
						node.m_v[c] = -clampDeltaV;
					}
				}
				node.m_x = node.m_x + timeStep * (node.m_v + node.m_splitv);
				node.m_q = node.m_x;
				node.m_vn = node.m_v;
			}
			// enforce anchor constraints
			for (int j = 0; j < psb->m_deformableAnchors.size(); ++j)
			{
				btSoftBody::DeformableNodeRigidAnchor& a = psb->m_deformableAnchors[j];
				btSoftBody::Node* n = a.m_node;
				n->m_x = a.m_cti.m_colObj->getWorldTransform() * a.m_local;

				// update multibody anchor info
				if (a.m_cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
				{
					btMultiBodyLinkCollider* multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(a.m_cti.m_colObj);
					if (multibodyLinkCol)
					{
						btVector3 nrm;
						const btCollisionShape* shp = multibodyLinkCol->getCollisionShape();
						const btTransform& wtr = multibodyLinkCol->getWorldTransform();
						psb->m_worldInfo->m_sparsesdf.Evaluate(
							wtr.invXform(n->m_x),
							shp,
							nrm,
							0);
						a.m_cti.m_normal = wtr.getBasis() * nrm;
						btVector3 normal = a.m_cti.m_normal;
						btVector3 t1 = generateUnitOrthogonalVector(normal);
						btVector3 t2 = btCross(normal, t1);
						btMultiBodyJacobianData jacobianData_normal, jacobianData_t1, jacobianData_t2;
						findJacobian(multibodyLinkCol, jacobianData_normal, a.m_node->m_x, normal);
						findJacobian(multibodyLinkCol, jacobianData_t1, a.m_node->m_x, t1);
						findJacobian(multibodyLinkCol, jacobianData_t2, a.m_node->m_x, t2);

						btScalar* J_n = &jacobianData_normal.m_jacobians[0];
						btScalar* J_t1 = &jacobianData_t1.m_jacobians[0];
						btScalar* J_t2 = &jacobianData_t2.m_jacobians[0];

						btScalar* u_n = &jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
						btScalar* u_t1 = &jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
						btScalar* u_t2 = &jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];

						btMatrix3x3 rot(normal.getX(), normal.getY(), normal.getZ(),
										t1.getX(), t1.getY(), t1.getZ(),
										t2.getX(), t2.getY(), t2.getZ());  // world frame to local frame
						const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
						btMatrix3x3 local_impulse_matrix = (Diagonal(n->m_im) + OuterProduct(J_n, J_t1, J_t2, u_n, u_t1, u_t2, ndof)).inverse();
						a.m_c0 = rot.transpose() * local_impulse_matrix * rot;
						a.jacobianData_normal = jacobianData_normal;
						a.jacobianData_t1 = jacobianData_t1;
						a.jacobianData_t2 = jacobianData_t2;
						a.t1 = t1;
						a.t2 = t2;
					}
				}
			}
			psb->interpolateRenderMesh();
		}
	}

	void positionCorrection(btScalar timeStep)
	{
		// correct the position of rigid bodies with temporary velocity generated from split impulse
		btContactSolverInfo infoGlobal;
		btVector3 zero(0, 0, 0);
		for (int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
		{
			btRigidBody* rb = m_nonStaticRigidBodies[i];
			//correct the position/orientation based on push/turn recovery
			btTransform newTransform;
			btVector3 pushVelocity = rb->getPushVelocity();
			btVector3 turnVelocity = rb->getTurnVelocity();
			if (pushVelocity[0] != 0.f || pushVelocity[1] != 0 || pushVelocity[2] != 0 || turnVelocity[0] != 0.f || turnVelocity[1] != 0 || turnVelocity[2] != 0)
			{
				btTransformUtil::integrateTransform(rb->getWorldTransform(), pushVelocity, turnVelocity * infoGlobal.m_splitImpulseTurnErp, timeStep, newTransform);
				rb->setWorldTransform(newTransform);
				rb->setPushVelocity(zero);
				rb->setTurnVelocity(zero);
			}
		}
	}

	void solveConstraints(btScalar timeStep)
	{
		BT_PROFILE("btDeformableMultiBodyDynamicsWorld::solveConstraints");
		// save v_{n+1}^* velocity after explicit forces
		m_deformableBodySolver->backupVelocity();

		// set up constraints among multibodies and between multibodies and deformable bodies
		setupConstraints();

		// solve contact constraints
		solveContactConstraints();

		// set up the directions in which the velocity does not change in the momentum solve
		if (m_useProjection)
			m_deformableBodySolver->m_objective->m_projection.setProjection();
		else
			m_deformableBodySolver->m_objective->m_projection.setLagrangeMultiplier();

		// for explicit scheme, m_backupVelocity = v_{n+1}^*
		// for implicit scheme, m_backupVelocity = v_n
		// Here, set dv = v_{n+1} - v_n for nodes in contact
		m_deformableBodySolver->setupDeformableSolve(m_implicit);

		// At this point, dv should be golden for nodes in contact
		// proceed to solve deformable momentum equation
		m_deformableBodySolver->solveDeformableConstraints(timeStep);
	}

	void updateActivationState(btScalar timeStep)
	{
		for (int i = 0; i < m_softBodies.size(); i++)
		{
			btSoftBody* psb = m_softBodies[i];
			psb->updateDeactivation(timeStep);
			if (psb->wantsSleeping())
			{
				if (psb->getActivationState() == ACTIVE_TAG)
					psb->setActivationState(WANTS_DEACTIVATION);
				if (psb->getActivationState() == ISLAND_SLEEPING)
				{
					psb->setZeroVelocity();
				}
			}
			else
			{
				if (psb->getActivationState() != DISABLE_DEACTIVATION)
					psb->setActivationState(ACTIVE_TAG);
			}
		}
		btMultiBodyDynamicsWorld::updateActivationState(timeStep);
	}

	void clearGravity()
	{
		BT_PROFILE("btMultiBody clearGravity");
		// clear rigid body gravity
		for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
		{
			btRigidBody* body = m_nonStaticRigidBodies[i];
			if (body->isActive())
			{
				body->clearGravity();
			}
		}
		// clear multibody gravity
		for (int i = 0; i < this->m_multiBodies.size(); i++)
		{
			btMultiBody* bod = m_multiBodies[i];

			bool isSleeping = false;

			if (bod->getBaseCollider() && bod->getBaseCollider()->getActivationState() == ISLAND_SLEEPING)
			{
				isSleeping = true;
			}
			for (int b = 0; b < bod->getNumLinks(); b++)
			{
				if (bod->getLink(b).m_collider && bod->getLink(b).m_collider->getActivationState() == ISLAND_SLEEPING)
					isSleeping = true;
			}

			if (!isSleeping)
			{
				bod->addBaseForce(-m_gravity * bod->getBaseMass());

				for (int j = 0; j < bod->getNumLinks(); ++j)
				{
					bod->addLinkForce(j, -m_gravity * bod->getLinkMass(j));
				}
			}
		}
	}

public:
	btDeformableMultiBodyDynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btDeformableMultiBodyConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration, btDeformableBodySolver* deformableBodySolver = 0)
		: btMultiBodyDynamicsWorld(dispatcher, pairCache, (btMultiBodyConstraintSolver*)constraintSolver, collisionConfiguration),
		  m_deformableBodySolver(deformableBodySolver),
		  m_solverCallback(0)
	{
		m_drawFlags = fDrawFlags::Std;
		m_drawNodeTree = true;
		m_drawFaceTree = false;
		m_drawClusterTree = false;
		m_sbi.m_broadphase = pairCache;
		m_sbi.m_dispatcher = dispatcher;
		m_sbi.m_sparsesdf.Initialize();
		m_sbi.m_sparsesdf.setDefaultVoxelsz(0.005);
		m_sbi.m_sparsesdf.Reset();

		m_sbi.air_density = (btScalar)1.2;
		m_sbi.water_density = 0;
		m_sbi.water_offset = 0;
		m_sbi.water_normal = btVector3(0, 0, 0);
		m_sbi.m_gravity.setValue(0, -9.8, 0);
		m_internalTime = 0.0;
		m_implicit = false;
		m_lineSearch = false;
		m_useProjection = false;
		m_ccdIterations = 5;
		m_solverDeformableBodyIslandCallback = new DeformableBodyInplaceSolverIslandCallback(constraintSolver, dispatcher);
	}

	virtual int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.))
	{
		startProfiling(timeStep);

		int numSimulationSubSteps = 0;

		if (maxSubSteps)
		{
			//fixed timestep with interpolation
			m_fixedTimeStep = fixedTimeStep;
			m_localTime += timeStep;
			if (m_localTime >= fixedTimeStep)
			{
				numSimulationSubSteps = int(m_localTime / fixedTimeStep);
				m_localTime -= numSimulationSubSteps * fixedTimeStep;
			}
		}
		else
		{
			//variable timestep
			fixedTimeStep = timeStep;
			m_localTime = m_latencyMotionStateInterpolation ? 0 : timeStep;
			m_fixedTimeStep = 0;
			if (btFuzzyZero(timeStep))
			{
				numSimulationSubSteps = 0;
				maxSubSteps = 0;
			}
			else
			{
				numSimulationSubSteps = 1;
				maxSubSteps = 1;
			}
		}

		//process some debugging flags
		if (getDebugDrawer())
		{
			btIDebugDraw* debugDrawer = getDebugDrawer();
			gDisableDeactivation = (debugDrawer->getDebugMode() & btIDebugDraw::DBG_NoDeactivation) != 0;
		}
		if (numSimulationSubSteps)
		{
			//clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
			int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps) ? maxSubSteps : numSimulationSubSteps;

			saveKinematicState(fixedTimeStep * clampedSimulationSteps);

			for (int i = 0; i < clampedSimulationSteps; i++)
			{
				internalSingleStepSimulation(fixedTimeStep);
				synchronizeMotionStates();
			}
		}
		else
		{
			synchronizeMotionStates();
		}

		clearForces();

#ifndef BT_NO_PROFILE
		CProfileManager::Increment_Frame_Counter();
#endif  //BT_NO_PROFILE

		return numSimulationSubSteps;
	}

	virtual void debugDrawWorld()
	{
		btMultiBodyDynamicsWorld::debugDrawWorld();

		for (int i = 0; i < getSoftBodyArray().size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)getSoftBodyArray()[i];
			{
				btSoftBodyHelpers::DrawFrame(psb, getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, getDebugDrawer(), getDrawFlags());
			}
		}
	}

	void setSolverCallback(btSolverCallback cb)
	{
		m_solverCallback = cb;
	}

	virtual ~btDeformableMultiBodyDynamicsWorld()
	{
		delete m_solverDeformableBodyIslandCallback;
	}

	virtual btMultiBodyDynamicsWorld* getMultiBodyDynamicsWorld()
	{
		return (btMultiBodyDynamicsWorld*)(this);
	}

	virtual const btMultiBodyDynamicsWorld* getMultiBodyDynamicsWorld() const
	{
		return (const btMultiBodyDynamicsWorld*)(this);
	}

	virtual btDynamicsWorldType getWorldType() const
	{
		return BT_DEFORMABLE_MULTIBODY_DYNAMICS_WORLD;
	}

	virtual void predictUnconstraintMotion(btScalar timeStep)
	{
		BT_PROFILE("predictUnconstraintMotion");
		btMultiBodyDynamicsWorld::predictUnconstraintMotion(timeStep);
		m_deformableBodySolver->predictMotion(timeStep);
	}

	virtual void addSoftBody(btSoftBody* body, int collisionFilterGroup = btBroadphaseProxy::DefaultFilter, int collisionFilterMask = btBroadphaseProxy::AllFilter)
	{
		m_softBodies.push_back(body);

		// Set the soft body solver that will deal with this body to be the world's solver
		body->setSoftBodySolver(m_deformableBodySolver);

		btCollisionWorld::addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
	}

	btSoftBodyArray& getSoftBodyArray()
	{
		return m_softBodies;
	}

	const btSoftBodyArray& getSoftBodyArray() const
	{
		return m_softBodies;
	}

	btSoftBodyWorldInfo& getWorldInfo()
	{
		return m_sbi;
	}

	const btSoftBodyWorldInfo& getWorldInfo() const
	{
		return m_sbi;
	}

	void reinitialize(btScalar timeStep)
	{
		m_internalTime += timeStep;
		m_deformableBodySolver->setImplicit(m_implicit);
		m_deformableBodySolver->setLineSearch(m_lineSearch);
		m_deformableBodySolver->reinitialize(m_softBodies, timeStep);
		btDispatcherInfo& dispatchInfo = btMultiBodyDynamicsWorld::getDispatchInfo();
		dispatchInfo.m_timeStep = timeStep;
		dispatchInfo.m_stepCount = 0;
		dispatchInfo.m_debugDraw = btMultiBodyDynamicsWorld::getDebugDrawer();
		btMultiBodyDynamicsWorld::getSolverInfo().m_timeStep = timeStep;
		if (m_useProjection)
		{
			m_deformableBodySolver->m_useProjection = true;
			m_deformableBodySolver->m_objective->m_projection.m_useStrainLimiting = true;
			m_deformableBodySolver->m_objective->m_preconditioner = m_deformableBodySolver->m_objective->m_massPreconditioner;
		}
		else
		{
			m_deformableBodySolver->m_useProjection = false;
			m_deformableBodySolver->m_objective->m_projection.m_useStrainLimiting = false;
			m_deformableBodySolver->m_objective->m_preconditioner = m_deformableBodySolver->m_objective->m_KKTPreconditioner;
		}
	}

	void applyRigidBodyGravity(btScalar timeStep)
	{
		// Gravity is applied in stepSimulation and then cleared here and then applied here and then cleared here again
		// so that 1) gravity is applied to velocity before constraint solve and 2) gravity is applied in each substep
		// when there are multiple substeps
		btMultiBodyDynamicsWorld::applyGravity();
		// integrate rigid body gravity
		for (int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
		{
			btRigidBody* rb = m_nonStaticRigidBodies[i];
			rb->integrateVelocities(timeStep);
		}

		// integrate multibody gravity
		{
			forwardKinematics();
			clearMultiBodyConstraintForces();
			{
				for (int i = 0; i < this->m_multiBodies.size(); i++)
				{
					btMultiBody* bod = m_multiBodies[i];

					bool isSleeping = false;

					if (bod->getBaseCollider() && bod->getBaseCollider()->getActivationState() == ISLAND_SLEEPING)
					{
						isSleeping = true;
					}
					for (int b = 0; b < bod->getNumLinks(); b++)
					{
						if (bod->getLink(b).m_collider && bod->getLink(b).m_collider->getActivationState() == ISLAND_SLEEPING)
							isSleeping = true;
					}

					if (!isSleeping)
					{
						m_scratch_r.resize(bod->getNumLinks() + 1);
						m_scratch_v.resize(bod->getNumLinks() + 1);
						m_scratch_m.resize(bod->getNumLinks() + 1);
						bool isConstraintPass = false;
						{
							if (!bod->isUsingRK4Integration())
							{
								bod->computeAccelerationsArticulatedBodyAlgorithmMultiDof(m_solverInfo.m_timeStep,
																						  m_scratch_r, m_scratch_v, m_scratch_m, isConstraintPass,
																						  getSolverInfo().m_jointFeedbackInWorldSpace,
																						  getSolverInfo().m_jointFeedbackInJointFrame);
							}
							else
							{
								btAssert(" RK4Integration is not supported");
							}
						}
					}
				}
			}
		}
		clearGravity();
	}

	void beforeSolverCallbacks(btScalar timeStep)
	{
		if (0 != m_internalTickCallback)
		{
			(*m_internalTickCallback)(this, timeStep);
		}

		if (0 != m_solverCallback)
		{
			(*m_solverCallback)(m_internalTime, this);
		}
	}

	void afterSolverCallbacks(btScalar timeStep)
	{
		if (0 != m_solverCallback)
		{
			(*m_solverCallback)(m_internalTime, this);
		}
	}

	void addForce(btSoftBody* psb, btDeformableLagrangianForce* force)
	{
		btAlignedObjectArray<btDeformableLagrangianForce*>& forces = m_deformableBodySolver->m_objective->m_lf;
		bool added = false;
		for (int i = 0; i < forces.size(); ++i)
		{
			if (forces[i]->getForceType() == force->getForceType())
			{
				forces[i]->addSoftBody(psb);
				added = true;
				break;
			}
		}
		if (!added)
		{
			force->addSoftBody(psb);
			force->setIndices(m_deformableBodySolver->m_objective->getIndices());
			forces.push_back(force);
		}
	}

	void removeForce(btSoftBody* psb, btDeformableLagrangianForce* force)
	{
		btAlignedObjectArray<btDeformableLagrangianForce*>& forces = m_deformableBodySolver->m_objective->m_lf;
		int removed_index = -1;
		for (int i = 0; i < forces.size(); ++i)
		{
			if (forces[i]->getForceType() == force->getForceType())
			{
				forces[i]->removeSoftBody(psb);
				if (forces[i]->m_softBodies.size() == 0)
					removed_index = i;
				break;
			}
		}
		if (removed_index >= 0)
			forces.removeAtIndex(removed_index);
	}

	void removeSoftBodyForce(btSoftBody* psb)
	{
		btAlignedObjectArray<btDeformableLagrangianForce*>& forces = m_deformableBodySolver->m_objective->m_lf;
		for (int i = 0; i < forces.size(); ++i)
		{
			forces[i]->removeSoftBody(psb);
		}
	}

	void removeSoftBody(btSoftBody* body)
	{
		removeSoftBodyForce(body);
		m_softBodies.remove(body);
		btCollisionWorld::removeCollisionObject(body);
		// force a reinitialize so that node indices get updated.
		m_deformableBodySolver->reinitialize(m_softBodies, btScalar(-1));
	}

	void removeCollisionObject(btCollisionObject* collisionObject)
	{
		btSoftBody* body = btSoftBody::upcast(collisionObject);
		if (body)
			removeSoftBody(body);
		else
			btDiscreteDynamicsWorld::removeCollisionObject(collisionObject);
	}

	int getDrawFlags() const { return (m_drawFlags); }
	void setDrawFlags(int f) { m_drawFlags = f; }

	void setupConstraints()
	{
		// set up constraints between multibody and deformable bodies
		m_deformableBodySolver->setConstraints(m_solverInfo);

		// set up constraints among multibodies
		{
			sortConstraints();
			// setup the solver callback
			btMultiBodyConstraint** sortedMultiBodyConstraints = m_sortedMultiBodyConstraints.size() ? &m_sortedMultiBodyConstraints[0] : 0;
			btTypedConstraint** constraintsPtr = getNumConstraints() ? &m_sortedConstraints[0] : 0;
			m_solverDeformableBodyIslandCallback->setup(&m_solverInfo, constraintsPtr, m_sortedConstraints.size(), sortedMultiBodyConstraints, m_sortedMultiBodyConstraints.size(), getDebugDrawer());

			// build islands
			m_islandManager->buildIslands(getCollisionWorld()->getDispatcher(), getCollisionWorld());
		}
	}

	void performDeformableCollisionDetection()
	{
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			m_softBodies[i]->m_softSoftCollision = true;
		}

		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			for (int j = i; j < m_softBodies.size(); ++j)
			{
				m_softBodies[i]->defaultCollisionHandler(m_softBodies[j]);
			}
		}

		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			m_softBodies[i]->m_softSoftCollision = false;
		}
	}

	void solveMultiBodyConstraints();

	void solveContactConstraints()
	{
		// process constraints on each island
		m_islandManager->processIslands(getCollisionWorld()->getDispatcher(), getCollisionWorld(), m_solverDeformableBodyIslandCallback);

		// process deferred
		m_solverDeformableBodyIslandCallback->processConstraints();
		m_constraintSolver->allSolved(m_solverInfo, m_debugDrawer);

		// write joint feedback
		{
			for (int i = 0; i < this->m_multiBodies.size(); i++)
			{
				btMultiBody* bod = m_multiBodies[i];

				bool isSleeping = false;

				if (bod->getBaseCollider() && bod->getBaseCollider()->getActivationState() == ISLAND_SLEEPING)
				{
					isSleeping = true;
				}
				for (int b = 0; b < bod->getNumLinks(); b++)
				{
					if (bod->getLink(b).m_collider && bod->getLink(b).m_collider->getActivationState() == ISLAND_SLEEPING)
						isSleeping = true;
				}

				if (!isSleeping)
				{
					//useless? they get resized in stepVelocities once again (AND DIFFERENTLY)
					m_scratch_r.resize(bod->getNumLinks() + 1);  //multidof? ("Y"s use it and it is used to store qdd)
					m_scratch_v.resize(bod->getNumLinks() + 1);
					m_scratch_m.resize(bod->getNumLinks() + 1);

					if (bod->internalNeedsJointFeedback())
					{
						if (!bod->isUsingRK4Integration())
						{
							if (bod->internalNeedsJointFeedback())
							{
								bool isConstraintPass = true;
								bod->computeAccelerationsArticulatedBodyAlgorithmMultiDof(m_solverInfo.m_timeStep, m_scratch_r, m_scratch_v, m_scratch_m, isConstraintPass,
																						  getSolverInfo().m_jointFeedbackInWorldSpace,
																						  getSolverInfo().m_jointFeedbackInJointFrame);
							}
						}
					}
				}
			}
		}

		for (int i = 0; i < this->m_multiBodies.size(); i++)
		{
			btMultiBody* bod = m_multiBodies[i];
			bod->processDeltaVeeMultiDof2();
		}
	}

	void sortConstraints()
	{
		m_sortedConstraints.resize(m_constraints.size());
		int i;
		for (i = 0; i < getNumConstraints(); i++)
		{
			m_sortedConstraints[i] = m_constraints[i];
		}
		m_sortedConstraints.quickSort(btSortConstraintOnIslandPredicate2());

		m_sortedMultiBodyConstraints.resize(m_multiBodyConstraints.size());
		for (i = 0; i < m_multiBodyConstraints.size(); i++)
		{
			m_sortedMultiBodyConstraints[i] = m_multiBodyConstraints[i];
		}
		m_sortedMultiBodyConstraints.quickSort(btSortMultiBodyConstraintOnIslandPredicate());
	}

	void softBodySelfCollision()
	{
		BT_PROFILE("btDeformableMultiBodyDynamicsWorld::softBodySelfCollision");
		for (int i = 0; i < m_softBodies.size(); i++)
		{
			btSoftBody* psb = m_softBodies[i];
			if (psb->isActive())
			{
				psb->defaultCollisionHandler(psb);
			}
		}
	}

	void setImplicit(bool implicit)
	{
		m_implicit = implicit;
	}

	void setLineSearch(bool lineSearch)
	{
		m_lineSearch = lineSearch;
	}

	void setUseProjection(bool useProjection)
	{
		m_useProjection = useProjection;
	}

	void applyRepulsionForce(btScalar timeStep)
	{
		BT_PROFILE("btDeformableMultiBodyDynamicsWorld::applyRepulsionForce");
		for (int i = 0; i < m_softBodies.size(); i++)
		{
			btSoftBody* psb = m_softBodies[i];
			if (psb->isActive())
			{
				psb->applyRepulsionForce(timeStep, true);
			}
		}
	}

	void performGeometricCollisions(btScalar timeStep)
	{
		BT_PROFILE("btDeformableMultiBodyDynamicsWorld::performGeometricCollisions");
		// refit the BVH tree for CCD
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			if (psb->isActive())
			{
				m_softBodies[i]->updateFaceTree(true, false);
				m_softBodies[i]->updateNodeTree(true, false);
				for (int j = 0; j < m_softBodies[i]->m_faces.size(); ++j)
				{
					btSoftBody::Face& f = m_softBodies[i]->m_faces[j];
					f.m_n0 = (f.m_n[1]->m_x - f.m_n[0]->m_x).cross(f.m_n[2]->m_x - f.m_n[0]->m_x);
				}
			}
		}

		// clear contact points & update DBVT
		for (int r = 0; r < m_ccdIterations; ++r)
		{
			for (int i = 0; i < m_softBodies.size(); ++i)
			{
				btSoftBody* psb = m_softBodies[i];
				if (psb->isActive())
				{
					// clear contact points in the previous iteration
					psb->m_faceNodeContacts.clear();

					// update m_q and normals for CCD calculation
					for (int j = 0; j < psb->m_nodes.size(); ++j)
					{
						psb->m_nodes[j].m_q = psb->m_nodes[j].m_x + timeStep * psb->m_nodes[j].m_v;
					}
					for (int j = 0; j < psb->m_faces.size(); ++j)
					{
						btSoftBody::Face& f = psb->m_faces[j];
						f.m_n1 = (f.m_n[1]->m_q - f.m_n[0]->m_q).cross(f.m_n[2]->m_q - f.m_n[0]->m_q);
						f.m_vn = (f.m_n[1]->m_v - f.m_n[0]->m_v).cross(f.m_n[2]->m_v - f.m_n[0]->m_v) * timeStep * timeStep;
					}
				}
			}

			// apply CCD to register new contact points
			for (int i = 0; i < m_softBodies.size(); ++i)
			{
				for (int j = i; j < m_softBodies.size(); ++j)
				{
					btSoftBody* psb1 = m_softBodies[i];
					btSoftBody* psb2 = m_softBodies[j];
					if (psb1->isActive() && psb2->isActive())
					{
						m_softBodies[i]->geometricCollisionHandler(m_softBodies[j]);
					}
				}
			}

			int penetration_count = 0;
			for (int i = 0; i < m_softBodies.size(); ++i)
			{
				btSoftBody* psb = m_softBodies[i];
				if (psb->isActive())
				{
					penetration_count += psb->m_faceNodeContacts.size();
				}
			}
			if (penetration_count == 0)
			{
				break;
			}

			// apply inelastic impulse
			for (int i = 0; i < m_softBodies.size(); ++i)
			{
				btSoftBody* psb = m_softBodies[i];
				if (psb->isActive())
				{
					psb->applyRepulsionForce(timeStep, false);
				}
			}
		}
	}

	struct btDeformableSingleRayCallback : public btBroadphaseRayCallback
	{
		btVector3 m_rayFromWorld;
		btVector3 m_rayToWorld;
		btTransform m_rayFromTrans;
		btTransform m_rayToTrans;
		btVector3 m_hitNormal;

		const btDeformableMultiBodyDynamicsWorld* m_world;
		btCollisionWorld::RayResultCallback& m_resultCallback;

		btDeformableSingleRayCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld, const btDeformableMultiBodyDynamicsWorld* world, btCollisionWorld::RayResultCallback& resultCallback)
			: m_rayFromWorld(rayFromWorld),
			  m_rayToWorld(rayToWorld),
			  m_world(world),
			  m_resultCallback(resultCallback)
		{
			m_rayFromTrans.setIdentity();
			m_rayFromTrans.setOrigin(m_rayFromWorld);
			m_rayToTrans.setIdentity();
			m_rayToTrans.setOrigin(m_rayToWorld);

			btVector3 rayDir = (rayToWorld - rayFromWorld);

			rayDir.normalize();
			///what about division by zero? --> just set rayDirection[i] to INF/1e30
			m_rayDirectionInverse[0] = rayDir[0] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[0];
			m_rayDirectionInverse[1] = rayDir[1] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[1];
			m_rayDirectionInverse[2] = rayDir[2] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[2];
			m_signs[0] = m_rayDirectionInverse[0] < 0.0;
			m_signs[1] = m_rayDirectionInverse[1] < 0.0;
			m_signs[2] = m_rayDirectionInverse[2] < 0.0;

			m_lambda_max = rayDir.dot(m_rayToWorld - m_rayFromWorld);
		}

		virtual bool process(const btBroadphaseProxy* proxy)
		{
			///terminate further ray tests, once the closestHitFraction reached zero
			if (m_resultCallback.m_closestHitFraction == btScalar(0.f))
				return false;

			btCollisionObject* collisionObject = (btCollisionObject*)proxy->m_clientObject;

			//only perform raycast if filterMask matches
			if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
			{
				//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
				//btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
#if 0
#ifdef RECALCULATE_AABB
                btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
                collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
#else
                //getBroadphase()->getAabb(collisionObject->getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
                const btVector3& collisionObjectAabbMin = collisionObject->getBroadphaseHandle()->m_aabbMin;
                const btVector3& collisionObjectAabbMax = collisionObject->getBroadphaseHandle()->m_aabbMax;
#endif
#endif
				//btScalar hitLambda = m_resultCallback.m_closestHitFraction;
				//culling already done by broadphase
				//if (btRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
				{
					m_world->rayTestSingle(m_rayFromTrans, m_rayToTrans,
										   collisionObject,
										   collisionObject->getCollisionShape(),
										   collisionObject->getWorldTransform(),
										   m_resultCallback);
				}
			}
			return true;
		}
	};

	void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const
	{
		BT_PROFILE("rayTest");
		/// use the broadphase to accelerate the search for objects, based on their aabb
		/// and for each object with ray-aabb overlap, perform an exact ray test
		btDeformableSingleRayCallback rayCB(rayFromWorld, rayToWorld, this, resultCallback);

#ifndef USE_BRUTEFORCE_RAYBROADPHASE
		m_broadphasePairCache->rayTest(rayFromWorld, rayToWorld, rayCB);
#else
		for (int i = 0; i < this->getNumCollisionObjects(); i++)
		{
			rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
		}
#endif  //USE_BRUTEFORCE_RAYBROADPHASE
	}

	void rayTestSingle(const btTransform& rayFromTrans, const btTransform& rayToTrans,
					   btCollisionObject* collisionObject,
					   const btCollisionShape* collisionShape,
					   const btTransform& colObjWorldTransform,
					   RayResultCallback& resultCallback) const
	{
		if (collisionShape->isSoftBody())
		{
			btSoftBody* softBody = btSoftBody::upcast(collisionObject);
			if (softBody)
			{
				btSoftBody::sRayCast softResult;
				if (softBody->rayFaceTest(rayFromTrans.getOrigin(), rayToTrans.getOrigin(), softResult))
				{
					if (softResult.fraction <= resultCallback.m_closestHitFraction)
					{
						btCollisionWorld::LocalShapeInfo shapeInfo;
						shapeInfo.m_shapePart = 0;
						shapeInfo.m_triangleIndex = softResult.index;
						// get the normal
						btVector3 rayDir = rayToTrans.getOrigin() - rayFromTrans.getOrigin();
						btVector3 normal = -rayDir;
						normal.normalize();
						{
							normal = softBody->m_faces[softResult.index].m_normal;
							if (normal.dot(rayDir) > 0)
							{
								// normal always point toward origin of the ray
								normal = -normal;
							}
						}

						btCollisionWorld::LocalRayResult rayResult(collisionObject,
																   &shapeInfo,
																   normal,
																   softResult.fraction);
						bool normalInWorldSpace = true;
						resultCallback.addSingleResult(rayResult, normalInWorldSpace);
					}
				}
			}
		}
		else
		{
			btCollisionWorld::rayTestSingle(rayFromTrans, rayToTrans, collisionObject, collisionShape, colObjWorldTransform, resultCallback);
		}
	}
};

#endif  //BT_DEFORMABLE_MULTIBODY_DYNAMICS_WORLD_H
