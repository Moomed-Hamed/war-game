/*
	Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
	Bullet Continuous Collision Detection and Physics Library
	Copyright (c) 2019 Google Inc. http://bulletphysics.org
*/

#ifndef BT_DEFORMABLE_MULTIBODY_CONSTRAINT_SOLVER_H
#define BT_DEFORMABLE_MULTIBODY_CONSTRAINT_SOLVER_H

#include "btDeformableBodySolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

class btDeformableBodySolver;

// btDeformableMultiBodyConstraintSolver extendsn btMultiBodyConstraintSolver to solve for the contact among rigid/multibody and deformable bodies. Notice that the following constraints
// 1. rigid/multibody against rigid/multibody
// 2. rigid/multibody against deforamble
// 3. deformable against deformable
// 4. deformable self collision
// 5. joint constraints
// are all coupled in this solve.
ATTRIBUTE_ALIGNED16(class)
btDeformableMultiBodyConstraintSolver : public btMultiBodyConstraintSolver
{
	btDeformableBodySolver* m_deformableSolver;

protected:
	// override the iterations method to include deformable/multibody contact
	//    virtual btScalar solveGroupCacheFriendlyIterations(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);

	// write the velocity of the the solver body to the underlying rigid body
	void solverBodyWriteBack(const btContactSolverInfo& infoGlobal)
	{
		for (int i = 0; i < m_tmpSolverBodyPool.size(); i++)
		{
			btRigidBody* body = m_tmpSolverBodyPool[i].m_originalBody;
			if (body)
			{
				m_tmpSolverBodyPool[i].m_originalBody->setLinearVelocity(m_tmpSolverBodyPool[i].m_linearVelocity + m_tmpSolverBodyPool[i].m_deltaLinearVelocity);
				m_tmpSolverBodyPool[i].m_originalBody->setAngularVelocity(m_tmpSolverBodyPool[i].m_angularVelocity + m_tmpSolverBodyPool[i].m_deltaAngularVelocity);
			}
		}
	}

	// write the velocity of the underlying rigid body to the the the solver body
	void writeToSolverBody(btCollisionObject * *bodies, int numBodies, const btContactSolverInfo& infoGlobal)
	{
		for (int i = 0; i < numBodies; i++)
		{
			int bodyId = getOrInitSolverBody(*bodies[i], infoGlobal.m_timeStep);

			btRigidBody* body = btRigidBody::upcast(bodies[i]);
			if (body && body->getInvMass())
			{
				btSolverBody& solverBody = m_tmpSolverBodyPool[bodyId];
				solverBody.m_linearVelocity = body->getLinearVelocity() - solverBody.m_deltaLinearVelocity;
				solverBody.m_angularVelocity = body->getAngularVelocity() - solverBody.m_deltaAngularVelocity;
			}
		}
	}

	virtual void solveGroupCacheFriendlySplitImpulseIterations(btCollisionObject * *bodies, int numBodies, btCollisionObject** deformableBodies, int numDeformableBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
	{
		BT_PROFILE("solveGroupCacheFriendlySplitImpulseIterations");
		int iteration;
		if (infoGlobal.m_splitImpulse)
		{
			{
				for (iteration = 0; iteration < infoGlobal.m_numIterations; iteration++)
				{
					btScalar leastSquaresResidual = 0.f;
					{
						int numPoolConstraints = m_tmpSolverContactConstraintPool.size();
						int j;
						for (j = 0; j < numPoolConstraints; j++)
						{
							const btSolverConstraint& solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];

							btScalar residual = resolveSplitPenetrationImpulse(m_tmpSolverBodyPool[solveManifold.m_solverBodyIdA], m_tmpSolverBodyPool[solveManifold.m_solverBodyIdB], solveManifold);
							leastSquaresResidual = btMax(leastSquaresResidual, residual * residual);
						}
						// solve the position correction between deformable and rigid/multibody
						//                    btScalar residual = m_deformableSolver->solveSplitImpulse(infoGlobal);
						btScalar residual = m_deformableSolver->m_objective->m_projection.solveSplitImpulse(deformableBodies, numDeformableBodies, infoGlobal);
						leastSquaresResidual = btMax(leastSquaresResidual, residual * residual);
					}
					if (leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || iteration >= (infoGlobal.m_numIterations - 1))
					{
#ifdef VERBOSE_RESIDUAL_PRINTF
						if (iteration >= (infoGlobal.m_numIterations - 1))
							printf("split impulse residual = %f at iteration #%d\n", leastSquaresResidual, iteration);
#endif
						break;
					}
				}
			}
		}
	}

	// override the iterations method to include deformable/multibody contact
	virtual btScalar solveDeformableGroupIterations(btCollisionObject * *bodies, int numBodies, btCollisionObject** deformableBodies, int numDeformableBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
	{
		{
			///this is a special step to resolve penetrations (just for contacts)
			solveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, deformableBodies, numDeformableBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);

			int maxIterations = m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations ? m_maxOverrideNumSolverIterations : infoGlobal.m_numIterations;
			for (int iteration = 0; iteration < maxIterations; iteration++)
			{
				// rigid bodies are solved using solver body velocity, but rigid/deformable contact directly uses the velocity of the actual rigid body. So we have to do the following: Solve one iteration of the rigid/rigid contact, get the updated velocity in the solver body and update the velocity of the underlying rigid body. Then solve the rigid/deformable contact. Finally, grab the (once again) updated rigid velocity and update the velocity of the wrapping solver body

				// solve rigid/rigid in solver body
				m_leastSquaresResidual = solveSingleIteration(iteration, bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
				// solver body velocity -> rigid body velocity
				solverBodyWriteBack(infoGlobal);
				btScalar deformableResidual = m_deformableSolver->solveContactConstraints(deformableBodies, numDeformableBodies, infoGlobal);
				// update rigid body velocity in rigid/deformable contact
				m_leastSquaresResidual = btMax(m_leastSquaresResidual, deformableResidual);
				// solver body velocity <- rigid body velocity
				writeToSolverBody(bodies, numBodies, infoGlobal);

				if (m_leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || (iteration >= (maxIterations - 1)))
				{
#ifdef VERBOSE_RESIDUAL_PRINTF
					if (iteration >= (maxIterations - 1))
						printf("residual = %f at iteration #%d\n", m_leastSquaresResidual, iteration);
#endif
					m_analyticsData.m_numSolverCalls++;
					m_analyticsData.m_numIterationsUsed = iteration + 1;
					m_analyticsData.m_islandId = -2;
					if (numBodies > 0)
						m_analyticsData.m_islandId = bodies[0]->getCompanionId();
					m_analyticsData.m_numBodies = numBodies;
					m_analyticsData.m_numContactManifolds = numManifolds;
					m_analyticsData.m_remainingLeastSquaresResidual = m_leastSquaresResidual;
					break;
				}
			}
		}
		return 0.f;
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	void setDeformableSolver(btDeformableBodySolver * deformableSolver)
	{
		m_deformableSolver = deformableSolver;
	}

	virtual void solveDeformableBodyGroup(btCollisionObject * *bodies, int numBodies, btCollisionObject** deformableBodies, int numDeformableBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher)
	{
		m_tmpMultiBodyConstraints = multiBodyConstraints;
		m_tmpNumMultiBodyConstraints = numMultiBodyConstraints;

		// inherited from MultiBodyConstraintSolver
		solveGroupCacheFriendlySetup(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

		// overriden
		solveDeformableGroupIterations(bodies, numBodies, deformableBodies, numDeformableBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

		// inherited from MultiBodyConstraintSolver
		solveGroupCacheFriendlyFinish(bodies, numBodies, info);

		m_tmpMultiBodyConstraints = 0;
		m_tmpNumMultiBodyConstraints = 0;
	}
};

#endif /* BT_DEFORMABLE_MULTIBODY_CONSTRAINT_SOLVER_H */
