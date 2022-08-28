// This file was written by Erwin Coumans

#ifndef BT_MULTIBODY_FIXED_CONSTRAINT_H
#define BT_MULTIBODY_FIXED_CONSTRAINT_H

#include "btMultiBodyConstraint.h"

#include "btMultiBodyLinkCollider.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "LinearMath/btIDebugDraw.h"

#define BTMBFIXEDCONSTRAINT_DIM 6

class btMultiBodyFixedConstraint : public btMultiBodyConstraint
{
protected:
	btRigidBody* m_rigidBodyA;
	btRigidBody* m_rigidBodyB;
	btVector3 m_pivotInA;
	btVector3 m_pivotInB;
	btMatrix3x3 m_frameInA;
	btMatrix3x3 m_frameInB;

public:
	btMultiBodyFixedConstraint(btMultiBody* body, int link, btRigidBody* bodyB, const btVector3& pivotInA, const btVector3& pivotInB, const btMatrix3x3& frameInA, const btMatrix3x3& frameInB)
		: btMultiBodyConstraint(body, 0, link, -1, BTMBFIXEDCONSTRAINT_DIM, false, MULTIBODY_CONSTRAINT_FIXED),
		  m_rigidBodyA(0),
		  m_rigidBodyB(bodyB),
		  m_pivotInA(pivotInA),
		  m_pivotInB(pivotInB),
		  m_frameInA(frameInA),
		  m_frameInB(frameInB)
	{
		m_data.resize(BTMBFIXEDCONSTRAINT_DIM);  //at least store the applied impulses
	}
	btMultiBodyFixedConstraint(btMultiBody* bodyA, int linkA, btMultiBody* bodyB, int linkB, const btVector3& pivotInA, const btVector3& pivotInB, const btMatrix3x3& frameInA, const btMatrix3x3& frameInB)
		: btMultiBodyConstraint(bodyA, bodyB, linkA, linkB, BTMBFIXEDCONSTRAINT_DIM, false, MULTIBODY_CONSTRAINT_FIXED),
		  m_rigidBodyA(0),
		  m_rigidBodyB(0),
		  m_pivotInA(pivotInA),
		  m_pivotInB(pivotInB),
		  m_frameInA(frameInA),
		  m_frameInB(frameInB)
	{
		m_data.resize(BTMBFIXEDCONSTRAINT_DIM);  //at least store the applied impulses
	}
	virtual ~btMultiBodyFixedConstraint() {}

	virtual void finalizeMultiDof()
	{
		//not implemented yet
		btAssert(0);
	}

	virtual int getIslandIdA() const
	{
		if (m_rigidBodyA)
			return m_rigidBodyA->getIslandTag();

		if (m_bodyA)
		{
			if (m_linkA < 0)
			{
				btMultiBodyLinkCollider* col = m_bodyA->getBaseCollider();
				if (col)
					return col->getIslandTag();
			}
			else
			{
				if (m_bodyA->getLink(m_linkA).m_collider)
					return m_bodyA->getLink(m_linkA).m_collider->getIslandTag();
			}
		}
		return -1;
	}
	virtual int getIslandIdB() const
	{
		if (m_rigidBodyB)
			return m_rigidBodyB->getIslandTag();
		if (m_bodyB)
		{
			if (m_linkB < 0)
			{
				btMultiBodyLinkCollider* col = m_bodyB->getBaseCollider();
				if (col)
					return col->getIslandTag();
			}
			else
			{
				if (m_bodyB->getLink(m_linkB).m_collider)
					return m_bodyB->getLink(m_linkB).m_collider->getIslandTag();
			}
		}
		return -1;
	}

	virtual void createConstraintRows(btMultiBodyConstraintArray& constraintRows, btMultiBodyJacobianData& data, const btContactSolverInfo& infoGlobal)
	{
		int numDim = BTMBFIXEDCONSTRAINT_DIM;
		for (int i = 0; i < numDim; i++)
		{
			btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();
			constraintRow.m_orgConstraint = this;
			constraintRow.m_orgDofIndex = i;
			constraintRow.m_relpos1CrossNormal.setValue(0, 0, 0);
			constraintRow.m_contactNormal1.setValue(0, 0, 0);
			constraintRow.m_relpos2CrossNormal.setValue(0, 0, 0);
			constraintRow.m_contactNormal2.setValue(0, 0, 0);
			constraintRow.m_angularComponentA.setValue(0, 0, 0);
			constraintRow.m_angularComponentB.setValue(0, 0, 0);

			constraintRow.m_solverBodyIdA = data.m_fixedBodyId;
			constraintRow.m_solverBodyIdB = data.m_fixedBodyId;

			// Convert local points back to world
			btVector3 pivotAworld = m_pivotInA;
			btMatrix3x3 frameAworld = m_frameInA;
			if (m_rigidBodyA)
			{
				constraintRow.m_solverBodyIdA = m_rigidBodyA->getCompanionId();
				pivotAworld = m_rigidBodyA->getCenterOfMassTransform() * m_pivotInA;
				frameAworld = frameAworld.transpose() * btMatrix3x3(m_rigidBodyA->getOrientation());
			}
			else
			{
				if (m_bodyA)
				{
					pivotAworld = m_bodyA->localPosToWorld(m_linkA, m_pivotInA);
					frameAworld = m_bodyA->localFrameToWorld(m_linkA, frameAworld);
				}
			}
			btVector3 pivotBworld = m_pivotInB;
			btMatrix3x3 frameBworld = m_frameInB;
			if (m_rigidBodyB)
			{
				constraintRow.m_solverBodyIdB = m_rigidBodyB->getCompanionId();
				pivotBworld = m_rigidBodyB->getCenterOfMassTransform() * m_pivotInB;
				frameBworld = frameBworld.transpose() * btMatrix3x3(m_rigidBodyB->getOrientation());
			}
			else
			{
				if (m_bodyB)
				{
					pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotInB);
					frameBworld = m_bodyB->localFrameToWorld(m_linkB, frameBworld);
				}
			}

			btMatrix3x3 relRot = frameAworld.inverse() * frameBworld;
			btVector3 angleDiff;
			btGeneric6DofSpring2Constraint::matrixToEulerXYZ(relRot, angleDiff);

			btVector3 constraintNormalLin(0, 0, 0);
			btVector3 constraintNormalAng(0, 0, 0);
			btScalar posError = 0.0;
			if (i < 3)
			{
				constraintNormalLin[i] = 1;
				posError = (pivotAworld - pivotBworld).dot(constraintNormalLin);
				fillMultiBodyConstraint(constraintRow, data, 0, 0, constraintNormalAng,
										constraintNormalLin, pivotAworld, pivotBworld,
										posError,
										infoGlobal,
										-m_maxAppliedImpulse, m_maxAppliedImpulse);
			}
			else
			{  //i>=3
				constraintNormalAng = frameAworld.getColumn(i % 3);
				posError = angleDiff[i % 3];
				fillMultiBodyConstraint(constraintRow, data, 0, 0, constraintNormalAng,
										constraintNormalLin, pivotAworld, pivotBworld,
										posError,
										infoGlobal,
										-m_maxAppliedImpulse, m_maxAppliedImpulse, true);
			}
		}
	}

	const btVector3& getPivotInA() const
	{
		return m_pivotInA;
	}

	void setPivotInA(const btVector3& pivotInA)
	{
		m_pivotInA = pivotInA;
	}

	const btVector3& getPivotInB() const
	{
		return m_pivotInB;
	}

	virtual void setPivotInB(const btVector3& pivotInB)
	{
		m_pivotInB = pivotInB;
	}

	const btMatrix3x3& getFrameInA() const
	{
		return m_frameInA;
	}

	void setFrameInA(const btMatrix3x3& frameInA)
	{
		m_frameInA = frameInA;
	}

	const btMatrix3x3& getFrameInB() const
	{
		return m_frameInB;
	}

	virtual void setFrameInB(const btMatrix3x3& frameInB)
	{
		m_frameInB = frameInB;
	}

	virtual void debugDraw(class btIDebugDraw* drawer)
	{
		btTransform tr;
		tr.setIdentity();

		if (m_rigidBodyA)
		{
			btVector3 pivot = m_rigidBodyA->getCenterOfMassTransform() * m_pivotInA;
			tr.setOrigin(pivot);
			drawer->drawTransform(tr, 0.1);
		}
		if (m_bodyA)
		{
			btVector3 pivotAworld = m_bodyA->localPosToWorld(m_linkA, m_pivotInA);
			tr.setOrigin(pivotAworld);
			drawer->drawTransform(tr, 0.1);
		}
		if (m_rigidBodyB)
		{
			// that ideally should draw the same frame
			btVector3 pivot = m_rigidBodyB->getCenterOfMassTransform() * m_pivotInB;
			tr.setOrigin(pivot);
			drawer->drawTransform(tr, 0.1);
		}
		if (m_bodyB)
		{
			btVector3 pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotInB);
			tr.setOrigin(pivotBworld);
			drawer->drawTransform(tr, 0.1);
		}
	}
};

#endif  //BT_MULTIBODY_FIXED_CONSTRAINT_H
