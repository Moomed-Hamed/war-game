// This file was written by Erwin Coumans

#ifndef BT_MULTIBODY_GEAR_CONSTRAINT_H
#define BT_MULTIBODY_GEAR_CONSTRAINT_H

#include "btMultiBodyConstraint.h"
#include "btMultiBodyLinkCollider.h"

class btMultiBodyGearConstraint : public btMultiBodyConstraint
{
protected:
	btRigidBody* m_rigidBodyA;
	btRigidBody* m_rigidBodyB;
	btVector3 m_pivotInA;
	btVector3 m_pivotInB;
	btMatrix3x3 m_frameInA;
	btMatrix3x3 m_frameInB;
	btScalar m_gearRatio;
	int m_gearAuxLink;
	btScalar m_erp;
	btScalar m_relativePositionTarget;

public:
	btMultiBodyGearConstraint(btMultiBody* bodyA, int linkA, btMultiBody* bodyB, int linkB, const btVector3& pivotInA, const btVector3& pivotInB, const btMatrix3x3& frameInA, const btMatrix3x3& frameInB)
		: btMultiBodyConstraint(bodyA, bodyB, linkA, linkB, 1, false, MULTIBODY_CONSTRAINT_GEAR),
		  m_gearRatio(1),
		  m_gearAuxLink(-1),
		  m_erp(0),
		  m_relativePositionTarget(0)
	{
	}
	virtual ~btMultiBodyGearConstraint() {}

	virtual void finalizeMultiDof()
	{
		allocateJacobiansMultiDof();
		m_numDofsFinalized = m_jacSizeBoth;
	}

	virtual int getIslandIdA() const
	{
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
		// only positions need to be updated -- data.m_jacobians & force
		// directions were set in the ctor and never change.

		if (m_numDofsFinalized != m_jacSizeBoth)
		{
			finalizeMultiDof();
		}

		//don't crash
		if (m_numDofsFinalized != m_jacSizeBoth)
			return;

		if (m_maxAppliedImpulse == 0.f)
			return;

		// note: we rely on the fact that data.m_jacobians are
		// always initialized to zero by the Constraint ctor
		int linkDoF = 0;
		unsigned int offsetA = 6 + (m_bodyA->getLink(m_linkA).m_dofOffset + linkDoF);
		unsigned int offsetB = 6 + (m_bodyB->getLink(m_linkB).m_dofOffset + linkDoF);

		// row 0: the lower bound
		jacobianA(0)[offsetA] = 1;
		jacobianB(0)[offsetB] = m_gearRatio;

		btScalar posError = 0;
		const btVector3 dummy(0, 0, 0);

		btScalar kp = 1;
		btScalar kd = 1;
		int numRows = getNumRows();

		for (int row = 0; row < numRows; row++)
		{
			btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();

			int dof = 0;
			btScalar currentPosition = m_bodyA->getJointPosMultiDof(m_linkA)[dof];
			btScalar currentVelocity = m_bodyA->getJointVelMultiDof(m_linkA)[dof];
			btScalar auxVel = 0;

			if (m_gearAuxLink >= 0)
			{
				auxVel = m_bodyA->getJointVelMultiDof(m_gearAuxLink)[dof];
			}
			currentVelocity += auxVel;
			if (m_erp != 0)
			{
				btScalar currentPositionA = m_bodyA->getJointPosMultiDof(m_linkA)[dof];
				if (m_gearAuxLink >= 0)
				{
					currentPositionA -= m_bodyA->getJointPosMultiDof(m_gearAuxLink)[dof];
				}
				btScalar currentPositionB = m_gearRatio * m_bodyA->getJointPosMultiDof(m_linkB)[dof];
				btScalar diff = currentPositionB + currentPositionA;
				btScalar desiredPositionDiff = this->m_relativePositionTarget;
				posError = -m_erp * (desiredPositionDiff - diff);
			}

			btScalar desiredRelativeVelocity = auxVel;

			fillMultiBodyConstraint(constraintRow, data, jacobianA(row), jacobianB(row), dummy, dummy, dummy, dummy, posError, infoGlobal, -m_maxAppliedImpulse, m_maxAppliedImpulse, false, 1, false, desiredRelativeVelocity);

			constraintRow.m_orgConstraint = this;
			constraintRow.m_orgDofIndex = row;
			{
				//expect either prismatic or revolute joint type for now
				btAssert((m_bodyA->getLink(m_linkA).m_jointType == btMultibodyLink::eRevolute) || (m_bodyA->getLink(m_linkA).m_jointType == btMultibodyLink::ePrismatic));
				switch (m_bodyA->getLink(m_linkA).m_jointType)
				{
					case btMultibodyLink::eRevolute:
					{
						constraintRow.m_contactNormal1.setZero();
						constraintRow.m_contactNormal2.setZero();
						btVector3 revoluteAxisInWorld = quatRotate(m_bodyA->getLink(m_linkA).m_cachedWorldTransform.getRotation(), m_bodyA->getLink(m_linkA).m_axes[0].m_topVec);
						constraintRow.m_relpos1CrossNormal = revoluteAxisInWorld;
						constraintRow.m_relpos2CrossNormal = -revoluteAxisInWorld;

						break;
					}
					case btMultibodyLink::ePrismatic:
					{
						btVector3 prismaticAxisInWorld = quatRotate(m_bodyA->getLink(m_linkA).m_cachedWorldTransform.getRotation(), m_bodyA->getLink(m_linkA).m_axes[0].m_bottomVec);
						constraintRow.m_contactNormal1 = prismaticAxisInWorld;
						constraintRow.m_contactNormal2 = -prismaticAxisInWorld;
						constraintRow.m_relpos1CrossNormal.setZero();
						constraintRow.m_relpos2CrossNormal.setZero();
						break;
					}
					default:
					{
						btAssert(0);
					}
				};
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
		//todo(erwincoumans)
	}

	virtual void setGearRatio(btScalar gearRatio)
	{
		m_gearRatio = gearRatio;
	}
	virtual void setGearAuxLink(int gearAuxLink)
	{
		m_gearAuxLink = gearAuxLink;
	}
	virtual void setRelativePositionTarget(btScalar relPosTarget)
	{
		m_relativePositionTarget = relPosTarget;
	}
	virtual void setErp(btScalar erp)
	{
		m_erp = erp;
	}
};

#endif  //BT_MULTIBODY_GEAR_CONSTRAINT_H
