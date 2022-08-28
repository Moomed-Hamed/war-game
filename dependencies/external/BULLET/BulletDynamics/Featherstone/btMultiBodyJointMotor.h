#ifndef BT_MULTIBODY_JOINT_MOTOR_H
#define BT_MULTIBODY_JOINT_MOTOR_H

#include "btMultiBodyConstraint.h"
#include "btMultiBodyLinkCollider.h"
struct btSolverInfo;

class btMultiBodyJointMotor : public btMultiBodyConstraint
{
protected:
	btScalar m_desiredVelocity;
	btScalar m_desiredPosition;
	btScalar m_kd;
	btScalar m_kp;
	btScalar m_erp;
	btScalar m_rhsClamp;  //maximum error

public:
	btMultiBodyJointMotor(btMultiBody* body, int link, btScalar desiredVelocity, btScalar maxMotorImpulse)
		: btMultiBodyConstraint(body, body, link, body->getLink(link).m_parent, 1, true, MULTIBODY_CONSTRAINT_1DOF_JOINT_MOTOR),
		  m_desiredVelocity(desiredVelocity),
		  m_desiredPosition(0),
		  m_kd(1.),
		  m_kp(0),
		  m_erp(1),
		  m_rhsClamp(SIMD_INFINITY)
	{
		m_maxAppliedImpulse = maxMotorImpulse;
		// data.m_jacobians never change, so may as well initialize them here
	}
	btMultiBodyJointMotor(btMultiBody* body, int link, int linkDoF, btScalar desiredVelocity, btScalar maxMotorImpulse)
		: btMultiBodyConstraint(body, body, link, body->getLink(link).m_parent, 1, true, MULTIBODY_CONSTRAINT_1DOF_JOINT_MOTOR),
		  m_desiredVelocity(desiredVelocity),
		  m_desiredPosition(0),
		  m_kd(1.),
		  m_kp(0),
		  m_erp(1),
		  m_rhsClamp(SIMD_INFINITY)
	{
		btAssert(linkDoF < body->getLink(link).m_dofCount);

		m_maxAppliedImpulse = maxMotorImpulse;
	}
	virtual ~btMultiBodyJointMotor() {}
	virtual void finalizeMultiDof()
	{
		allocateJacobiansMultiDof();
		// note: we rely on the fact that data.m_jacobians are always initialized to zero by the Constraint ctor
		int linkDoF = 0;
		unsigned int offset = 6 + (m_bodyA->getLink(m_linkA).m_dofOffset + linkDoF);

		// row 0: the lower bound
		// row 0: the lower bound
		jacobianA(0)[offset] = 1;

		m_numDofsFinalized = m_jacSizeBoth;
	}

	virtual int getIslandIdA() const
	{
		if (this->m_linkA < 0)
		{
			btMultiBodyLinkCollider* col = m_bodyA->getBaseCollider();
			if (col)
				return col->getIslandTag();
		}
		else
		{
			if (m_bodyA->getLink(m_linkA).m_collider)
			{
				return m_bodyA->getLink(m_linkA).m_collider->getIslandTag();
			}
		}
		return -1;
	}
	virtual int getIslandIdB() const
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
			{
				return m_bodyB->getLink(m_linkB).m_collider->getIslandTag();
			}
		}
		return -1;
	}

	virtual void createConstraintRows(btMultiBodyConstraintArray& constraintRows, btMultiBodyJacobianData& data, const btContactSolverInfo& infoGlobal)
	{
		// only positions need to be updated -- data.m_jacobians and force
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

		const btScalar posError = 0;
		const btVector3 dummy(0, 0, 0);

		for (int row = 0; row < getNumRows(); row++)
		{
			btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();

			int dof = 0;
			btScalar currentPosition = m_bodyA->getJointPosMultiDof(m_linkA)[dof];
			btScalar currentVelocity = m_bodyA->getJointVelMultiDof(m_linkA)[dof];
			btScalar positionStabiliationTerm = m_erp * (m_desiredPosition - currentPosition) / infoGlobal.m_timeStep;

			btScalar velocityError = (m_desiredVelocity - currentVelocity);
			btScalar rhs = m_kp * positionStabiliationTerm + currentVelocity + m_kd * velocityError;
			if (rhs > m_rhsClamp)
			{
				rhs = m_rhsClamp;
			}
			if (rhs < -m_rhsClamp)
			{
				rhs = -m_rhsClamp;
			}

			fillMultiBodyConstraint(constraintRow, data, jacobianA(row), jacobianB(row), dummy, dummy, dummy, dummy, posError, infoGlobal, -m_maxAppliedImpulse, m_maxAppliedImpulse, false, 1, false, rhs);
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

	virtual void setVelocityTarget(btScalar velTarget, btScalar kd = 1.f)
	{
		m_desiredVelocity = velTarget;
		m_kd = kd;
	}

	virtual void setPositionTarget(btScalar posTarget, btScalar kp = 1.f)
	{
		m_desiredPosition = posTarget;
		m_kp = kp;
	}

	virtual void setErp(btScalar erp)
	{
		m_erp = erp;
	}
	virtual btScalar getErp() const
	{
		return m_erp;
	}
	virtual void setRhsClamp(btScalar rhsClamp)
	{
		m_rhsClamp = rhsClamp;
	}
	virtual void debugDraw(class btIDebugDraw* drawer)
	{
		//todo(erwincoumans)
	}
};

#endif  //BT_MULTIBODY_JOINT_MOTOR_H
