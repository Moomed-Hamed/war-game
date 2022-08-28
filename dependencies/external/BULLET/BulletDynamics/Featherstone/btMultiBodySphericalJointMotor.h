// This file was written by Erwin Coumans

#ifndef BT_MULTIBODY_SPHERICAL_JOINT_MOTOR_H
#define BT_MULTIBODY_SPHERICAL_JOINT_MOTOR_H

#include "btMultiBodyConstraint.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
struct btSolverInfo;

class btMultiBodySphericalJointMotor : public btMultiBodyConstraint
{
protected:
	btVector3 m_desiredVelocity;
	btQuaternion m_desiredPosition;
	bool m_use_multi_dof_params;
	btVector3 m_kd;
	btVector3 m_kp;
	btScalar m_erp;
	btScalar m_rhsClamp;  //maximum error
	btVector3 m_maxAppliedImpulseMultiDof;
	btVector3 m_damping;

public:
	btMultiBodySphericalJointMotor(btMultiBody* body, int link, btScalar maxMotorImpulse)
		: btMultiBodyConstraint(body, body, link, body->getLink(link).m_parent, 3, true, MULTIBODY_CONSTRAINT_SPHERICAL_MOTOR),
		  m_desiredVelocity(0, 0, 0),
		  m_desiredPosition(0, 0, 0, 1),
		  m_use_multi_dof_params(false),
		  m_kd(1., 1., 1.),
		  m_kp(0.2, 0.2, 0.2),
		  m_erp(1),
		  m_rhsClamp(SIMD_INFINITY),
		  m_maxAppliedImpulseMultiDof(maxMotorImpulse, maxMotorImpulse, maxMotorImpulse),
		  m_damping(1.0, 1.0, 1.0)
	{
		m_maxAppliedImpulse = maxMotorImpulse;
	}
	
	virtual ~btMultiBodySphericalJointMotor() {}
	virtual void finalizeMultiDof()
	{
		allocateJacobiansMultiDof();
		// note: we rely on the fact that data.m_jacobians are
		// always initialized to zero by the Constraint ctor
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

		btVector3 axis[3] = {btVector3(1, 0, 0), btVector3(0, 1, 0), btVector3(0, 0, 1)};

		btQuaternion desiredQuat = m_desiredPosition;
		btQuaternion currentQuat(m_bodyA->getJointPosMultiDof(m_linkA)[0],
								 m_bodyA->getJointPosMultiDof(m_linkA)[1],
								 m_bodyA->getJointPosMultiDof(m_linkA)[2],
								 m_bodyA->getJointPosMultiDof(m_linkA)[3]);

		btQuaternion relRot = currentQuat.inverse() * desiredQuat;
		btVector3 angleDiff;
		btGeneric6DofSpring2Constraint::matrixToEulerXYZ(btMatrix3x3(relRot), angleDiff);

		for (int row = 0; row < getNumRows(); row++)
		{
			btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();

			int dof = row;

			btScalar currentVelocity = m_bodyA->getJointVelMultiDof(m_linkA)[dof];
			btScalar desiredVelocity = this->m_desiredVelocity[row];

			double kd = m_use_multi_dof_params ? m_kd[row % 3] : m_kd[0];
			btScalar velocityError = (desiredVelocity - currentVelocity) * kd;

			btMatrix3x3 frameAworld;
			frameAworld.setIdentity();
			frameAworld = m_bodyA->localFrameToWorld(m_linkA, frameAworld);
			btScalar posError = 0;
			{
				btAssert(m_bodyA->getLink(m_linkA).m_jointType == btMultibodyLink::eSpherical);
				switch (m_bodyA->getLink(m_linkA).m_jointType)
				{
					case btMultibodyLink::eSpherical:
					{
						btVector3 constraintNormalAng = frameAworld.getColumn(row % 3);
						double kp = m_use_multi_dof_params ? m_kp[row % 3] : m_kp[0];
						posError = kp * angleDiff[row % 3];
						double max_applied_impulse = m_use_multi_dof_params ? m_maxAppliedImpulseMultiDof[row % 3] : m_maxAppliedImpulse;
						fillMultiBodyConstraint(constraintRow, data, 0, 0, constraintNormalAng,
												btVector3(0, 0, 0), dummy, dummy,
												posError,
												infoGlobal,
												-max_applied_impulse, max_applied_impulse, true,
												1.0, false, 0, 0,
												m_damping[row % 3]);
						constraintRow.m_orgConstraint = this;
						constraintRow.m_orgDofIndex = row;
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

	virtual void setVelocityTarget(const btVector3& velTarget, btScalar kd = 1.0)
	{
		m_desiredVelocity = velTarget;
		m_kd = btVector3(kd, kd, kd);
		m_use_multi_dof_params = false;
	}

	virtual void setVelocityTargetMultiDof(const btVector3& velTarget, const btVector3& kd = btVector3(1.0, 1.0, 1.0))
	{
		m_desiredVelocity = velTarget;
		m_kd = kd;
		m_use_multi_dof_params = true;
	}

	virtual void setPositionTarget(const btQuaternion& posTarget, btScalar kp =1.f)
	{
		m_desiredPosition = posTarget;
		m_kp = btVector3(kp, kp, kp);
		m_use_multi_dof_params = false;
	}

	virtual void setPositionTargetMultiDof(const btQuaternion& posTarget, const btVector3& kp = btVector3(1.f, 1.f, 1.f))
	{
		m_desiredPosition = posTarget;
		m_kp = kp;
		m_use_multi_dof_params = true;
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

	btScalar getMaxAppliedImpulseMultiDof(int i) const
	{
		return m_maxAppliedImpulseMultiDof[i];
	}

	void setMaxAppliedImpulseMultiDof(const btVector3& maxImp)
	{
		m_maxAppliedImpulseMultiDof = maxImp;
		m_use_multi_dof_params = true;
	}

	btScalar getDamping(int i) const
	{
		return m_damping[i];
	}

	void setDamping(const btVector3& damping)
	{
		m_damping = damping;
	}

	virtual void debugDraw(class btIDebugDraw* drawer)
	{
		//todo(erwincoumans)
	}
};

#endif  //BT_MULTIBODY_SPHERICAL_JOINT_MOTOR_H
