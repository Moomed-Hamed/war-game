// This file was written by Erwin Coumans

#ifndef BT_MULTIBODY_SPHERICAL_JOINT_LIMIT_H
#define BT_MULTIBODY_SPHERICAL_JOINT_LIMIT_H

#include "btMultiBodyConstraint.h"

#include "btMultiBody.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "LinearMath/btTransformUtil.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "LinearMath/btIDebugDraw.h"

struct btSolverInfo;

class btMultiBodySphericalJointLimit : public btMultiBodyConstraint
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
	btVector3 m_pivotA;
	btVector3 m_pivotB;
	btScalar m_swingxRange;
	btScalar m_swingyRange;
	btScalar m_twistRange;

public:
	btMultiBodySphericalJointLimit(btMultiBody* body, int link, btScalar swingxRange, btScalar swingyRange, btScalar twistRange, btScalar maxAppliedImpulse)
		: btMultiBodyConstraint(body, body, link, body->getLink(link).m_parent, 3, true, MULTIBODY_CONSTRAINT_SPHERICAL_LIMIT),
		  m_desiredVelocity(0, 0, 0),
		  m_desiredPosition(0, 0, 0, 1),
		  m_use_multi_dof_params(false),
		  m_kd(1., 1., 1.),
		  m_kp(0.2, 0.2, 0.2),
		  m_erp(1),
		  m_rhsClamp(SIMD_INFINITY),
		  m_maxAppliedImpulseMultiDof(maxAppliedImpulse, maxAppliedImpulse, maxAppliedImpulse),
		  m_pivotA(m_bodyA->getLink(link).m_eVector),
		  m_pivotB(m_bodyB->getLink(link).m_eVector),
		  m_swingxRange(swingxRange),
		  m_swingyRange(swingyRange),
		  m_twistRange(twistRange)

	{
		m_maxAppliedImpulse = maxAppliedImpulse;
	}
	
	virtual ~btMultiBodySphericalJointLimit() {}
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
		jacobianB(1)[offset] = -1;

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
		const btVector3 zero(0, 0, 0);

		btVector3 axis[3] = {btVector3(1, 0, 0), btVector3(0, 1, 0), btVector3(0, 0, 1)};

		btQuaternion currentQuat(m_bodyA->getJointPosMultiDof(m_linkA)[0],
								 m_bodyA->getJointPosMultiDof(m_linkA)[1],
								 m_bodyA->getJointPosMultiDof(m_linkA)[2],
								 m_bodyA->getJointPosMultiDof(m_linkA)[3]);

		btQuaternion refQuat = m_desiredPosition;
		btVector3 vTwist(0, 0, 1);

		btVector3 vConeNoTwist = quatRotate(currentQuat, vTwist);
		vConeNoTwist.normalize();
		btQuaternion qABCone = shortestArcQuat(vTwist, vConeNoTwist);
		qABCone.normalize();
		btQuaternion qABTwist = qABCone.inverse() * currentQuat;
		qABTwist.normalize();
		btQuaternion desiredQuat = qABTwist;

		btQuaternion relRot = currentQuat.inverse() * desiredQuat;
		btVector3 angleDiff;
		btGeneric6DofSpring2Constraint::matrixToEulerXYZ(btMatrix3x3(relRot), angleDiff);

		btScalar limitRanges[3] = {m_swingxRange, m_swingyRange, m_twistRange};

		/// twist axis/angle
		btQuaternion qMinTwist = qABTwist;
		btScalar twistAngle = qABTwist.getAngle();

		if (twistAngle > SIMD_PI)  // long way around. flip quat and recalculate.
		{
			qMinTwist = -(qABTwist);
			twistAngle = qMinTwist.getAngle();
		}
		btVector3 vTwistAxis = btVector3(qMinTwist.x(), qMinTwist.y(), qMinTwist.z());
		if (twistAngle > SIMD_EPSILON)
			vTwistAxis.normalize();

		if (vTwistAxis.dot(vTwist) < 0)
			twistAngle *= -1.;

		angleDiff[2] = twistAngle;

		for (int row = 0; row < getNumRows(); row++)
		{
			btScalar allowed = limitRanges[row];
			btScalar damp = 1;
			if ((angleDiff[row] > -allowed) && (angleDiff[row] < allowed))
			{
				angleDiff[row] = 0;
				damp = 0;
			}
			else
			{
				if (angleDiff[row] > allowed)
				{
					angleDiff[row] -= allowed;
				}
				if (angleDiff[row] < -allowed)
				{
					angleDiff[row] += allowed;
				}
			}

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
						//should multiply by time step
						//max_applied_impulse *= infoGlobal.m_timeStep

						double min_applied_impulse = -max_applied_impulse;

						if (posError > 0)
							max_applied_impulse = 0;
						else
							min_applied_impulse = 0;

						if (btFabs(posError) > SIMD_EPSILON)
						{
							btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();
							fillMultiBodyConstraint(constraintRow, data, 0, 0, constraintNormalAng,
													zero, zero, zero,  //pure angular, so zero out linear parts
													posError,
													infoGlobal,
													min_applied_impulse, max_applied_impulse, true,
													1.0, false, 0, 0,
													damp);
							constraintRow.m_orgConstraint = this;
							constraintRow.m_orgDofIndex = row;
						}
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


	virtual void debugDraw(class btIDebugDraw* drawer)
	{
		btTransform tr;
		tr.setIdentity();
		if (m_bodyB)
		{
			btVector3 pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotB);
			tr.setOrigin(pivotBworld);
			drawer->drawTransform(tr, 0.1);
		}
	}

};

#endif  //BT_MULTIBODY_SPHERICAL_JOINT_LIMIT_H
