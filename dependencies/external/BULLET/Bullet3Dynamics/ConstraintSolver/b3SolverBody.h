#ifndef B3_SOLVER_BODY_H
#define B3_SOLVER_BODY_H

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Matrix3x3.h"

#include "Bullet3Common/b3AlignedAllocator.h"
#include "Bullet3Common/b3TransformUtil.h"

#define b3SimdScalar b3Scalar

///The b3SolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase cache coherence/performance.
B3_ATTRIBUTE_ALIGNED16(struct)
b3SolverBody
{
	B3_DECLARE_ALIGNED_ALLOCATOR();
	b3Transform m_worldTransform;
	b3Vector3 m_deltaLinearVelocity;
	b3Vector3 m_deltaAngularVelocity;
	b3Vector3 m_angularFactor;
	b3Vector3 m_linearFactor;
	b3Vector3 m_invMass;
	b3Vector3 m_pushVelocity;
	b3Vector3 m_turnVelocity;
	b3Vector3 m_linearVelocity;
	b3Vector3 m_angularVelocity;

	union {
		void* m_originalBody;
		int m_originalBodyIndex;
	};

	int padding[3];

	void setWorldTransform(const b3Transform& worldTransform)
	{
		m_worldTransform = worldTransform;
	}

	const b3Transform& getWorldTransform() const
	{
		return m_worldTransform;
	}

	B3_FORCE_INLINE void getVelocityInLocalPointObsolete(const b3Vector3& rel_pos, b3Vector3& velocity) const
	{
		if (m_originalBody)
			velocity = m_linearVelocity + m_deltaLinearVelocity + (m_angularVelocity + m_deltaAngularVelocity).cross(rel_pos);
		else
			velocity.setValue(0, 0, 0);
	}

	B3_FORCE_INLINE void getAngularVelocity(b3Vector3 & angVel) const
	{
		if (m_originalBody)
			angVel = m_angularVelocity + m_deltaAngularVelocity;
		else
			angVel.setValue(0, 0, 0);
	}

	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	B3_FORCE_INLINE void applyImpulse(const b3Vector3& linearComponent, const b3Vector3& angularComponent, const b3Scalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_deltaLinearVelocity += linearComponent * impulseMagnitude * m_linearFactor;
			m_deltaAngularVelocity += angularComponent * (impulseMagnitude * m_angularFactor);
		}
	}

	B3_FORCE_INLINE void internalApplyPushImpulse(const b3Vector3& linearComponent, const b3Vector3& angularComponent, b3Scalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_pushVelocity += linearComponent * impulseMagnitude * m_linearFactor;
			m_turnVelocity += angularComponent * (impulseMagnitude * m_angularFactor);
		}
	}

	const b3Vector3& getDeltaLinearVelocity() const
	{
		return m_deltaLinearVelocity;
	}

	const b3Vector3& getDeltaAngularVelocity() const
	{
		return m_deltaAngularVelocity;
	}

	const b3Vector3& getPushVelocity() const
	{
		return m_pushVelocity;
	}

	const b3Vector3& getTurnVelocity() const
	{
		return m_turnVelocity;
	}

	////////////////////////////////////////////////
	///some internal methods, don't use them

	b3Vector3& internalGetDeltaLinearVelocity()
	{
		return m_deltaLinearVelocity;
	}

	b3Vector3& internalGetDeltaAngularVelocity()
	{
		return m_deltaAngularVelocity;
	}

	const b3Vector3& internalGetAngularFactor() const
	{
		return m_angularFactor;
	}

	const b3Vector3& internalGetInvMass() const
	{
		return m_invMass;
	}

	void internalSetInvMass(const b3Vector3& invMass)
	{
		m_invMass = invMass;
	}

	b3Vector3& internalGetPushVelocity()
	{
		return m_pushVelocity;
	}

	b3Vector3& internalGetTurnVelocity()
	{
		return m_turnVelocity;
	}

	B3_FORCE_INLINE void internalGetVelocityInLocalPointObsolete(const b3Vector3& rel_pos, b3Vector3& velocity) const
	{
		velocity = m_linearVelocity + m_deltaLinearVelocity + (m_angularVelocity + m_deltaAngularVelocity).cross(rel_pos);
	}

	B3_FORCE_INLINE void internalGetAngularVelocity(b3Vector3 & angVel) const
	{
		angVel = m_angularVelocity + m_deltaAngularVelocity;
	}

	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	B3_FORCE_INLINE void internalApplyImpulse(const b3Vector3& linearComponent, const b3Vector3& angularComponent, const b3Scalar impulseMagnitude)
	{
		m_deltaLinearVelocity += linearComponent * impulseMagnitude * m_linearFactor;
		m_deltaAngularVelocity += angularComponent * (impulseMagnitude * m_angularFactor);
	}

	void writebackVelocity()
	{
		m_linearVelocity += m_deltaLinearVelocity;
		m_angularVelocity += m_deltaAngularVelocity;
	}

	void writebackVelocityAndTransform(b3Scalar timeStep, b3Scalar splitImpulseTurnErp)
	{
		(void)timeStep;
		if (m_originalBody)
		{
			m_linearVelocity += m_deltaLinearVelocity;
			m_angularVelocity += m_deltaAngularVelocity;

			//correct the position/orientation based on push/turn recovery
			b3Transform newTransform;
			if (m_pushVelocity[0] != 0.f || m_pushVelocity[1] != 0 || m_pushVelocity[2] != 0 || m_turnVelocity[0] != 0.f || m_turnVelocity[1] != 0 || m_turnVelocity[2] != 0)
			{
				b3TransformUtil::integrateTransform(m_worldTransform, m_pushVelocity, m_turnVelocity * splitImpulseTurnErp, timeStep, newTransform);
				m_worldTransform = newTransform;
			}
		}
	}
};

#endif  //B3_SOLVER_BODY_H
