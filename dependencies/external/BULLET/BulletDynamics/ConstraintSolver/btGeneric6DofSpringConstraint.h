/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
*/

#ifndef BT_GENERIC_6DOF_SPRING_CONSTRAINT_H
#define BT_GENERIC_6DOF_SPRING_CONSTRAINT_H

#include "btGeneric6DofConstraint.h"

#define btGeneric6DofSpringConstraintData2 btGeneric6DofSpringConstraintData
#define btGeneric6DofSpringConstraintDataName "btGeneric6DofSpringConstraintData"

/// Generic 6 DOF constraint that allows to set spring motors to any translational and rotational DOF

/// DOF index used in enableSpring() and setStiffness() means:
/// 0 : translation X
/// 1 : translation Y
/// 2 : translation Z
/// 3 : rotation X (3rd Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
/// 4 : rotation Y (2nd Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
/// 5 : rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )

ATTRIBUTE_ALIGNED16(class)
btGeneric6DofSpringConstraint : public btGeneric6DofConstraint
{
protected:
	bool m_springEnabled[6];
	btScalar m_equilibriumPoint[6];
	btScalar m_springStiffness[6];
	btScalar m_springDamping[6];  // between 0 and 1 (1 == no damping)
	void init()
	{
		m_objectType = D6_SPRING_CONSTRAINT_TYPE;

		for (int i = 0; i < 6; i++)
		{
			m_springEnabled[i] = false;
			m_equilibriumPoint[i] = btScalar(0.f);
			m_springStiffness[i] = btScalar(0.f);
			m_springDamping[i] = btScalar(1.f);
		}
	}
	void internalUpdateSprings(btConstraintInfo2 * info)
	{
		// it is assumed that calculateTransforms() have been called before this call
		int i;
		//btVector3 relVel = m_rbB.getLinearVelocity() - m_rbA.getLinearVelocity();
		for (i = 0; i < 3; i++)
		{
			if (m_springEnabled[i])
			{
				// get current position of constraint
				btScalar currPos = m_calculatedLinearDiff[i];
				// calculate difference
				btScalar delta = currPos - m_equilibriumPoint[i];
				// spring force is (delta * m_stiffness) according to Hooke's Law
				btScalar force = delta * m_springStiffness[i];
				btScalar velFactor = info->fps * m_springDamping[i] / btScalar(info->m_numIterations);
				m_linearLimits.m_targetVelocity[i] = velFactor * force;
				m_linearLimits.m_maxMotorForce[i] = btFabs(force);
			}
		}
		for (i = 0; i < 3; i++)
		{
			if (m_springEnabled[i + 3])
			{
				// get current position of constraint
				btScalar currPos = m_calculatedAxisAngleDiff[i];
				// calculate difference
				btScalar delta = currPos - m_equilibriumPoint[i + 3];
				// spring force is (-delta * m_stiffness) according to Hooke's Law
				btScalar force = -delta * m_springStiffness[i + 3];
				btScalar velFactor = info->fps * m_springDamping[i + 3] / btScalar(info->m_numIterations);
				m_angularLimits[i].m_targetVelocity = velFactor * force;
				m_angularLimits[i].m_maxMotorForce = btFabs(force);
			}
		}
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btGeneric6DofSpringConstraint(btRigidBody & rbA, btRigidBody & rbB, const btTransform& frameInA, const btTransform& frameInB, bool useLinearReferenceFrameA)
		: btGeneric6DofConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)
	{
		init();
	}
	btGeneric6DofSpringConstraint(btRigidBody & rbB, const btTransform& frameInB, bool useLinearReferenceFrameB)
		: btGeneric6DofConstraint(rbB, frameInB, useLinearReferenceFrameB)
	{
		init();
	}
	void enableSpring(int index, bool onOff)
	{
		btAssert((index >= 0) && (index < 6));
		m_springEnabled[index] = onOff;
		if (index < 3)
		{
			m_linearLimits.m_enableMotor[index] = onOff;
		}
		else
		{
			m_angularLimits[index - 3].m_enableMotor = onOff;
		}
	}
	void setStiffness(int index, btScalar stiffness)
	{
		btAssert((index >= 0) && (index < 6));
		m_springStiffness[index] = stiffness;
	}
	void setDamping(int index, btScalar damping)
	{
		btAssert((index >= 0) && (index < 6));
		m_springDamping[index] = damping;
	}
	void setEquilibriumPoint()           // set the current constraint position/orientation as an equilibrium point for all DOF
	{
		calculateTransforms();
		int i;

		for (i = 0; i < 3; i++)
		{
			m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
		}
		for (i = 0; i < 3; i++)
		{
			m_equilibriumPoint[i + 3] = m_calculatedAxisAngleDiff[i];
		}
	}
	void setEquilibriumPoint(int index)  // set the current constraint position/orientation as an equilibrium point for given DOF
	{
		btAssert((index >= 0) && (index < 6));
		calculateTransforms();
		if (index < 3)
		{
			m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
		}
		else
		{
			m_equilibriumPoint[index] = m_calculatedAxisAngleDiff[index - 3];
		}
	}
	void setEquilibriumPoint(int index, btScalar val)
	{
		btAssert((index >= 0) && (index < 6));
		m_equilibriumPoint[index] = val;
	}

	bool isSpringEnabled(int index) const
	{
		return m_springEnabled[index];
	}

	btScalar getStiffness(int index) const
	{
		return m_springStiffness[index];
	}

	btScalar getDamping(int index) const
	{
		return m_springDamping[index];
	}

	btScalar getEquilibriumPoint(int index) const
	{
		return m_equilibriumPoint[index];
	}

	virtual void setAxis(const btVector3& axis1, const btVector3& axis2)
	{
		btVector3 zAxis = axis1.normalized();
		btVector3 yAxis = axis2.normalized();
		btVector3 xAxis = yAxis.cross(zAxis);  // we want right coordinate system

		btTransform frameInW;
		frameInW.setIdentity();
		frameInW.getBasis().setValue(xAxis[0], yAxis[0], zAxis[0],
									 xAxis[1], yAxis[1], zAxis[1],
									 xAxis[2], yAxis[2], zAxis[2]);

		// now get constraint frame in local coordinate systems
		m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
		m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;

		calculateTransforms();
	}

	virtual void getInfo2(btConstraintInfo2 * info)
	{
		// this will be called by constraint solver at the constraint setup stage
		// set current motor parameters
		internalUpdateSprings(info);
		// do the rest of job for constraint setup
		btGeneric6DofConstraint::getInfo2(info);
	}

	virtual int calculateSerializeBufferSize() const;
	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

struct btGeneric6DofSpringConstraintData
{
	btGeneric6DofConstraintData m_6dofData;

	int m_springEnabled[6];
	float m_equilibriumPoint[6];
	float m_springStiffness[6];
	float m_springDamping[6];
};

struct btGeneric6DofSpringConstraintDoubleData2
{
	btGeneric6DofConstraintDoubleData2 m_6dofData;

	int m_springEnabled[6];
	double m_equilibriumPoint[6];
	double m_springStiffness[6];
	double m_springDamping[6];
};

SIMD_FORCE_INLINE int btGeneric6DofSpringConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btGeneric6DofSpringConstraintData2);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btGeneric6DofSpringConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btGeneric6DofSpringConstraintData2* dof = (btGeneric6DofSpringConstraintData2*)dataBuffer;
	btGeneric6DofConstraint::serialize(&dof->m_6dofData, serializer);

	int i;
	for (i = 0; i < 6; i++)
	{
		dof->m_equilibriumPoint[i] = m_equilibriumPoint[i];
		dof->m_springDamping[i] = m_springDamping[i];
		dof->m_springEnabled[i] = m_springEnabled[i] ? 1 : 0;
		dof->m_springStiffness[i] = m_springStiffness[i];
	}
	return btGeneric6DofSpringConstraintDataName;
}

#endif  // BT_GENERIC_6DOF_SPRING_CONSTRAINT_H
