#ifndef B3_POINT2POINTCONSTRAINT_H
#define B3_POINT2POINTCONSTRAINT_H

#include "Bullet3Common/b3Vector3.h"
#include "b3TypedConstraint.h"

#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"

class b3RigidBody;

#define b3Point2PointConstraintData b3Point2PointConstraintFloatData
#define b3Point2PointConstraintDataName "b3Point2PointConstraintFloatData"

struct b3ConstraintSetting
{
	b3ConstraintSetting() : m_tau(b3Scalar(0.3)), m_damping(b3Scalar(1.)), m_impulseClamp(b3Scalar(0.)) {}
	b3Scalar m_tau;
	b3Scalar m_damping;
	b3Scalar m_impulseClamp;
};

enum b3Point2PointFlags
{
	B3_P2P_FLAGS_ERP = 1,
	B3_P2P_FLAGS_CFM = 2
};

/// point to point constraint between two rigidbodies each with a pivotpoint that descibes the 'ballsocket' location in local space
B3_ATTRIBUTE_ALIGNED16(class)
b3Point2PointConstraint : public b3TypedConstraint
{
	b3Vector3 m_pivotInA;
	b3Vector3 m_pivotInB;

	int m_flags;
	b3Scalar m_erp;
	b3Scalar m_cfm;

public:
	B3_DECLARE_ALIGNED_ALLOCATOR();

	b3ConstraintSetting m_setting;

	b3Point2PointConstraint(int rbA, int rbB, const b3Vector3& pivotInA, const b3Vector3& pivotInB)
		: b3TypedConstraint(B3_POINT2POINT_CONSTRAINT_TYPE, rbA, rbB), m_pivotInA(pivotInA), m_pivotInB(pivotInB), m_flags(0)
	{
	}

	virtual void getInfo1(b3ConstraintInfo1 * info, const b3RigidBodyData* bodies)
	{
		getInfo1NonVirtual(info, bodies);
	}

	void getInfo1NonVirtual(b3ConstraintInfo1 * info, const b3RigidBodyData* bodies)
	{
		info->m_numConstraintRows = 3;
		info->nub = 3;
	}

	virtual void getInfo2(b3ConstraintInfo2 * info, const b3RigidBodyData* bodies)
	{
		b3Transform trA;
		trA.setIdentity();
		trA.setOrigin(bodies[m_rbA].m_pos);
		trA.setRotation(bodies[m_rbA].m_quat);

		b3Transform trB;
		trB.setIdentity();
		trB.setOrigin(bodies[m_rbB].m_pos);
		trB.setRotation(bodies[m_rbB].m_quat);

		getInfo2NonVirtual(info, trA, trB);
	}

	void getInfo2NonVirtual(b3ConstraintInfo2 * info, const b3Transform& body0_trans, const b3Transform& body1_trans)
	{
		//retrieve matrices

		// anchor points in global coordinates with respect to body PORs.

		// set jacobian
		info->m_J1linearAxis[0] = 1;
		info->m_J1linearAxis[info->rowskip + 1] = 1;
		info->m_J1linearAxis[2 * info->rowskip + 2] = 1;

		b3Vector3 a1 = body0_trans.getBasis() * getPivotInA();
		//b3Vector3 a1a = b3QuatRotate(body0_trans.getRotation(),getPivotInA());

		{
			b3Vector3* angular0 = (b3Vector3*)(info->m_J1angularAxis);
			b3Vector3* angular1 = (b3Vector3*)(info->m_J1angularAxis + info->rowskip);
			b3Vector3* angular2 = (b3Vector3*)(info->m_J1angularAxis + 2 * info->rowskip);
			b3Vector3 a1neg = -a1;
			a1neg.getSkewSymmetricMatrix(angular0, angular1, angular2);
		}

		if (info->m_J2linearAxis)
		{
			info->m_J2linearAxis[0] = -1;
			info->m_J2linearAxis[info->rowskip + 1] = -1;
			info->m_J2linearAxis[2 * info->rowskip + 2] = -1;
		}

		b3Vector3 a2 = body1_trans.getBasis() * getPivotInB();

		{
			//	b3Vector3 a2n = -a2;
			b3Vector3* angular0 = (b3Vector3*)(info->m_J2angularAxis);
			b3Vector3* angular1 = (b3Vector3*)(info->m_J2angularAxis + info->rowskip);
			b3Vector3* angular2 = (b3Vector3*)(info->m_J2angularAxis + 2 * info->rowskip);
			a2.getSkewSymmetricMatrix(angular0, angular1, angular2);
		}

		// set right hand side
		b3Scalar currERP = (m_flags & B3_P2P_FLAGS_ERP) ? m_erp : info->erp;
		b3Scalar k = info->fps * currERP;
		int j;
		for (j = 0; j < 3; j++)
		{
			info->m_constraintError[j * info->rowskip] = k * (a2[j] + body1_trans.getOrigin()[j] - a1[j] - body0_trans.getOrigin()[j]);
			//printf("info->m_constraintError[%d]=%f\n",j,info->m_constraintError[j]);
		}
		if (m_flags & B3_P2P_FLAGS_CFM)
		{
			for (j = 0; j < 3; j++)
			{
				info->cfm[j * info->rowskip] = m_cfm;
			}
		}

		b3Scalar impulseClamp = m_setting.m_impulseClamp;  //
		for (j = 0; j < 3; j++)
		{
			if (m_setting.m_impulseClamp > 0)
			{
				info->m_lowerLimit[j * info->rowskip] = -impulseClamp;
				info->m_upperLimit[j * info->rowskip] = impulseClamp;
			}
		}
		info->m_damping = m_setting.m_damping;
	}

	void updateRHS(b3Scalar timeStep)
	{
		(void)timeStep;
	}

	void setPivotA(const b3Vector3& pivotA)
	{
		m_pivotInA = pivotA;
	}

	void setPivotB(const b3Vector3& pivotB)
	{
		m_pivotInB = pivotB;
	}

	const b3Vector3& getPivotInA() const
	{
		return m_pivotInA;
	}

	const b3Vector3& getPivotInB() const
	{
		return m_pivotInB;
	}

	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
	///If no axis is provided, it uses the default axis for this constraint.
	virtual void setParam(int num, b3Scalar value, int axis = -1)
	{
		if (axis != -1)
		{
			b3AssertConstrParams(0);
		}
		else
		{
			switch (num)
			{
				case B3_CONSTRAINT_ERP:
				case B3_CONSTRAINT_STOP_ERP:
					m_erp = value;
					m_flags |= B3_P2P_FLAGS_ERP;
					break;
				case B3_CONSTRAINT_CFM:
				case B3_CONSTRAINT_STOP_CFM:
					m_cfm = value;
					m_flags |= B3_P2P_FLAGS_CFM;
					break;
				default:
					b3AssertConstrParams(0);
			}
		}
	}
	///return the local value of parameter
	virtual b3Scalar getParam(int num, int axis = -1) const
	{
		b3Scalar retVal(B3_INFINITY);
		if (axis != -1)
		{
			b3AssertConstrParams(0);
		}
		else
		{
			switch (num)
			{
				case B3_CONSTRAINT_ERP:
				case B3_CONSTRAINT_STOP_ERP:
					b3AssertConstrParams(m_flags & B3_P2P_FLAGS_ERP);
					retVal = m_erp;
					break;
				case B3_CONSTRAINT_CFM:
				case B3_CONSTRAINT_STOP_CFM:
					b3AssertConstrParams(m_flags & B3_P2P_FLAGS_CFM);
					retVal = m_cfm;
					break;
				default:
					b3AssertConstrParams(0);
			}
		}
		return retVal;
	}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	//	virtual	const char*	serialize(void* dataBuffer, b3Serializer* serializer) const;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct b3Point2PointConstraintFloatData
{
	b3TypedConstraintData m_typeConstraintData;
	b3Vector3FloatData m_pivotInA;
	b3Vector3FloatData m_pivotInB;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct b3Point2PointConstraintDoubleData
{
	b3TypedConstraintData m_typeConstraintData;
	b3Vector3DoubleData m_pivotInA;
	b3Vector3DoubleData m_pivotInB;
};

#endif  //B3_POINT2POINTCONSTRAINT_H
