/*
2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
Pros:
- Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
- Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
- Servo motor functionality
- Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
- Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)

Cons:
- It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation.
- At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
*/

// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
	2007-09-09
	btGeneric6DofConstraint Refactored by Francisco Le?n
	email: projectileman@yahoo.com
	http://gimpact.sf.net
*/

#ifndef BT_GENERIC_6DOF_CONSTRAINT2_H
#define BT_GENERIC_6DOF_CONSTRAINT2_H

#include "btTypedConstraint.h"
#include <cmath>

#define btGeneric6DofSpring2ConstraintData2 btGeneric6DofSpring2ConstraintData
#define btGeneric6DofSpring2ConstraintDataName "btGeneric6DofSpring2ConstraintData"

enum RotateOrder
{
	RO_XYZ = 0,
	RO_XZY,
	RO_YXZ,
	RO_YZX,
	RO_ZXY,
	RO_ZYX
};

class btRotationalLimitMotor2
{
public:
	// upper < lower means free
	// upper == lower means locked
	// upper > lower means limited
	btScalar m_loLimit;
	btScalar m_hiLimit;
	btScalar m_bounce;
	btScalar m_stopERP;
	btScalar m_stopCFM;
	btScalar m_motorERP;
	btScalar m_motorCFM;
	bool m_enableMotor;
	btScalar m_targetVelocity;
	btScalar m_maxMotorForce;
	bool m_servoMotor;
	btScalar m_servoTarget;
	bool m_enableSpring;
	btScalar m_springStiffness;
	bool m_springStiffnessLimited;
	btScalar m_springDamping;
	bool m_springDampingLimited;
	btScalar m_equilibriumPoint;

	btScalar m_currentLimitError;
	btScalar m_currentLimitErrorHi;
	btScalar m_currentPosition;
	int m_currentLimit;

	btRotationalLimitMotor2()
	{
		m_loLimit = 1.0f;
		m_hiLimit = -1.0f;
		m_bounce = 0.0f;
		m_stopERP = 0.2f;
		m_stopCFM = 0.f;
		m_motorERP = 0.9f;
		m_motorCFM = 0.f;
		m_enableMotor = false;
		m_targetVelocity = 0;
		m_maxMotorForce = 6.0f;
		m_servoMotor = false;
		m_servoTarget = 0;
		m_enableSpring = false;
		m_springStiffness = 0;
		m_springStiffnessLimited = false;
		m_springDamping = 0;
		m_springDampingLimited = false;
		m_equilibriumPoint = 0;

		m_currentLimitError = 0;
		m_currentLimitErrorHi = 0;
		m_currentPosition = 0;
		m_currentLimit = 0;
	}

	btRotationalLimitMotor2(const btRotationalLimitMotor2& limot)
	{
		m_loLimit = limot.m_loLimit;
		m_hiLimit = limot.m_hiLimit;
		m_bounce = limot.m_bounce;
		m_stopERP = limot.m_stopERP;
		m_stopCFM = limot.m_stopCFM;
		m_motorERP = limot.m_motorERP;
		m_motorCFM = limot.m_motorCFM;
		m_enableMotor = limot.m_enableMotor;
		m_targetVelocity = limot.m_targetVelocity;
		m_maxMotorForce = limot.m_maxMotorForce;
		m_servoMotor = limot.m_servoMotor;
		m_servoTarget = limot.m_servoTarget;
		m_enableSpring = limot.m_enableSpring;
		m_springStiffness = limot.m_springStiffness;
		m_springStiffnessLimited = limot.m_springStiffnessLimited;
		m_springDamping = limot.m_springDamping;
		m_springDampingLimited = limot.m_springDampingLimited;
		m_equilibriumPoint = limot.m_equilibriumPoint;

		m_currentLimitError = limot.m_currentLimitError;
		m_currentLimitErrorHi = limot.m_currentLimitErrorHi;
		m_currentPosition = limot.m_currentPosition;
		m_currentLimit = limot.m_currentLimit;
	}

	bool isLimited()
	{
		if (m_loLimit > m_hiLimit) return false;
		return true;
	}

	void testLimitValue(btScalar test_value)
	{
		//we can't normalize the angles here because we would lost the sign that we use later, but it doesn't seem to be a problem
		if (m_loLimit > m_hiLimit)
		{
			m_currentLimit = 0;
			m_currentLimitError = btScalar(0.f);
		}
		else if (m_loLimit == m_hiLimit)
		{
			m_currentLimitError = test_value - m_loLimit;
			m_currentLimit = 3;
		}
		else
		{
			m_currentLimitError = test_value - m_loLimit;
			m_currentLimitErrorHi = test_value - m_hiLimit;
			m_currentLimit = 4;
		}
	}
};

class btTranslationalLimitMotor2
{
public:
	// upper < lower means free
	// upper == lower means locked
	// upper > lower means limited
	btVector3 m_lowerLimit;
	btVector3 m_upperLimit;
	btVector3 m_bounce;
	btVector3 m_stopERP;
	btVector3 m_stopCFM;
	btVector3 m_motorERP;
	btVector3 m_motorCFM;
	bool m_enableMotor[3];
	bool m_servoMotor[3];
	bool m_enableSpring[3];
	btVector3 m_servoTarget;
	btVector3 m_springStiffness;
	bool m_springStiffnessLimited[3];
	btVector3 m_springDamping;
	bool m_springDampingLimited[3];
	btVector3 m_equilibriumPoint;
	btVector3 m_targetVelocity;
	btVector3 m_maxMotorForce;

	btVector3 m_currentLimitError;
	btVector3 m_currentLimitErrorHi;
	btVector3 m_currentLinearDiff;
	int m_currentLimit[3];

	btTranslationalLimitMotor2()
	{
		m_lowerLimit.setValue(0.f, 0.f, 0.f);
		m_upperLimit.setValue(0.f, 0.f, 0.f);
		m_bounce.setValue(0.f, 0.f, 0.f);
		m_stopERP.setValue(0.2f, 0.2f, 0.2f);
		m_stopCFM.setValue(0.f, 0.f, 0.f);
		m_motorERP.setValue(0.9f, 0.9f, 0.9f);
		m_motorCFM.setValue(0.f, 0.f, 0.f);

		m_currentLimitError.setValue(0.f, 0.f, 0.f);
		m_currentLimitErrorHi.setValue(0.f, 0.f, 0.f);
		m_currentLinearDiff.setValue(0.f, 0.f, 0.f);

		for (int i = 0; i < 3; i++)
		{
			m_enableMotor[i] = false;
			m_servoMotor[i] = false;
			m_enableSpring[i] = false;
			m_servoTarget[i] = btScalar(0.f);
			m_springStiffness[i] = btScalar(0.f);
			m_springStiffnessLimited[i] = false;
			m_springDamping[i] = btScalar(0.f);
			m_springDampingLimited[i] = false;
			m_equilibriumPoint[i] = btScalar(0.f);
			m_targetVelocity[i] = btScalar(0.f);
			m_maxMotorForce[i] = btScalar(0.f);

			m_currentLimit[i] = 0;
		}
	}

	btTranslationalLimitMotor2(const btTranslationalLimitMotor2& other)
	{
		m_lowerLimit = other.m_lowerLimit;
		m_upperLimit = other.m_upperLimit;
		m_bounce = other.m_bounce;
		m_stopERP = other.m_stopERP;
		m_stopCFM = other.m_stopCFM;
		m_motorERP = other.m_motorERP;
		m_motorCFM = other.m_motorCFM;

		m_currentLimitError = other.m_currentLimitError;
		m_currentLimitErrorHi = other.m_currentLimitErrorHi;
		m_currentLinearDiff = other.m_currentLinearDiff;

		for (int i = 0; i < 3; i++)
		{
			m_enableMotor[i] = other.m_enableMotor[i];
			m_servoMotor[i] = other.m_servoMotor[i];
			m_enableSpring[i] = other.m_enableSpring[i];
			m_servoTarget[i] = other.m_servoTarget[i];
			m_springStiffness[i] = other.m_springStiffness[i];
			m_springStiffnessLimited[i] = other.m_springStiffnessLimited[i];
			m_springDamping[i] = other.m_springDamping[i];
			m_springDampingLimited[i] = other.m_springDampingLimited[i];
			m_equilibriumPoint[i] = other.m_equilibriumPoint[i];
			m_targetVelocity[i] = other.m_targetVelocity[i];
			m_maxMotorForce[i] = other.m_maxMotorForce[i];

			m_currentLimit[i] = other.m_currentLimit[i];
		}
	}

	inline bool isLimited(int limitIndex)
	{
		return (m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex]);
	}

	void testLimitValue(int limitIndex, btScalar test_value)
	{
		btScalar loLimit = m_lowerLimit[limitIndex];
		btScalar hiLimit = m_upperLimit[limitIndex];
		if (loLimit > hiLimit)
		{
			m_currentLimitError[limitIndex] = 0;
			m_currentLimit[limitIndex] = 0;
		}
		else if (loLimit == hiLimit)
		{
			m_currentLimitError[limitIndex] = test_value - loLimit;
			m_currentLimit[limitIndex] = 3;
		}
		else
		{
			m_currentLimitError[limitIndex] = test_value - loLimit;
			m_currentLimitErrorHi[limitIndex] = test_value - hiLimit;
			m_currentLimit[limitIndex] = 4;
		}
	}
};

enum bt6DofFlags2
{
	BT_6DOF_FLAGS_CFM_STOP2 = 1,
	BT_6DOF_FLAGS_ERP_STOP2 = 2,
	BT_6DOF_FLAGS_CFM_MOTO2 = 4,
	BT_6DOF_FLAGS_ERP_MOTO2 = 8,
	BT_6DOF_FLAGS_USE_INFINITE_ERROR = (1<<16)
};
#define BT_6DOF_FLAGS_AXIS_SHIFT2 4  // bits per axis

ATTRIBUTE_ALIGNED16(class)
btGeneric6DofSpring2Constraint : public btTypedConstraint
{
protected:
	btTransform m_frameInA;
	btTransform m_frameInB;

	btJacobianEntry m_jacLinear[3];
	btJacobianEntry m_jacAng[3];

	btTranslationalLimitMotor2 m_linearLimits;
	btRotationalLimitMotor2 m_angularLimits[3];

	RotateOrder m_rotateOrder;

protected:
	btTransform m_calculatedTransformA;
	btTransform m_calculatedTransformB;
	btVector3 m_calculatedAxisAngleDiff;
	btVector3 m_calculatedAxis[3];
	btVector3 m_calculatedLinearDiff;
	btScalar m_factA;
	btScalar m_factB;
	bool m_hasStaticBody;
	int m_flags;

	btGeneric6DofSpring2Constraint& operator=(const btGeneric6DofSpring2Constraint&)
	{
		btAssert(0);
		return *this;
	}

	int setAngularLimits(btConstraintInfo2 * info, int row_offset, const btTransform& transA, const btTransform& transB, const btVector3& linVelA, const btVector3& linVelB, const btVector3& angVelA, const btVector3& angVelB)
	{
		int row = row_offset;

		//order of rotational constraint rows
		int cIdx[] = {0, 1, 2};
		switch (m_rotateOrder)
		{
			case RO_XYZ:
				cIdx[0] = 0;
				cIdx[1] = 1;
				cIdx[2] = 2;
				break;
			case RO_XZY:
				cIdx[0] = 0;
				cIdx[1] = 2;
				cIdx[2] = 1;
				break;
			case RO_YXZ:
				cIdx[0] = 1;
				cIdx[1] = 0;
				cIdx[2] = 2;
				break;
			case RO_YZX:
				cIdx[0] = 1;
				cIdx[1] = 2;
				cIdx[2] = 0;
				break;
			case RO_ZXY:
				cIdx[0] = 2;
				cIdx[1] = 0;
				cIdx[2] = 1;
				break;
			case RO_ZYX:
				cIdx[0] = 2;
				cIdx[1] = 1;
				cIdx[2] = 0;
				break;
			default:
				btAssert(false);
		}

		for (int ii = 0; ii < 3; ii++)
		{
			int i = cIdx[ii];
			if (m_angularLimits[i].m_currentLimit || m_angularLimits[i].m_enableMotor || m_angularLimits[i].m_enableSpring)
			{
				btVector3 axis = getAxis(i);
				int flags = m_flags >> ((i + 3) * BT_6DOF_FLAGS_AXIS_SHIFT2);
				if (!(flags & BT_6DOF_FLAGS_CFM_STOP2))
				{
					m_angularLimits[i].m_stopCFM = info->cfm[0];
				}
				if (!(flags & BT_6DOF_FLAGS_ERP_STOP2))
				{
					m_angularLimits[i].m_stopERP = info->erp;
				}
				if (!(flags & BT_6DOF_FLAGS_CFM_MOTO2))
				{
					m_angularLimits[i].m_motorCFM = info->cfm[0];
				}
				if (!(flags & BT_6DOF_FLAGS_ERP_MOTO2))
				{
					m_angularLimits[i].m_motorERP = info->erp;
				}
				row += get_limit_motor_info2(&m_angularLimits[i], transA, transB, linVelA, linVelB, angVelA, angVelB, info, row, axis, 1);
			}
		}

		return row;
	}
	int setLinearLimits(btConstraintInfo2 * info, int row, const btTransform& transA, const btTransform& transB, const btVector3& linVelA, const btVector3& linVelB, const btVector3& angVelA, const btVector3& angVelB)
	{
		//solve linear limits
		btRotationalLimitMotor2 limot;
		for (int i = 0; i < 3; i++)
		{
			if (m_linearLimits.m_currentLimit[i] || m_linearLimits.m_enableMotor[i] || m_linearLimits.m_enableSpring[i])
			{  // re-use rotational motor code
				limot.m_bounce = m_linearLimits.m_bounce[i];
				limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
				limot.m_currentPosition = m_linearLimits.m_currentLinearDiff[i];
				limot.m_currentLimitError = m_linearLimits.m_currentLimitError[i];
				limot.m_currentLimitErrorHi = m_linearLimits.m_currentLimitErrorHi[i];
				limot.m_enableMotor = m_linearLimits.m_enableMotor[i];
				limot.m_servoMotor = m_linearLimits.m_servoMotor[i];
				limot.m_servoTarget = m_linearLimits.m_servoTarget[i];
				limot.m_enableSpring = m_linearLimits.m_enableSpring[i];
				limot.m_springStiffness = m_linearLimits.m_springStiffness[i];
				limot.m_springStiffnessLimited = m_linearLimits.m_springStiffnessLimited[i];
				limot.m_springDamping = m_linearLimits.m_springDamping[i];
				limot.m_springDampingLimited = m_linearLimits.m_springDampingLimited[i];
				limot.m_equilibriumPoint = m_linearLimits.m_equilibriumPoint[i];
				limot.m_hiLimit = m_linearLimits.m_upperLimit[i];
				limot.m_loLimit = m_linearLimits.m_lowerLimit[i];
				limot.m_maxMotorForce = m_linearLimits.m_maxMotorForce[i];
				limot.m_targetVelocity = m_linearLimits.m_targetVelocity[i];
				btVector3 axis = m_calculatedTransformA.getBasis().getColumn(i);
				int flags = m_flags >> (i * BT_6DOF_FLAGS_AXIS_SHIFT2);
				limot.m_stopCFM = (flags & BT_6DOF_FLAGS_CFM_STOP2) ? m_linearLimits.m_stopCFM[i] : info->cfm[0];
				limot.m_stopERP = (flags & BT_6DOF_FLAGS_ERP_STOP2) ? m_linearLimits.m_stopERP[i] : info->erp;
				limot.m_motorCFM = (flags & BT_6DOF_FLAGS_CFM_MOTO2) ? m_linearLimits.m_motorCFM[i] : info->cfm[0];
				limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) ? m_linearLimits.m_motorERP[i] : info->erp;

				//rotAllowed is a bit of a magic from the original 6dof. The calculation of it here is something that imitates the original behavior as much as possible.
				int indx1 = (i + 1) % 3;
				int indx2 = (i + 2) % 3;
				int rotAllowed = 1;  // rotations around orthos to current axis (it is used only when one of the body is static)
#define D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION 1.0e-3
				bool indx1Violated = m_angularLimits[indx1].m_currentLimit == 1 ||
									 m_angularLimits[indx1].m_currentLimit == 2 ||
									 (m_angularLimits[indx1].m_currentLimit == 3 && (m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION)) ||
									 (m_angularLimits[indx1].m_currentLimit == 4 && (m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION));
				bool indx2Violated = m_angularLimits[indx2].m_currentLimit == 1 ||
									 m_angularLimits[indx2].m_currentLimit == 2 ||
									 (m_angularLimits[indx2].m_currentLimit == 3 && (m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION)) ||
									 (m_angularLimits[indx2].m_currentLimit == 4 && (m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION));
				if (indx1Violated && indx2Violated)
				{
					rotAllowed = 0;
				}
				row += get_limit_motor_info2(&limot, transA, transB, linVelA, linVelB, angVelA, angVelB, info, row, axis, 0, rotAllowed);
			}
		}
		return row;
	}

	void calculateLinearInfo()
	{
		m_calculatedLinearDiff = m_calculatedTransformB.getOrigin() - m_calculatedTransformA.getOrigin();
		m_calculatedLinearDiff = m_calculatedTransformA.getBasis().inverse() * m_calculatedLinearDiff;
		for (int i = 0; i < 3; i++)
		{
			m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
			m_linearLimits.testLimitValue(i, m_calculatedLinearDiff[i]);
		}
	}
	void calculateAngleInfo()
	{
		btMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().inverse() * m_calculatedTransformB.getBasis();
		switch (m_rotateOrder)
		{
			case RO_XYZ:
				matrixToEulerXYZ(relative_frame, m_calculatedAxisAngleDiff);
				break;
			case RO_XZY:
				matrixToEulerXZY(relative_frame, m_calculatedAxisAngleDiff);
				break;
			case RO_YXZ:
				matrixToEulerYXZ(relative_frame, m_calculatedAxisAngleDiff);
				break;
			case RO_YZX:
				matrixToEulerYZX(relative_frame, m_calculatedAxisAngleDiff);
				break;
			case RO_ZXY:
				matrixToEulerZXY(relative_frame, m_calculatedAxisAngleDiff);
				break;
			case RO_ZYX:
				matrixToEulerZYX(relative_frame, m_calculatedAxisAngleDiff);
				break;
			default:
				btAssert(false);
		}
		// in euler angle mode we do not actually constrain the angular velocity
		// along the axes axis[0] and axis[2] (although we do use axis[1]) :
		//
		//    to get			constrain w2-w1 along		...not
		//    ------			---------------------		------
		//    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
		//    d(angle[1])/dt = 0	ax[1]
		//    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
		//
		// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
		// to prove the result for angle[0], write the expression for angle[0] from
		// GetInfo1 then take the derivative. to prove this for angle[2] it is
		// easier to take the euler rate expression for d(angle[2])/dt with respect
		// to the components of w and set that to 0.
		switch (m_rotateOrder)
		{
			case RO_XYZ:
			{
				//Is this the "line of nodes" calculation choosing planes YZ (B coordinate system) and xy (A coordinate system)? (http://en.wikipedia.org/wiki/Euler_angles)
				//The two planes are non-homologous, so this is a Tait Bryan angle formalism and not a proper Euler
				//Extrinsic rotations are equal to the reversed order intrinsic rotations so the above xyz extrinsic rotations (axes are fixed) are the same as the zy'x" intrinsic rotations (axes are refreshed after each rotation)
				//that is why xy and YZ planes are chosen (this will describe a zy'x" intrinsic rotation) (see the figure on the left at http://en.wikipedia.org/wiki/Euler_angles under Tait Bryan angles)
				// x' = Nperp = N.cross(axis2)
				// y' = N = axis2.cross(axis0)
				// z' = z
				//
				// x" = X
				// y" = y'
				// z" = ??
				//in other words:
				//first rotate around z
				//second rotate around y'= z.cross(X)
				//third rotate around x" = X
				//Original XYZ extrinsic rotation order.
				//Planes: xy and YZ normals: z, X.  Plane intersection (N) is z.cross(X)
				btVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
				btVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);
				m_calculatedAxis[1] = axis2.cross(axis0);
				m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
				m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
				break;
			}
			case RO_XZY:
			{
				//planes: xz,ZY normals: y, X
				//first rotate around y
				//second rotate around z'= y.cross(X)
				//third rotate around x" = X
				btVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
				btVector3 axis1 = m_calculatedTransformA.getBasis().getColumn(1);
				m_calculatedAxis[2] = axis0.cross(axis1);
				m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
				m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
				break;
			}
			case RO_YXZ:
			{
				//planes: yx,XZ normals: z, Y
				//first rotate around z
				//second rotate around x'= z.cross(Y)
				//third rotate around y" = Y
				btVector3 axis1 = m_calculatedTransformB.getBasis().getColumn(1);
				btVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);
				m_calculatedAxis[0] = axis1.cross(axis2);
				m_calculatedAxis[1] = axis2.cross(m_calculatedAxis[0]);
				m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
				break;
			}
			case RO_YZX:
			{
				//planes: yz,ZX normals: x, Y
				//first rotate around x
				//second rotate around z'= x.cross(Y)
				//third rotate around y" = Y
				btVector3 axis0 = m_calculatedTransformA.getBasis().getColumn(0);
				btVector3 axis1 = m_calculatedTransformB.getBasis().getColumn(1);
				m_calculatedAxis[2] = axis0.cross(axis1);
				m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
				m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
				break;
			}
			case RO_ZXY:
			{
				//planes: zx,XY normals: y, Z
				//first rotate around y
				//second rotate around x'= y.cross(Z)
				//third rotate around z" = Z
				btVector3 axis1 = m_calculatedTransformA.getBasis().getColumn(1);
				btVector3 axis2 = m_calculatedTransformB.getBasis().getColumn(2);
				m_calculatedAxis[0] = axis1.cross(axis2);
				m_calculatedAxis[1] = axis2.cross(m_calculatedAxis[0]);
				m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
				break;
			}
			case RO_ZYX:
			{
				//planes: zy,YX normals: x, Z
				//first rotate around x
				//second rotate around y' = x.cross(Z)
				//third rotate around z" = Z
				btVector3 axis0 = m_calculatedTransformA.getBasis().getColumn(0);
				btVector3 axis2 = m_calculatedTransformB.getBasis().getColumn(2);
				m_calculatedAxis[1] = axis2.cross(axis0);
				m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
				m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
				break;
			}
			default:
				btAssert(false);
		}

		m_calculatedAxis[0].normalize();
		m_calculatedAxis[1].normalize();
		m_calculatedAxis[2].normalize();
	}
	void testAngularLimitMotor(int axis_index)
	{
		btScalar angle = m_calculatedAxisAngleDiff[axis_index];
		angle = btAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit);
		m_angularLimits[axis_index].m_currentPosition = angle;
		m_angularLimits[axis_index].testLimitValue(angle);
	}

	void calculateJacobi(btRotationalLimitMotor2 * limot, const btTransform& transA, const btTransform& transB, btConstraintInfo2* info, int srow, btVector3& ax1, int rotational, int rotAllowed)
	{
		btScalar* J1 = rotational ? info->m_J1angularAxis : info->m_J1linearAxis;
		btScalar* J2 = rotational ? info->m_J2angularAxis : info->m_J2linearAxis;

		J1[srow + 0] = ax1[0];
		J1[srow + 1] = ax1[1];
		J1[srow + 2] = ax1[2];

		J2[srow + 0] = -ax1[0];
		J2[srow + 1] = -ax1[1];
		J2[srow + 2] = -ax1[2];

		if (!rotational)
		{
			btVector3 tmpA, tmpB, relA, relB;
			// get vector from bodyB to frameB in WCS
			relB = m_calculatedTransformB.getOrigin() - transB.getOrigin();
			// same for bodyA
			relA = m_calculatedTransformA.getOrigin() - transA.getOrigin();
			tmpA = relA.cross(ax1);
			tmpB = relB.cross(ax1);
			if (m_hasStaticBody && (!rotAllowed))
			{
				tmpA *= m_factA;
				tmpB *= m_factB;
			}
			int i;
			for (i = 0; i < 3; i++) info->m_J1angularAxis[srow + i] = tmpA[i];
			for (i = 0; i < 3; i++) info->m_J2angularAxis[srow + i] = -tmpB[i];
		}
	}
	int get_limit_motor_info2(btRotationalLimitMotor2 * limot, const btTransform& transA, const btTransform& transB, const btVector3& linVelA, const btVector3& linVelB, const btVector3& angVelA, const btVector3& angVelB, btConstraintInfo2* info, int row, btVector3& ax1, int rotational, int rotAllowed = false)
	{
		int count = 0;
		int srow = row * info->rowskip;

		if (limot->m_currentLimit == 4)
		{
			btScalar vel = rotational ? angVelA.dot(ax1) - angVelB.dot(ax1) : linVelA.dot(ax1) - linVelB.dot(ax1);

			calculateJacobi(limot, transA, transB, info, srow, ax1, rotational, rotAllowed);
			info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitError * (rotational ? -1 : 1);
			if (rotational)
			{
				if (info->m_constraintError[srow] - vel * limot->m_stopERP > 0)
				{
					btScalar bounceerror = -limot->m_bounce * vel;
					if (bounceerror > info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
				}
			}
			else
			{
				if (info->m_constraintError[srow] - vel * limot->m_stopERP < 0)
				{
					btScalar bounceerror = -limot->m_bounce * vel;
					if (bounceerror < info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
				}
			}
			info->m_lowerLimit[srow] = rotational ? 0 : -SIMD_INFINITY;
			info->m_upperLimit[srow] = rotational ? SIMD_INFINITY : 0;
			info->cfm[srow] = limot->m_stopCFM;
			srow += info->rowskip;
			++count;

			calculateJacobi(limot, transA, transB, info, srow, ax1, rotational, rotAllowed);
			info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitErrorHi * (rotational ? -1 : 1);
			if (rotational)
			{
				if (info->m_constraintError[srow] - vel * limot->m_stopERP < 0)
				{
					btScalar bounceerror = -limot->m_bounce * vel;
					if (bounceerror < info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
				}
			}
			else
			{
				if (info->m_constraintError[srow] - vel * limot->m_stopERP > 0)
				{
					btScalar bounceerror = -limot->m_bounce * vel;
					if (bounceerror > info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
				}
			}
			info->m_lowerLimit[srow] = rotational ? -SIMD_INFINITY : 0;
			info->m_upperLimit[srow] = rotational ? 0 : SIMD_INFINITY;
			info->cfm[srow] = limot->m_stopCFM;
			srow += info->rowskip;
			++count;
		}
		else if (limot->m_currentLimit == 3)
		{
			calculateJacobi(limot, transA, transB, info, srow, ax1, rotational, rotAllowed);
			info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitError * (rotational ? -1 : 1);
			info->m_lowerLimit[srow] = -SIMD_INFINITY;
			info->m_upperLimit[srow] = SIMD_INFINITY;
			info->cfm[srow] = limot->m_stopCFM;
			srow += info->rowskip;
			++count;
		}

		if (limot->m_enableMotor && !limot->m_servoMotor)
		{
			calculateJacobi(limot, transA, transB, info, srow, ax1, rotational, rotAllowed);
			btScalar tag_vel = rotational ? limot->m_targetVelocity : -limot->m_targetVelocity;
			btScalar mot_fact = getMotorFactor(limot->m_currentPosition,
											   limot->m_loLimit,
											   limot->m_hiLimit,
											   tag_vel,
											   info->fps * limot->m_motorERP);
			info->m_constraintError[srow] = mot_fact * limot->m_targetVelocity;
			info->m_lowerLimit[srow] = -limot->m_maxMotorForce / info->fps;
			info->m_upperLimit[srow] = limot->m_maxMotorForce / info->fps;
			info->cfm[srow] = limot->m_motorCFM;
			srow += info->rowskip;
			++count;
		}

		if (limot->m_enableMotor && limot->m_servoMotor)
		{
			btScalar error = limot->m_currentPosition - limot->m_servoTarget;
			btScalar curServoTarget = limot->m_servoTarget;
			if (rotational)
			{
				if (error > SIMD_PI)
				{
					error -= SIMD_2_PI;
					curServoTarget += SIMD_2_PI;
				}
				if (error < -SIMD_PI)
				{
					error += SIMD_2_PI;
					curServoTarget -= SIMD_2_PI;
				}
			}

			calculateJacobi(limot, transA, transB, info, srow, ax1, rotational, rotAllowed);
			btScalar targetvelocity = error < 0 ? -limot->m_targetVelocity : limot->m_targetVelocity;
			btScalar tag_vel = -targetvelocity;
			btScalar mot_fact;
			if (error != 0)
			{
				btScalar lowLimit;
				btScalar hiLimit;
				if (limot->m_loLimit > limot->m_hiLimit)
				{
					lowLimit = error > 0 ? curServoTarget : -SIMD_INFINITY;
					hiLimit = error < 0 ? curServoTarget : SIMD_INFINITY;
				}
				else
				{
					lowLimit = error > 0 && curServoTarget > limot->m_loLimit ? curServoTarget : limot->m_loLimit;
					hiLimit = error < 0 && curServoTarget < limot->m_hiLimit ? curServoTarget : limot->m_hiLimit;
				}
				mot_fact = getMotorFactor(limot->m_currentPosition, lowLimit, hiLimit, tag_vel, info->fps * limot->m_motorERP);
			}
			else
			{
				mot_fact = 0;
			}
			info->m_constraintError[srow] = mot_fact * targetvelocity * (rotational ? -1 : 1);
			info->m_lowerLimit[srow] = -limot->m_maxMotorForce / info->fps;
			info->m_upperLimit[srow] = limot->m_maxMotorForce / info->fps;
			info->cfm[srow] = limot->m_motorCFM;
			srow += info->rowskip;
			++count;
		}

		if (limot->m_enableSpring)
		{
			btScalar error = limot->m_currentPosition - limot->m_equilibriumPoint;
			calculateJacobi(limot, transA, transB, info, srow, ax1, rotational, rotAllowed);

			//btScalar cfm = 1.0 / ((1.0/info->fps)*limot->m_springStiffness+ limot->m_springDamping);
			//if(cfm > 0.99999)
			//	cfm = 0.99999;
			//btScalar erp = (1.0/info->fps)*limot->m_springStiffness / ((1.0/info->fps)*limot->m_springStiffness + limot->m_springDamping);
			//info->m_constraintError[srow] = info->fps * erp * error * (rotational ? -1.0 : 1.0);
			//info->m_lowerLimit[srow] = -SIMD_INFINITY;
			//info->m_upperLimit[srow] = SIMD_INFINITY;

			btScalar dt = BT_ONE / info->fps;
			btScalar kd = limot->m_springDamping;
			btScalar ks = limot->m_springStiffness;
			btScalar vel;
			if (rotational)
			{
				vel = angVelA.dot(ax1) - angVelB.dot(ax1);
			}
			else
			{
				btVector3 tanVelA = angVelA.cross(m_calculatedTransformA.getOrigin() - transA.getOrigin());
				btVector3 tanVelB = angVelB.cross(m_calculatedTransformB.getOrigin() - transB.getOrigin());
				vel = (linVelA + tanVelA).dot(ax1) - (linVelB + tanVelB).dot(ax1);
			}
			btScalar cfm = BT_ZERO;
			btScalar mA = BT_ONE / m_rbA.getInvMass();
			btScalar mB = BT_ONE / m_rbB.getInvMass();
			if (rotational)
			{
				btScalar rrA = (m_calculatedTransformA.getOrigin() - transA.getOrigin()).length2();
				btScalar rrB = (m_calculatedTransformB.getOrigin() - transB.getOrigin()).length2();
				if (m_rbA.getInvMass()) mA = mA * rrA + 1 / (m_rbA.getInvInertiaTensorWorld() * ax1).length();
				if (m_rbB.getInvMass()) mB = mB * rrB + 1 / (m_rbB.getInvInertiaTensorWorld() * ax1).length();
			}
			btScalar m;
			if (m_rbA.getInvMass() == 0)
				m = mB;
			else if (m_rbB.getInvMass() == 0)
				m = mA;
			else
				m = mA * mB / (mA + mB);
			btScalar angularfreq = btSqrt(ks / m);

			//limit stiffness (the spring should not be sampled faster that the quarter of its angular frequency)
			if (limot->m_springStiffnessLimited && 0.25 < angularfreq * dt)
			{
				ks = BT_ONE / dt / dt / btScalar(16.0) * m;
			}
			//avoid damping that would blow up the spring
			if (limot->m_springDampingLimited && kd * dt > m)
			{
				kd = m / dt;
			}
			btScalar fs = ks * error * dt;
			btScalar fd = -kd * (vel) * (rotational ? -1 : 1) * dt;
			btScalar f = (fs + fd);

			// after the spring force affecting the body(es) the new velocity will be
			// vel + f / m * (rotational ? -1 : 1)
			// so in theory this should be set here for m_constraintError
			// (with m_constraintError we set a desired velocity for the affected body(es))
			// however in practice any value is fine as long as it is greater than the "proper" velocity,
			// because the m_lowerLimit and the m_upperLimit will determinate the strength of the final pulling force
			// so it is much simpler (and more robust) just to simply use inf (with the proper sign)
			// (Even with our best intent the "new" velocity is only an estimation. If we underestimate
			// the "proper" velocity that will weaken the spring, however if we overestimate it, it doesn't
			// matter, because the solver will limit it according the force limit)
			// you may also wonder what if the current velocity (vel) so high that the pulling force will not change its direction (in this iteration)
			// will we not request a velocity with the wrong direction ?
			// and the answer is not, because in practice during the solving the current velocity is subtracted from the m_constraintError
			// so the sign of the force that is really matters
			if (m_flags & BT_6DOF_FLAGS_USE_INFINITE_ERROR)
				info->m_constraintError[srow] = (rotational ? -1 : 1) * (f < 0 ? -SIMD_INFINITY : SIMD_INFINITY);
			else
				info->m_constraintError[srow] = vel + f / m * (rotational ? -1 : 1);

			btScalar minf = f < fd ? f : fd;
			btScalar maxf = f < fd ? fd : f;
			if (!rotational)
			{
				info->m_lowerLimit[srow] = minf > 0 ? 0 : minf;
				info->m_upperLimit[srow] = maxf < 0 ? 0 : maxf;
			}
			else
			{
				info->m_lowerLimit[srow] = -maxf > 0 ? 0 : -maxf;
				info->m_upperLimit[srow] = -minf < 0 ? 0 : -minf;
			}

			info->cfm[srow] = cfm;
			srow += info->rowskip;
			++count;
		}

		return count;
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btGeneric6DofSpring2Constraint(btRigidBody & rbA, btRigidBody & rbB, const btTransform& frameInA, const btTransform& frameInB, RotateOrder rotOrder = RO_XYZ)
		: btTypedConstraint(D6_SPRING_2_CONSTRAINT_TYPE, rbA, rbB), m_frameInA(frameInA), m_frameInB(frameInB), m_rotateOrder(rotOrder), m_flags(0)
	{
		calculateTransforms();
	}
	btGeneric6DofSpring2Constraint(btRigidBody & rbB, const btTransform& frameInB, RotateOrder rotOrder = RO_XYZ)
		: btTypedConstraint(D6_SPRING_2_CONSTRAINT_TYPE, getFixedBody(), rbB), m_frameInB(frameInB), m_rotateOrder(rotOrder), m_flags(0)
	{
		///not providing rigidbody A means implicitly using worldspace for body A
		m_frameInA = rbB.getCenterOfMassTransform() * m_frameInB;
		calculateTransforms();
	}

	virtual void buildJacobian() {}
	virtual void getInfo1(btConstraintInfo1 * info)
	{
		//prepare constraint
		calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
		info->m_numConstraintRows = 0;
		info->nub = 0;
		int i;
		//test linear limits
		for (i = 0; i < 3; i++)
		{
			if (m_linearLimits.m_currentLimit[i] == 4)
				info->m_numConstraintRows += 2;
			else if (m_linearLimits.m_currentLimit[i] != 0)
				info->m_numConstraintRows += 1;
			if (m_linearLimits.m_enableMotor[i]) info->m_numConstraintRows += 1;
			if (m_linearLimits.m_enableSpring[i]) info->m_numConstraintRows += 1;
		}
		//test angular limits
		for (i = 0; i < 3; i++)
		{
			testAngularLimitMotor(i);
			if (m_angularLimits[i].m_currentLimit == 4)
				info->m_numConstraintRows += 2;
			else if (m_angularLimits[i].m_currentLimit != 0)
				info->m_numConstraintRows += 1;
			if (m_angularLimits[i].m_enableMotor) info->m_numConstraintRows += 1;
			if (m_angularLimits[i].m_enableSpring) info->m_numConstraintRows += 1;
		}
	}
	virtual void getInfo2(btConstraintInfo2 * info)
	{
		const btTransform& transA = m_rbA.getCenterOfMassTransform();
		const btTransform& transB = m_rbB.getCenterOfMassTransform();
		const btVector3& linVelA = m_rbA.getLinearVelocity();
		const btVector3& linVelB = m_rbB.getLinearVelocity();
		const btVector3& angVelA = m_rbA.getAngularVelocity();
		const btVector3& angVelB = m_rbB.getAngularVelocity();

		// for stability better to solve angular limits first
		int row = setAngularLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
		setLinearLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
	}
	virtual int calculateSerializeBufferSize() const;
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

	btRotationalLimitMotor2* getRotationalLimitMotor(int index) { return &m_angularLimits[index]; }
	btTranslationalLimitMotor2* getTranslationalLimitMotor() { return &m_linearLimits; }

	// Calculates the global transform for the joint offset for body A an B, and also calculates the angle differences between the bodies.
	void calculateTransforms(const btTransform& transA, const btTransform& transB)
	{
		m_calculatedTransformA = transA * m_frameInA;
		m_calculatedTransformB = transB * m_frameInB;
		calculateLinearInfo();
		calculateAngleInfo();

		btScalar miA = getRigidBodyA().getInvMass();
		btScalar miB = getRigidBodyB().getInvMass();
		m_hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
		btScalar miS = miA + miB;
		if (miS > btScalar(0.f))
		{
			m_factA = miB / miS;
		}
		else
		{
			m_factA = btScalar(0.5f);
		}
		m_factB = btScalar(1.0f) - m_factA;
	}
	void calculateTransforms()
	{
		calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
	}

	// Gets the global transform of the offset for body A
	const btTransform& getCalculatedTransformA() const { return m_calculatedTransformA; }
	// Gets the global transform of the offset for body B
	const btTransform& getCalculatedTransformB() const { return m_calculatedTransformB; }

	const btTransform& getFrameOffsetA() const { return m_frameInA; }
	const btTransform& getFrameOffsetB() const { return m_frameInB; }

	btTransform& getFrameOffsetA() { return m_frameInA; }
	btTransform& getFrameOffsetB() { return m_frameInB; }

	// Get the rotation axis in global coordinates ( btGeneric6DofSpring2Constraint::calculateTransforms() must be called previously )
	btVector3 getAxis(int axis_index) const { return m_calculatedAxis[axis_index]; }

	// Get the relative Euler angle ( btGeneric6DofSpring2Constraint::calculateTransforms() must be called previously )
	btScalar getAngle(int axis_index) const { return m_calculatedAxisAngleDiff[axis_index]; }

	// Get the relative position of the constraint pivot ( btGeneric6DofSpring2Constraint::calculateTransforms() must be called previously )
	btScalar getRelativePivotPosition(int axis_index) const { return m_calculatedLinearDiff[axis_index]; }

	void setFrames(const btTransform& frameA, const btTransform& frameB)
	{
		m_frameInA = frameA;
		m_frameInB = frameB;
		buildJacobian();
		calculateTransforms();
	}

	void setLinearLowerLimit(const btVector3& linearLower) { m_linearLimits.m_lowerLimit = linearLower; }
	void getLinearLowerLimit(btVector3 & linearLower) { linearLower = m_linearLimits.m_lowerLimit; }
	void setLinearUpperLimit(const btVector3& linearUpper) { m_linearLimits.m_upperLimit = linearUpper; }
	void getLinearUpperLimit(btVector3 & linearUpper) { linearUpper = m_linearLimits.m_upperLimit; }

	void setAngularLowerLimit(const btVector3& angularLower)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[i].m_loLimit = btNormalizeAngle(angularLower[i]);
	}

	void setAngularLowerLimitReversed(const btVector3& angularLower)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[i].m_hiLimit = btNormalizeAngle(-angularLower[i]);
	}

	void getAngularLowerLimit(btVector3 & angularLower)
	{
		for (int i = 0; i < 3; i++)
			angularLower[i] = m_angularLimits[i].m_loLimit;
	}

	void getAngularLowerLimitReversed(btVector3 & angularLower)
	{
		for (int i = 0; i < 3; i++)
			angularLower[i] = -m_angularLimits[i].m_hiLimit;
	}

	void setAngularUpperLimit(const btVector3& angularUpper)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[i].m_hiLimit = btNormalizeAngle(angularUpper[i]);
	}

	void setAngularUpperLimitReversed(const btVector3& angularUpper)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[i].m_loLimit = btNormalizeAngle(-angularUpper[i]);
	}

	void getAngularUpperLimit(btVector3 & angularUpper)
	{
		for (int i = 0; i < 3; i++)
			angularUpper[i] = m_angularLimits[i].m_hiLimit;
	}

	void getAngularUpperLimitReversed(btVector3 & angularUpper)
	{
		for (int i = 0; i < 3; i++)
			angularUpper[i] = -m_angularLimits[i].m_loLimit;
	}

	//first 3 are linear, next 3 are angular

	void setLimit(int axis, btScalar lo, btScalar hi)
	{
		if (axis < 3)
		{
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		}
		else
		{
			lo = btNormalizeAngle(lo);
			hi = btNormalizeAngle(hi);
			m_angularLimits[axis - 3].m_loLimit = lo;
			m_angularLimits[axis - 3].m_hiLimit = hi;
		}
	}

	void setLimitReversed(int axis, btScalar lo, btScalar hi)
	{
		if (axis < 3)
		{
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		}
		else
		{
			lo = btNormalizeAngle(lo);
			hi = btNormalizeAngle(hi);
			m_angularLimits[axis - 3].m_hiLimit = -lo;
			m_angularLimits[axis - 3].m_loLimit = -hi;
		}
	}

	bool isLimited(int limitIndex)
	{
		if (limitIndex < 3)
		{
			return m_linearLimits.isLimited(limitIndex);
		}
		return m_angularLimits[limitIndex - 3].isLimited();
	}

	void setRotationOrder(RotateOrder order) { m_rotateOrder = order; }
	RotateOrder getRotationOrder() { return m_rotateOrder; }

	void setAxis(const btVector3& axis1, const btVector3& axis2)
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

	void setBounce(int index, btScalar bounce)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
			m_linearLimits.m_bounce[index] = bounce;
		else
			m_angularLimits[index - 3].m_bounce = bounce;
	}

	void enableMotor(int index, bool onOff)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
			m_linearLimits.m_enableMotor[index] = onOff;
		else
			m_angularLimits[index - 3].m_enableMotor = onOff;
	}
	void setServo(int index, bool onOff)   // set the type of the motor (servo or not) (the motor has to be turned on for servo also)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
			m_linearLimits.m_servoMotor[index] = onOff;
		else
			m_angularLimits[index - 3].m_servoMotor = onOff;
	}
	void setTargetVelocity(int index, btScalar velocity)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
			m_linearLimits.m_targetVelocity[index] = velocity;
		else
			m_angularLimits[index - 3].m_targetVelocity = velocity;
	}
	void setServoTarget(int index, btScalar targetOrg)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
		{
			m_linearLimits.m_servoTarget[index] = targetOrg;
		}
		else
		{
			//wrap between -PI and PI, see also
			//https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi

			btScalar target = targetOrg + SIMD_PI;
			if (1)
			{
				btScalar m = target - SIMD_2_PI * std::floor(target / SIMD_2_PI);
				// handle boundary cases resulted from floating-point cut off:
				{
					if (m >= SIMD_2_PI)
					{
						target = 0;
					}
					else
					{
						if (m < 0)
						{
							if (SIMD_2_PI + m == SIMD_2_PI)
								target = 0;
							else
								target = SIMD_2_PI + m;
						}
						else
						{
							target = m;
						}
					}
				}
				target -= SIMD_PI;
			}

			m_angularLimits[index - 3].m_servoTarget = target;
		}
	}
	void setMaxMotorForce(int index, btScalar force)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
			m_linearLimits.m_maxMotorForce[index] = force;
		else
			m_angularLimits[index - 3].m_maxMotorForce = force;
	}

	void enableSpring(int index, bool onOff)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
			m_linearLimits.m_enableSpring[index] = onOff;
		else
			m_angularLimits[index - 3].m_enableSpring = onOff;
	}
	void setStiffness(int index, btScalar stiffness, bool limitIfNeeded = true)  // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
		{
			m_linearLimits.m_springStiffness[index] = stiffness;
			m_linearLimits.m_springStiffnessLimited[index] = limitIfNeeded;
		}
		else
		{
			m_angularLimits[index - 3].m_springStiffness = stiffness;
			m_angularLimits[index - 3].m_springStiffnessLimited = limitIfNeeded;
		}
	}
	void setDamping(int index, btScalar damping, bool limitIfNeeded = true)      // if limitIfNeeded is true the system will automatically limit the damping in necessary situations where otherwise the spring would blow up
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
		{
			m_linearLimits.m_springDamping[index] = damping;
			m_linearLimits.m_springDampingLimited[index] = limitIfNeeded;
		}
		else
		{
			m_angularLimits[index - 3].m_springDamping = damping;
			m_angularLimits[index - 3].m_springDampingLimited = limitIfNeeded;
		}
	}
	void setEquilibriumPoint()                                                   // set the current constraint position/orientation as an equilibrium point for all DOF
	{
		calculateTransforms();
		int i;
		for (i = 0; i < 3; i++)
			m_linearLimits.m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
		for (i = 0; i < 3; i++)
			m_angularLimits[i].m_equilibriumPoint = m_calculatedAxisAngleDiff[i];
	}
	void setEquilibriumPoint(int index)                                          // set the current constraint position/orientation as an equilibrium point for given DOF
	{
		btAssert((index >= 0) && (index < 6));
		calculateTransforms();
		if (index < 3)
			m_linearLimits.m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
		else
			m_angularLimits[index - 3].m_equilibriumPoint = m_calculatedAxisAngleDiff[index - 3];
	}
	void setEquilibriumPoint(int index, btScalar val)
	{
		btAssert((index >= 0) && (index < 6));
		if (index < 3)
			m_linearLimits.m_equilibriumPoint[index] = val;
		else
			m_angularLimits[index - 3].m_equilibriumPoint = val;
	}

	//override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
	//If no axis is provided, it uses the default axis for this constraint.
	virtual void setParam(int num, btScalar value, int axis = -1)
	{
		if ((axis >= 0) && (axis < 3))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					m_linearLimits.m_stopERP[axis] = value;
					m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				case BT_CONSTRAINT_STOP_CFM:
					m_linearLimits.m_stopCFM[axis] = value;
					m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				case BT_CONSTRAINT_ERP:
					m_linearLimits.m_motorERP[axis] = value;
					m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				case BT_CONSTRAINT_CFM:
					m_linearLimits.m_motorCFM[axis] = value;
					m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				default:
					btAssertConstrParams(0);
			}
		}
		else if ((axis >= 3) && (axis < 6))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					m_angularLimits[axis - 3].m_stopERP = value;
					m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				case BT_CONSTRAINT_STOP_CFM:
					m_angularLimits[axis - 3].m_stopCFM = value;
					m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				case BT_CONSTRAINT_ERP:
					m_angularLimits[axis - 3].m_motorERP = value;
					m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				case BT_CONSTRAINT_CFM:
					m_angularLimits[axis - 3].m_motorCFM = value;
					m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
					break;
				default:
					btAssertConstrParams(0);
			}
		}
		else
		{
			btAssertConstrParams(0);
		}
	}
	//return the local value of parameter
	virtual btScalar getParam(int num, int axis = -1) const
	{
		btScalar retVal = 0;
		if ((axis >= 0) && (axis < 3))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_linearLimits.m_stopERP[axis];
					break;
				case BT_CONSTRAINT_STOP_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_linearLimits.m_stopCFM[axis];
					break;
				case BT_CONSTRAINT_ERP:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_linearLimits.m_motorERP[axis];
					break;
				case BT_CONSTRAINT_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_linearLimits.m_motorCFM[axis];
					break;
				default:
					btAssertConstrParams(0);
			}
		}
		else if ((axis >= 3) && (axis < 6))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_angularLimits[axis - 3].m_stopERP;
					break;
				case BT_CONSTRAINT_STOP_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_angularLimits[axis - 3].m_stopCFM;
					break;
				case BT_CONSTRAINT_ERP:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_angularLimits[axis - 3].m_motorERP;
					break;
				case BT_CONSTRAINT_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
					retVal = m_angularLimits[axis - 3].m_motorCFM;
					break;
				default:
					btAssertConstrParams(0);
			}
		}
		else
		{
			btAssertConstrParams(0);
		}
		return retVal;
	}

	static btScalar btGetMatrixElem(const btMatrix3x3& mat, int index)
	{
		int i = index % 3;
		int j = index / 3;
		return mat[i][j];
	}
	
	// MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
	static bool matrixToEulerXYZ(const btMatrix3x3& mat, btVector3& xyz)
	{
		// rot =  cy*cz          -cy*sz           sy
		//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
		//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy

		btScalar fi = btGetMatrixElem(mat, 2);
		if (fi < btScalar(1.0f))
		{
			if (fi > btScalar(-1.0f))
			{
				xyz[0] = btAtan2(-btGetMatrixElem(mat, 5), btGetMatrixElem(mat, 8));
				xyz[1] = btAsin(btGetMatrixElem(mat, 2));
				xyz[2] = btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 0));
				return true;
			}
			else
			{
				// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
				xyz[0] = -btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
				xyz[1] = -SIMD_HALF_PI;
				xyz[2] = btScalar(0.0);
				return false;
			}
		}
		else
		{
			// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
			xyz[0] = btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
			xyz[1] = SIMD_HALF_PI;
			xyz[2] = 0.0;
		}
		return false;
	}
	static bool matrixToEulerXZY(const btMatrix3x3& mat, btVector3& xyz)
	{
		// rot =  cy*cz          -sz           sy*cz
		//        cy*cx*sz+sx*sy  cx*cz        sy*cx*sz-cy*sx
		//        cy*sx*sz-cx*sy  sx*cz        sy*sx*sz+cx*cy

		btScalar fi = btGetMatrixElem(mat, 1);
		if (fi < btScalar(1.0f))
		{
			if (fi > btScalar(-1.0f))
			{
				xyz[0] = btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 4));
				xyz[1] = btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 0));
				xyz[2] = btAsin(-btGetMatrixElem(mat, 1));
				return true;
			}
			else
			{
				xyz[0] = -btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 8));
				xyz[1] = btScalar(0.0);
				xyz[2] = SIMD_HALF_PI;
				return false;
			}
		}
		else
		{
			xyz[0] = btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 8));
			xyz[1] = 0.0;
			xyz[2] = -SIMD_HALF_PI;
		}
		return false;
	}
	static bool matrixToEulerYXZ(const btMatrix3x3& mat, btVector3& xyz)
	{
		// rot =  cy*cz+sy*sx*sz  cz*sy*sx-cy*sz  cx*sy
		//        cx*sz           cx*cz           -sx
		//        cy*sx*sz-cz*sy  sy*sz+cy*cz*sx  cy*cx

		btScalar fi = btGetMatrixElem(mat, 5);
		if (fi < btScalar(1.0f))
		{
			if (fi > btScalar(-1.0f))
			{
				xyz[0] = btAsin(-btGetMatrixElem(mat, 5));
				xyz[1] = btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 8));
				xyz[2] = btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 4));
				return true;
			}
			else
			{
				xyz[0] = SIMD_HALF_PI;
				xyz[1] = -btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 0));
				xyz[2] = btScalar(0.0);
				return false;
			}
		}
		else
		{
			xyz[0] = -SIMD_HALF_PI;
			xyz[1] = btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 0));
			xyz[2] = 0.0;
		}
		return false;
	}
	static bool matrixToEulerYZX(const btMatrix3x3& mat, btVector3& xyz)
	{
		// rot =  cy*cz   sy*sx-cy*cx*sz   cx*sy+cy*sz*sx
		//        sz           cz*cx           -cz*sx
		//        -cz*sy  cy*sx+cx*sy*sz   cy*cx-sy*sz*sx

		btScalar fi = btGetMatrixElem(mat, 3);
		if (fi < btScalar(1.0f))
		{
			if (fi > btScalar(-1.0f))
			{
				xyz[0] = btAtan2(-btGetMatrixElem(mat, 5), btGetMatrixElem(mat, 4));
				xyz[1] = btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 0));
				xyz[2] = btAsin(btGetMatrixElem(mat, 3));
				return true;
			}
			else
			{
				xyz[0] = btScalar(0.0);
				xyz[1] = -btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 8));
				xyz[2] = -SIMD_HALF_PI;
				return false;
			}
		}
		else
		{
			xyz[0] = btScalar(0.0);
			xyz[1] = btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 8));
			xyz[2] = SIMD_HALF_PI;
		}
		return false;
	}
	static bool matrixToEulerZXY(const btMatrix3x3& mat, btVector3& xyz)
	{
		// rot =  cz*cy-sz*sx*sy    -cx*sz   cz*sy+cy*sz*sx
		//        cy*sz+cz*sx*sy     cz*cx   sz*sy-cz*xy*sx
		//        -cx*sy              sx     cx*cy

		btScalar fi = btGetMatrixElem(mat, 7);
		if (fi < btScalar(1.0f))
		{
			if (fi > btScalar(-1.0f))
			{
				xyz[0] = btAsin(btGetMatrixElem(mat, 7));
				xyz[1] = btAtan2(-btGetMatrixElem(mat, 6), btGetMatrixElem(mat, 8));
				xyz[2] = btAtan2(-btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 4));
				return true;
			}
			else
			{
				xyz[0] = -SIMD_HALF_PI;
				xyz[1] = btScalar(0.0);
				xyz[2] = -btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 0));
				return false;
			}
		}
		else
		{
			xyz[0] = SIMD_HALF_PI;
			xyz[1] = btScalar(0.0);
			xyz[2] = btAtan2(btGetMatrixElem(mat, 2), btGetMatrixElem(mat, 0));
		}
		return false;
	}
	static bool matrixToEulerZYX(const btMatrix3x3& mat, btVector3& xyz)
	{
		// rot =  cz*cy   cz*sy*sx-cx*sz   sz*sx+cz*cx*sy
		//        cy*sz   cz*cx+sz*sy*sx   cx*sz*sy-cz*sx
		//        -sy          cy*sx         cy*cx

		btScalar fi = btGetMatrixElem(mat, 6);
		if (fi < btScalar(1.0f))
		{
			if (fi > btScalar(-1.0f))
			{
				xyz[0] = btAtan2(btGetMatrixElem(mat, 7), btGetMatrixElem(mat, 8));
				xyz[1] = btAsin(-btGetMatrixElem(mat, 6));
				xyz[2] = btAtan2(btGetMatrixElem(mat, 3), btGetMatrixElem(mat, 0));
				return true;
			}
			else
			{
				xyz[0] = btScalar(0.0);
				xyz[1] = SIMD_HALF_PI;
				xyz[2] = -btAtan2(btGetMatrixElem(mat, 1), btGetMatrixElem(mat, 2));
				return false;
			}
		}
		else
		{
			xyz[0] = btScalar(0.0);
			xyz[1] = -SIMD_HALF_PI;
			xyz[2] = btAtan2(-btGetMatrixElem(mat, 1), -btGetMatrixElem(mat, 2));
		}
		return false;
	}
};

struct btGeneric6DofSpring2ConstraintData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformFloatData m_rbAFrame;
	btTransformFloatData m_rbBFrame;

	btVector3FloatData m_linearUpperLimit;
	btVector3FloatData m_linearLowerLimit;
	btVector3FloatData m_linearBounce;
	btVector3FloatData m_linearStopERP;
	btVector3FloatData m_linearStopCFM;
	btVector3FloatData m_linearMotorERP;
	btVector3FloatData m_linearMotorCFM;
	btVector3FloatData m_linearTargetVelocity;
	btVector3FloatData m_linearMaxMotorForce;
	btVector3FloatData m_linearServoTarget;
	btVector3FloatData m_linearSpringStiffness;
	btVector3FloatData m_linearSpringDamping;
	btVector3FloatData m_linearEquilibriumPoint;
	char m_linearEnableMotor[4];
	char m_linearServoMotor[4];
	char m_linearEnableSpring[4];
	char m_linearSpringStiffnessLimited[4];
	char m_linearSpringDampingLimited[4];
	char m_padding1[4];

	btVector3FloatData m_angularUpperLimit;
	btVector3FloatData m_angularLowerLimit;
	btVector3FloatData m_angularBounce;
	btVector3FloatData m_angularStopERP;
	btVector3FloatData m_angularStopCFM;
	btVector3FloatData m_angularMotorERP;
	btVector3FloatData m_angularMotorCFM;
	btVector3FloatData m_angularTargetVelocity;
	btVector3FloatData m_angularMaxMotorForce;
	btVector3FloatData m_angularServoTarget;
	btVector3FloatData m_angularSpringStiffness;
	btVector3FloatData m_angularSpringDamping;
	btVector3FloatData m_angularEquilibriumPoint;
	char m_angularEnableMotor[4];
	char m_angularServoMotor[4];
	char m_angularEnableSpring[4];
	char m_angularSpringStiffnessLimited[4];
	char m_angularSpringDampingLimited[4];

	int m_rotateOrder;
};

struct btGeneric6DofSpring2ConstraintDoubleData2
{
	btTypedConstraintDoubleData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame;
	btTransformDoubleData m_rbBFrame;

	btVector3DoubleData m_linearUpperLimit;
	btVector3DoubleData m_linearLowerLimit;
	btVector3DoubleData m_linearBounce;
	btVector3DoubleData m_linearStopERP;
	btVector3DoubleData m_linearStopCFM;
	btVector3DoubleData m_linearMotorERP;
	btVector3DoubleData m_linearMotorCFM;
	btVector3DoubleData m_linearTargetVelocity;
	btVector3DoubleData m_linearMaxMotorForce;
	btVector3DoubleData m_linearServoTarget;
	btVector3DoubleData m_linearSpringStiffness;
	btVector3DoubleData m_linearSpringDamping;
	btVector3DoubleData m_linearEquilibriumPoint;
	char m_linearEnableMotor[4];
	char m_linearServoMotor[4];
	char m_linearEnableSpring[4];
	char m_linearSpringStiffnessLimited[4];
	char m_linearSpringDampingLimited[4];
	char m_padding1[4];

	btVector3DoubleData m_angularUpperLimit;
	btVector3DoubleData m_angularLowerLimit;
	btVector3DoubleData m_angularBounce;
	btVector3DoubleData m_angularStopERP;
	btVector3DoubleData m_angularStopCFM;
	btVector3DoubleData m_angularMotorERP;
	btVector3DoubleData m_angularMotorCFM;
	btVector3DoubleData m_angularTargetVelocity;
	btVector3DoubleData m_angularMaxMotorForce;
	btVector3DoubleData m_angularServoTarget;
	btVector3DoubleData m_angularSpringStiffness;
	btVector3DoubleData m_angularSpringDamping;
	btVector3DoubleData m_angularEquilibriumPoint;
	char m_angularEnableMotor[4];
	char m_angularServoMotor[4];
	char m_angularEnableSpring[4];
	char m_angularSpringStiffnessLimited[4];
	char m_angularSpringDampingLimited[4];

	int m_rotateOrder;
};

SIMD_FORCE_INLINE int btGeneric6DofSpring2Constraint::calculateSerializeBufferSize() const
{
	return sizeof(btGeneric6DofSpring2ConstraintData2);
}

SIMD_FORCE_INLINE const char* btGeneric6DofSpring2Constraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btGeneric6DofSpring2ConstraintData2* dof = (btGeneric6DofSpring2ConstraintData2*)dataBuffer;
	btTypedConstraint::serialize(&dof->m_typeConstraintData, serializer);

	m_frameInA.serialize(dof->m_rbAFrame);
	m_frameInB.serialize(dof->m_rbBFrame);

	int i;
	for (i = 0; i < 3; i++)
	{
		dof->m_angularLowerLimit.m_floats[i] = m_angularLimits[i].m_loLimit;
		dof->m_angularUpperLimit.m_floats[i] = m_angularLimits[i].m_hiLimit;
		dof->m_angularBounce.m_floats[i] = m_angularLimits[i].m_bounce;
		dof->m_angularStopERP.m_floats[i] = m_angularLimits[i].m_stopERP;
		dof->m_angularStopCFM.m_floats[i] = m_angularLimits[i].m_stopCFM;
		dof->m_angularMotorERP.m_floats[i] = m_angularLimits[i].m_motorERP;
		dof->m_angularMotorCFM.m_floats[i] = m_angularLimits[i].m_motorCFM;
		dof->m_angularTargetVelocity.m_floats[i] = m_angularLimits[i].m_targetVelocity;
		dof->m_angularMaxMotorForce.m_floats[i] = m_angularLimits[i].m_maxMotorForce;
		dof->m_angularServoTarget.m_floats[i] = m_angularLimits[i].m_servoTarget;
		dof->m_angularSpringStiffness.m_floats[i] = m_angularLimits[i].m_springStiffness;
		dof->m_angularSpringDamping.m_floats[i] = m_angularLimits[i].m_springDamping;
		dof->m_angularEquilibriumPoint.m_floats[i] = m_angularLimits[i].m_equilibriumPoint;
	}
	dof->m_angularLowerLimit.m_floats[3] = 0;
	dof->m_angularUpperLimit.m_floats[3] = 0;
	dof->m_angularBounce.m_floats[3] = 0;
	dof->m_angularStopERP.m_floats[3] = 0;
	dof->m_angularStopCFM.m_floats[3] = 0;
	dof->m_angularMotorERP.m_floats[3] = 0;
	dof->m_angularMotorCFM.m_floats[3] = 0;
	dof->m_angularTargetVelocity.m_floats[3] = 0;
	dof->m_angularMaxMotorForce.m_floats[3] = 0;
	dof->m_angularServoTarget.m_floats[3] = 0;
	dof->m_angularSpringStiffness.m_floats[3] = 0;
	dof->m_angularSpringDamping.m_floats[3] = 0;
	dof->m_angularEquilibriumPoint.m_floats[3] = 0;
	for (i = 0; i < 4; i++)
	{
		dof->m_angularEnableMotor[i] = i < 3 ? (m_angularLimits[i].m_enableMotor ? 1 : 0) : 0;
		dof->m_angularServoMotor[i] = i < 3 ? (m_angularLimits[i].m_servoMotor ? 1 : 0) : 0;
		dof->m_angularEnableSpring[i] = i < 3 ? (m_angularLimits[i].m_enableSpring ? 1 : 0) : 0;
		dof->m_angularSpringStiffnessLimited[i] = i < 3 ? (m_angularLimits[i].m_springStiffnessLimited ? 1 : 0) : 0;
		dof->m_angularSpringDampingLimited[i] = i < 3 ? (m_angularLimits[i].m_springDampingLimited ? 1 : 0) : 0;
	}

	m_linearLimits.m_lowerLimit.serialize(dof->m_linearLowerLimit);
	m_linearLimits.m_upperLimit.serialize(dof->m_linearUpperLimit);
	m_linearLimits.m_bounce.serialize(dof->m_linearBounce);
	m_linearLimits.m_stopERP.serialize(dof->m_linearStopERP);
	m_linearLimits.m_stopCFM.serialize(dof->m_linearStopCFM);
	m_linearLimits.m_motorERP.serialize(dof->m_linearMotorERP);
	m_linearLimits.m_motorCFM.serialize(dof->m_linearMotorCFM);
	m_linearLimits.m_targetVelocity.serialize(dof->m_linearTargetVelocity);
	m_linearLimits.m_maxMotorForce.serialize(dof->m_linearMaxMotorForce);
	m_linearLimits.m_servoTarget.serialize(dof->m_linearServoTarget);
	m_linearLimits.m_springStiffness.serialize(dof->m_linearSpringStiffness);
	m_linearLimits.m_springDamping.serialize(dof->m_linearSpringDamping);
	m_linearLimits.m_equilibriumPoint.serialize(dof->m_linearEquilibriumPoint);
	for (i = 0; i < 4; i++)
	{
		dof->m_linearEnableMotor[i] = i < 3 ? (m_linearLimits.m_enableMotor[i] ? 1 : 0) : 0;
		dof->m_linearServoMotor[i] = i < 3 ? (m_linearLimits.m_servoMotor[i] ? 1 : 0) : 0;
		dof->m_linearEnableSpring[i] = i < 3 ? (m_linearLimits.m_enableSpring[i] ? 1 : 0) : 0;
		dof->m_linearSpringStiffnessLimited[i] = i < 3 ? (m_linearLimits.m_springStiffnessLimited[i] ? 1 : 0) : 0;
		dof->m_linearSpringDampingLimited[i] = i < 3 ? (m_linearLimits.m_springDampingLimited[i] ? 1 : 0) : 0;
	}

	dof->m_rotateOrder = m_rotateOrder;

	dof->m_padding1[0] = 0;
	dof->m_padding1[1] = 0;
	dof->m_padding1[2] = 0;
	dof->m_padding1[3] = 0;

	return btGeneric6DofSpring2ConstraintDataName;
}

#endif  //BT_GENERIC_6DOF_CONSTRAINT_H
