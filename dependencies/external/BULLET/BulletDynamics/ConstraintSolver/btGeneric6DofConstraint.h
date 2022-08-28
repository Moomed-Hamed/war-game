// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/

#ifndef BT_GENERIC_6DOF_CONSTRAINT_H
#define BT_GENERIC_6DOF_CONSTRAINT_H

#include "btTypedConstraint.h"

#define D6_USE_OBSOLETE_METHOD false
#define D6_USE_FRAME_OFFSET true
#define GENERIC_D6_DISABLE_WARMSTARTING 1

#define btGeneric6DofConstraintData2 btGeneric6DofConstraintData
#define btGeneric6DofConstraintDataName "btGeneric6DofConstraintData"

static btScalar btGetMatrixElem(const btMatrix3x3& mat, int index)
{
	int i = index % 3;
	int j = index / 3;
	return mat[i][j];
}

///MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
static bool matrixToEulerXYZ(const btMatrix3x3& mat, btVector3& xyz)
{
	//	// rot =  cy*cz          -cy*sz           sy
	//	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
	//	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
	//

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

//! Rotation Limit structure for generic joints
class btRotationalLimitMotor
{
public:
	// limit_parameters
	btScalar m_loLimit;        // joint limit
	btScalar m_hiLimit;        // joint limit
	btScalar m_targetVelocity; // target motor velocity
	btScalar m_maxMotorForce;  // max force on motor
	btScalar m_maxLimitForce;  // max force on limit
	btScalar m_damping;        // Damping.
	btScalar m_limitSoftness;  // elaxation factor
	btScalar m_normalCFM;      // Constraint force mixing factor
	btScalar m_stopERP;        // Error tolerance factor when joint is at limit
	btScalar m_stopCFM;        // Constraint force mixing factor when joint is at limit
	btScalar m_bounce;         // restitution factor
	bool m_enableMotor;

	// temp_variables
	btScalar m_currentLimitError; // How much is violated this limit
	btScalar m_currentPosition;   // current value of angle
	int m_currentLimit;           // 0=free, 1=at lo limit, 2=at hi limit
	btScalar m_accumulatedImpulse;

	btRotationalLimitMotor()
	{
		m_accumulatedImpulse = 0.f;
		m_targetVelocity = 0;
		m_maxMotorForce = 6.0f;
		m_maxLimitForce = 300.0f;
		m_loLimit = 1.0f;
		m_hiLimit = -1.0f;
		m_normalCFM = 0.f;
		m_stopERP = 0.2f;
		m_stopCFM = 0.f;
		m_bounce = 0.0f;
		m_damping = 1.0f;
		m_limitSoftness = 0.5f;
		m_currentLimit = 0;
		m_currentLimitError = 0;
		m_enableMotor = false;
	}

	btRotationalLimitMotor(const btRotationalLimitMotor& limot)
	{
		m_targetVelocity = limot.m_targetVelocity;
		m_maxMotorForce = limot.m_maxMotorForce;
		m_limitSoftness = limot.m_limitSoftness;
		m_loLimit = limot.m_loLimit;
		m_hiLimit = limot.m_hiLimit;
		m_normalCFM = limot.m_normalCFM;
		m_stopERP = limot.m_stopERP;
		m_stopCFM = limot.m_stopCFM;
		m_bounce = limot.m_bounce;
		m_currentLimit = limot.m_currentLimit;
		m_currentLimitError = limot.m_currentLimitError;
		m_enableMotor = limot.m_enableMotor;
	}

	//! Is limited
	bool isLimited() const
	{
		if (m_loLimit > m_hiLimit) return false;
		return true;
	}

	//! Need apply correction
	bool needApplyTorques() const
	{
		if (m_currentLimit == 0 && m_enableMotor == false) return false;
		return true;
	}

	//! calculates  error
	/*!
	calculates m_currentLimit and m_currentLimitError.
	*/
	int testLimitValue(btScalar test_value)
	{
		if (m_loLimit > m_hiLimit)
		{
			m_currentLimit = 0;  //Free from violation
			return 0;
		}
		if (test_value < m_loLimit)
		{
			m_currentLimit = 1;  //low limit violation
			m_currentLimitError = test_value - m_loLimit;
			if (m_currentLimitError > SIMD_PI)
				m_currentLimitError -= SIMD_2_PI;
			else if (m_currentLimitError < -SIMD_PI)
				m_currentLimitError += SIMD_2_PI;
			return 1;
		}
		else if (test_value > m_hiLimit)
		{
			m_currentLimit = 2;  //High limit violation
			m_currentLimitError = test_value - m_hiLimit;
			if (m_currentLimitError > SIMD_PI)
				m_currentLimitError -= SIMD_2_PI;
			else if (m_currentLimitError < -SIMD_PI)
				m_currentLimitError += SIMD_2_PI;
			return 2;
		};

		m_currentLimit = 0;  //Free from violation
		return 0;
	}

	// apply the correction impulses for two bodies
	btScalar solveAngularLimits(btScalar timeStep, btVector3& axis, btScalar jacDiagABInv, btRigidBody* body0, btRigidBody* body1)
	{
		if (needApplyTorques() == false) return 0.0f;

		btScalar target_velocity = m_targetVelocity;
		btScalar maxMotorForce = m_maxMotorForce;

		//current error correction
		if (m_currentLimit != 0)
		{
			target_velocity = -m_stopERP * m_currentLimitError / (timeStep);
			maxMotorForce = m_maxLimitForce;
		}

		maxMotorForce *= timeStep;

		// current velocity difference

		btVector3 angVelA = body0->getAngularVelocity();
		btVector3 angVelB = body1->getAngularVelocity();

		btVector3 vel_diff;
		vel_diff = angVelA - angVelB;

		btScalar rel_vel = axis.dot(vel_diff);

		// correction velocity
		btScalar motor_relvel = m_limitSoftness * (target_velocity - m_damping * rel_vel);

		if (motor_relvel < SIMD_EPSILON && motor_relvel > -SIMD_EPSILON)
		{
			return 0.0f;  //no need for applying force
		}

		// correction impulse
		btScalar unclippedMotorImpulse = (1 + m_bounce) * motor_relvel * jacDiagABInv;

		// clip correction impulse
		btScalar clippedMotorImpulse;

		///@todo: should clip against accumulated impulse
		if (unclippedMotorImpulse > 0.0f)
		{
			clippedMotorImpulse = unclippedMotorImpulse > maxMotorForce ? maxMotorForce : unclippedMotorImpulse;
		}
		else
		{
			clippedMotorImpulse = unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce : unclippedMotorImpulse;
		}

		// sort with accumulated impulses
		btScalar lo = btScalar(-BT_LARGE_FLOAT);
		btScalar hi = btScalar(BT_LARGE_FLOAT);

		btScalar oldaccumImpulse = m_accumulatedImpulse;
		btScalar sum = oldaccumImpulse + clippedMotorImpulse;
		m_accumulatedImpulse = sum > hi ? btScalar(0.) : sum < lo ? btScalar(0.)
																  : sum;

		clippedMotorImpulse = m_accumulatedImpulse - oldaccumImpulse;

		btVector3 motorImp = clippedMotorImpulse * axis;

		body0->applyTorqueImpulse(motorImp);
		body1->applyTorqueImpulse(-motorImp);

		return clippedMotorImpulse;
	}
};

class btTranslationalLimitMotor
{
public:
	btVector3 m_lowerLimit; // the constraint lower limits
	btVector3 m_upperLimit; // the constraint upper limits
	btVector3 m_accumulatedImpulse;
	// Linear_Limit_parameters {
	btScalar m_limitSoftness; // Softness for linear limit
	btScalar m_damping;       // Damping for linear limit
	btScalar m_restitution;   // Bounce parameter for linear limit
	btVector3 m_normalCFM;    // Constraint force mixing factor
	btVector3 m_stopERP;      // Error tolerance factor when joint is at limit
	btVector3 m_stopCFM;      // Constraint force mixing factor when joint is at limit
	// }
	bool m_enableMotor[3];
	btVector3 m_targetVelocity;    // target motor velocity
	btVector3 m_maxMotorForce;     // max force on motor
	btVector3 m_currentLimitError; // How much is violated this limit
	btVector3 m_currentLinearDiff; // Current relative offset of constraint frames
	int m_currentLimit[3];         // 0=free, 1=at lower limit, 2=at upper limit

	btTranslationalLimitMotor()
	{
		m_lowerLimit.setValue(0.f, 0.f, 0.f);
		m_upperLimit.setValue(0.f, 0.f, 0.f);
		m_accumulatedImpulse.setValue(0.f, 0.f, 0.f);
		m_normalCFM.setValue(0.f, 0.f, 0.f);
		m_stopERP.setValue(0.2f, 0.2f, 0.2f);
		m_stopCFM.setValue(0.f, 0.f, 0.f);

		m_limitSoftness = 0.7f;
		m_damping = btScalar(1.0f);
		m_restitution = btScalar(0.5f);
		for (int i = 0; i < 3; i++)
		{
			m_enableMotor[i] = false;
			m_targetVelocity[i] = btScalar(0.f);
			m_maxMotorForce[i] = btScalar(0.f);
		}
	}

	btTranslationalLimitMotor(const btTranslationalLimitMotor& other)
	{
		m_lowerLimit = other.m_lowerLimit;
		m_upperLimit = other.m_upperLimit;
		m_accumulatedImpulse = other.m_accumulatedImpulse;

		m_limitSoftness = other.m_limitSoftness;
		m_damping = other.m_damping;
		m_restitution = other.m_restitution;
		m_normalCFM = other.m_normalCFM;
		m_stopERP = other.m_stopERP;
		m_stopCFM = other.m_stopCFM;

		for (int i = 0; i < 3; i++)
		{
			m_enableMotor[i] = other.m_enableMotor[i];
			m_targetVelocity[i] = other.m_targetVelocity[i];
			m_maxMotorForce[i] = other.m_maxMotorForce[i];
		}
	}

	//! Test limit
	/*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 are linear, next 3 are angular
    */
	inline bool isLimited(int limitIndex) const
	{
		return (m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex]);
	}
	inline bool needApplyForce(int limitIndex) const
	{
		if (m_currentLimit[limitIndex] == 0 && m_enableMotor[limitIndex] == false) return false;
		return true;
	}
	int testLimitValue(int limitIndex, btScalar test_value)
	{
		btScalar loLimit = m_lowerLimit[limitIndex];
		btScalar hiLimit = m_upperLimit[limitIndex];
		if (loLimit > hiLimit)
		{
			m_currentLimit[limitIndex] = 0;  //Free from violation
			m_currentLimitError[limitIndex] = btScalar(0.f);
			return 0;
		}

		if (test_value < loLimit)
		{
			m_currentLimit[limitIndex] = 2;  //low limit violation
			m_currentLimitError[limitIndex] = test_value - loLimit;
			return 2;
		}
		else if (test_value > hiLimit)
		{
			m_currentLimit[limitIndex] = 1;  //High limit violation
			m_currentLimitError[limitIndex] = test_value - hiLimit;
			return 1;
		};

		m_currentLimit[limitIndex] = 0;  //Free from violation
		m_currentLimitError[limitIndex] = btScalar(0.f);
		return 0;
	}

	btScalar solveLinearAxis(btScalar timeStep, btScalar jacDiagABInv, btRigidBody& body1, const btVector3& pointInA, btRigidBody& body2, const btVector3& pointInB, int limit_index, const btVector3& axis_normal_on_a, const btVector3& anchorPos)
	{
		// find relative velocity
		// btVector3 rel_pos1 = pointInA - body1.getCenterOfMassPosition();
		// btVector3 rel_pos2 = pointInB - body2.getCenterOfMassPosition();
		btVector3 rel_pos1 = anchorPos - body1.getCenterOfMassPosition();
		btVector3 rel_pos2 = anchorPos - body2.getCenterOfMassPosition();

		btVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
		btVector3 vel2 = body2.getVelocityInLocalPoint(rel_pos2);
		btVector3 vel = vel1 - vel2;

		btScalar rel_vel = axis_normal_on_a.dot(vel);

		/// apply displacement correction

		//positional error (zeroth order error)
		btScalar depth = -(pointInA - pointInB).dot(axis_normal_on_a);
		btScalar lo = btScalar(-BT_LARGE_FLOAT);
		btScalar hi = btScalar(BT_LARGE_FLOAT);

		btScalar minLimit = m_lowerLimit[limit_index];
		btScalar maxLimit = m_upperLimit[limit_index];

		//handle the limits
		if (minLimit < maxLimit)
		{
			{
				if (depth > maxLimit)
				{
					depth -= maxLimit;
					lo = btScalar(0.);
				}
				else
				{
					if (depth < minLimit)
					{
						depth -= minLimit;
						hi = btScalar(0.);
					}
					else
					{
						return 0.0f;
					}
				}
			}
		}

		btScalar normalImpulse = m_limitSoftness * (m_restitution * depth / timeStep - m_damping * rel_vel) * jacDiagABInv;

		btScalar oldNormalImpulse = m_accumulatedImpulse[limit_index];
		btScalar sum = oldNormalImpulse + normalImpulse;
		m_accumulatedImpulse[limit_index] = sum > hi ? btScalar(0.) : sum < lo ? btScalar(0.)
																			   : sum;
		normalImpulse = m_accumulatedImpulse[limit_index] - oldNormalImpulse;

		btVector3 impulse_vector = axis_normal_on_a * normalImpulse;
		body1.applyImpulse(impulse_vector, rel_pos1);
		body2.applyImpulse(-impulse_vector, rel_pos2);

		return normalImpulse;
	}
};

enum bt6DofFlags
{
	BT_6DOF_FLAGS_CFM_NORM = 1,
	BT_6DOF_FLAGS_CFM_STOP = 2,
	BT_6DOF_FLAGS_ERP_STOP = 4
};
#define BT_6DOF_FLAGS_AXIS_SHIFT 3  // bits per axis

/// btGeneric6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/*!
btGeneric6DofConstraint can leave any of the 6 degree of freedom 'free' or 'locked'.
currently this limit supports rotational motors<br>
<ul>
<li> For Linear limits, use btGeneric6DofConstraint.setLinearUpperLimit, btGeneric6DofConstraint.setLinearLowerLimit. You can set the parameters with the btTranslationalLimitMotor structure accsesible through the btGeneric6DofConstraint.getTranslationalLimitMotor method.
At this moment translational motors are not supported. May be in the future. </li>

<li> For Angular limits, use the btRotationalLimitMotor structure for configuring the limit.
This is accessible through btGeneric6DofConstraint.getLimitMotor method,
This brings support for limit parameters and motors. </li>

<li> Angulars limits have these possible ranges:
<table border=1 >
<tr>
	<td><b>AXIS</b></td>
	<td><b>MIN ANGLE</b></td>
	<td><b>MAX ANGLE</b></td>
</tr><tr>
	<td>X</td>
	<td>-PI</td>
	<td>PI</td>
</tr><tr>
	<td>Y</td>
	<td>-PI/2</td>
	<td>PI/2</td>
</tr><tr>
	<td>Z</td>
	<td>-PI</td>
	<td>PI</td>
</tr>
</table>
</li>
</ul>

*/
ATTRIBUTE_ALIGNED16(class)
btGeneric6DofConstraint : public btTypedConstraint
{
protected:
	// relative_frames
	// {
	btTransform m_frameInA;  // the constraint space w.r.t body A
	btTransform m_frameInB;  //!< the constraint space w.r.t body B
	//!@}

	// Jacobians
	// {
	btJacobianEntry m_jacLinear[3];  // 3 orthogonal linear constraints
	btJacobianEntry m_jacAng[3];     // 3 orthogonal angular constraints
	// }

	//! Linear_Limit_parameters
	//!@{
	btTranslationalLimitMotor m_linearLimits;
	// }

	//! hinge_parameters
	//!@{
	btRotationalLimitMotor m_angularLimits[3];
	// }

protected:
	//! temporal variables
	//!@{
	btScalar m_timeStep;
	btTransform m_calculatedTransformA;
	btTransform m_calculatedTransformB;
	btVector3 m_calculatedAxisAngleDiff;
	btVector3 m_calculatedAxis[3];
	btVector3 m_calculatedLinearDiff;
	btScalar m_factA;
	btScalar m_factB;
	bool m_hasStaticBody;

	btVector3 m_AnchorPos;  // point betwen pivots of bodies A and B to solve linear axes

	bool m_useLinearReferenceFrameA;
	bool m_useOffsetForConstraintFrame;

	int m_flags;

	//!@}

	btGeneric6DofConstraint& operator=(btGeneric6DofConstraint& other)
	{
		btAssert(0);
		(void)other;
		return *this;
	}

	int setAngularLimits(btConstraintInfo2 * info, int row_offset, const btTransform& transA, const btTransform& transB, const btVector3& linVelA, const btVector3& linVelB, const btVector3& angVelA, const btVector3& angVelB)
	{
		btGeneric6DofConstraint* d6constraint = this;
		int row = row_offset;
		//solve angular limits
		for (int i = 0; i < 3; i++)
		{
			if (d6constraint->getRotationalLimitMotor(i)->needApplyTorques())
			{
				btVector3 axis = d6constraint->getAxis(i);
				int flags = m_flags >> ((i + 3) * BT_6DOF_FLAGS_AXIS_SHIFT);
				if (!(flags & BT_6DOF_FLAGS_CFM_NORM))
				{
					m_angularLimits[i].m_normalCFM = info->cfm[0];
				}
				if (!(flags & BT_6DOF_FLAGS_CFM_STOP))
				{
					m_angularLimits[i].m_stopCFM = info->cfm[0];
				}
				if (!(flags & BT_6DOF_FLAGS_ERP_STOP))
				{
					m_angularLimits[i].m_stopERP = info->erp;
				}
				row += get_limit_motor_info2(d6constraint->getRotationalLimitMotor(i),
											 transA, transB, linVelA, linVelB, angVelA, angVelB, info, row, axis, 1);
			}
		}

		return row;
	}

	int setLinearLimits(btConstraintInfo2 * info, int row, const btTransform& transA, const btTransform& transB, const btVector3& linVelA, const btVector3& linVelB, const btVector3& angVelA, const btVector3& angVelB)
	{
		//	int row = 0;
		//solve linear limits
		btRotationalLimitMotor limot;
		for (int i = 0; i < 3; i++)
		{
			if (m_linearLimits.needApplyForce(i))
			{  // re-use rotational motor code
				limot.m_bounce = btScalar(0.f);
				limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
				limot.m_currentPosition = m_linearLimits.m_currentLinearDiff[i];
				limot.m_currentLimitError = m_linearLimits.m_currentLimitError[i];
				limot.m_damping = m_linearLimits.m_damping;
				limot.m_enableMotor = m_linearLimits.m_enableMotor[i];
				limot.m_hiLimit = m_linearLimits.m_upperLimit[i];
				limot.m_limitSoftness = m_linearLimits.m_limitSoftness;
				limot.m_loLimit = m_linearLimits.m_lowerLimit[i];
				limot.m_maxLimitForce = btScalar(0.f);
				limot.m_maxMotorForce = m_linearLimits.m_maxMotorForce[i];
				limot.m_targetVelocity = m_linearLimits.m_targetVelocity[i];
				btVector3 axis = m_calculatedTransformA.getBasis().getColumn(i);
				int flags = m_flags >> (i * BT_6DOF_FLAGS_AXIS_SHIFT);
				limot.m_normalCFM = (flags & BT_6DOF_FLAGS_CFM_NORM) ? m_linearLimits.m_normalCFM[i] : info->cfm[0];
				limot.m_stopCFM = (flags & BT_6DOF_FLAGS_CFM_STOP) ? m_linearLimits.m_stopCFM[i] : info->cfm[0];
				limot.m_stopERP = (flags & BT_6DOF_FLAGS_ERP_STOP) ? m_linearLimits.m_stopERP[i] : info->erp;
				if (m_useOffsetForConstraintFrame)
				{
					int indx1 = (i + 1) % 3;
					int indx2 = (i + 2) % 3;
					int rotAllowed = 1;  // rotations around orthos to current axis
					if (m_angularLimits[indx1].m_currentLimit && m_angularLimits[indx2].m_currentLimit)
					{
						rotAllowed = 0;
					}
					row += get_limit_motor_info2(&limot, transA, transB, linVelA, linVelB, angVelA, angVelB, info, row, axis, 0, rotAllowed);
				}
				else
				{
					row += get_limit_motor_info2(&limot, transA, transB, linVelA, linVelB, angVelA, angVelB, info, row, axis, 0);
				}
			}
		}
		return row;
	}

	void buildLinearJacobian(btJacobianEntry & jacLinear, const btVector3& normalWorld, const btVector3& pivotAInW, const btVector3& pivotBInW)
	{
		new (&jacLinear) btJacobianEntry(
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			pivotAInW - m_rbA.getCenterOfMassPosition(),
			pivotBInW - m_rbB.getCenterOfMassPosition(),
			normalWorld,
			m_rbA.getInvInertiaDiagLocal(),
			m_rbA.getInvMass(),
			m_rbB.getInvInertiaDiagLocal(),
			m_rbB.getInvMass());
	}

	void buildAngularJacobian(btJacobianEntry & jacAngular, const btVector3& jointAxisW)
	{
		new (&jacAngular) btJacobianEntry(jointAxisW,
										  m_rbA.getCenterOfMassTransform().getBasis().transpose(),
										  m_rbB.getCenterOfMassTransform().getBasis().transpose(),
										  m_rbA.getInvInertiaDiagLocal(),
										  m_rbB.getInvInertiaDiagLocal());
	}

	// tests linear limits
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

	//! calcs the euler angles between the two bodies.
	void calculateAngleInfo()
	{
		btMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().inverse() * m_calculatedTransformB.getBasis();
		matrixToEulerXYZ(relative_frame, m_calculatedAxisAngleDiff);
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
		btVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
		btVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);

		m_calculatedAxis[1] = axis2.cross(axis0);
		m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
		m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);

		m_calculatedAxis[0].normalize();
		m_calculatedAxis[1].normalize();
		m_calculatedAxis[2].normalize();
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	///for backwards compatibility during the transition to 'getInfo/getInfo2'
	bool m_useSolveConstraintObsolete;

	btGeneric6DofConstraint(btRigidBody & rbA, btRigidBody & rbB, const btTransform& frameInA, const btTransform& frameInB, bool useLinearReferenceFrameA)
		: btTypedConstraint(D6_CONSTRAINT_TYPE, rbA, rbB), m_frameInA(frameInA), m_frameInB(frameInB), m_useLinearReferenceFrameA(useLinearReferenceFrameA), m_useOffsetForConstraintFrame(D6_USE_FRAME_OFFSET), m_flags(0), m_useSolveConstraintObsolete(D6_USE_OBSOLETE_METHOD)
	{
		calculateTransforms();
	}
	btGeneric6DofConstraint(btRigidBody & rbB, const btTransform& frameInB, bool useLinearReferenceFrameB)
		: btTypedConstraint(D6_CONSTRAINT_TYPE, getFixedBody(), rbB),
		  m_frameInB(frameInB),
		  m_useLinearReferenceFrameA(useLinearReferenceFrameB),
		  m_useOffsetForConstraintFrame(D6_USE_FRAME_OFFSET),
		  m_flags(0),
		  m_useSolveConstraintObsolete(false)
	{
		///not providing rigidbody A means implicitly using worldspace for body A
		m_frameInA = rbB.getCenterOfMassTransform() * m_frameInB;
		calculateTransforms();
	}

	//! Calcs global transform of the offsets
	/*!
	Calcs the global transform for the joint offset for body A an B, and also calcs the agle differences between the bodies.
	\sa btGeneric6DofConstraint.getCalculatedTransformA , btGeneric6DofConstraint.getCalculatedTransformB, btGeneric6DofConstraint.calculateAngleInfo
	*/
	void calculateTransforms(const btTransform& transA, const btTransform& transB)
	{
		m_calculatedTransformA = transA * m_frameInA;
		m_calculatedTransformB = transB * m_frameInB;
		calculateLinearInfo();
		calculateAngleInfo();
		if (m_useOffsetForConstraintFrame)
		{  //  get weight factors depending on masses
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
	}

	void calculateTransforms()
	{
		calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
	}

	//! Gets the global transform of the offset for body A
	/*!
    \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
    */
	const btTransform& getCalculatedTransformA() const
	{
		return m_calculatedTransformA;
	}

	//! Gets the global transform of the offset for body B
	/*!
    \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
    */
	const btTransform& getCalculatedTransformB() const
	{
		return m_calculatedTransformB;
	}

	const btTransform& getFrameOffsetA() const
	{
		return m_frameInA;
	}

	const btTransform& getFrameOffsetB() const
	{
		return m_frameInB;
	}

	btTransform& getFrameOffsetA()
	{
		return m_frameInA;
	}

	btTransform& getFrameOffsetB()
	{
		return m_frameInB;
	}

	//! performs Jacobian calculation, and also calculates angle differences and axis
	virtual void buildJacobian()
	{
		if (m_useSolveConstraintObsolete)
		{
			// Clear accumulated impulses for the next simulation step
			m_linearLimits.m_accumulatedImpulse.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
			int i;
			for (i = 0; i < 3; i++)
			{
				m_angularLimits[i].m_accumulatedImpulse = btScalar(0.);
			}
			//calculates transform
			calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());

			//  const btVector3& pivotAInW = m_calculatedTransformA.getOrigin();
			//  const btVector3& pivotBInW = m_calculatedTransformB.getOrigin();
			calcAnchorPos();
			btVector3 pivotAInW = m_AnchorPos;
			btVector3 pivotBInW = m_AnchorPos;

			// not used here
			//    btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition();
			//    btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();

			btVector3 normalWorld;
			//linear part
			for (i = 0; i < 3; i++)
			{
				if (m_linearLimits.isLimited(i))
				{
					if (m_useLinearReferenceFrameA)
						normalWorld = m_calculatedTransformA.getBasis().getColumn(i);
					else
						normalWorld = m_calculatedTransformB.getBasis().getColumn(i);

					buildLinearJacobian(
						m_jacLinear[i], normalWorld,
						pivotAInW, pivotBInW);
				}
			}

			// angular part
			for (i = 0; i < 3; i++)
			{
				//calculates error angle
				if (testAngularLimitMotor(i))
				{
					normalWorld = this->getAxis(i);
					// Create angular atom
					buildAngularJacobian(m_jacAng[i], normalWorld);
				}
			}
		}
	}

	virtual void getInfo1(btConstraintInfo1 * info)
	{
		if (m_useSolveConstraintObsolete)
		{
			info->m_numConstraintRows = 0;
			info->nub = 0;
		}
		else
		{
			//prepare constraint
			calculateTransforms(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
			info->m_numConstraintRows = 0;
			info->nub = 6;
			int i;
			//test linear limits
			for (i = 0; i < 3; i++)
			{
				if (m_linearLimits.needApplyForce(i))
				{
					info->m_numConstraintRows++;
					info->nub--;
				}
			}
			//test angular limits
			for (i = 0; i < 3; i++)
			{
				if (testAngularLimitMotor(i))
				{
					info->m_numConstraintRows++;
					info->nub--;
				}
			}
		}
	}

	void getInfo1NonVirtual(btConstraintInfo1 * info)
	{
		if (m_useSolveConstraintObsolete)
		{
			info->m_numConstraintRows = 0;
			info->nub = 0;
		}
		else
		{
			//pre-allocate all 6
			info->m_numConstraintRows = 6;
			info->nub = 0;
		}
	}

	virtual void getInfo2(btConstraintInfo2 * info)
	{
		btAssert(!m_useSolveConstraintObsolete);

		const btTransform& transA = m_rbA.getCenterOfMassTransform();
		const btTransform& transB = m_rbB.getCenterOfMassTransform();
		const btVector3& linVelA = m_rbA.getLinearVelocity();
		const btVector3& linVelB = m_rbB.getLinearVelocity();
		const btVector3& angVelA = m_rbA.getAngularVelocity();
		const btVector3& angVelB = m_rbB.getAngularVelocity();

		if (m_useOffsetForConstraintFrame)
		{  // for stability better to solve angular limits first
			int row = setAngularLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
			setLinearLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
		}
		else
		{  // leave old version for compatibility
			int row = setLinearLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
			setAngularLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
		}
	}

	void getInfo2NonVirtual(btConstraintInfo2 * info, const btTransform& transA, const btTransform& transB, const btVector3& linVelA, const btVector3& linVelB, const btVector3& angVelA, const btVector3& angVelB)
	{
		btAssert(!m_useSolveConstraintObsolete);
		//prepare constraint
		calculateTransforms(transA, transB);

		int i;
		for (i = 0; i < 3; i++)
		{
			testAngularLimitMotor(i);
		}

		if (m_useOffsetForConstraintFrame)
		{  // for stability better to solve angular limits first
			int row = setAngularLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
			setLinearLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
		}
		else
		{  // leave old version for compatibility
			int row = setLinearLimits(info, 0, transA, transB, linVelA, linVelB, angVelA, angVelB);
			setAngularLimits(info, row, transA, transB, linVelA, linVelB, angVelA, angVelB);
		}
	}

	void updateRHS(btScalar timeStep)
	{
		(void)timeStep;
	}

	//! Get the rotation axis in global coordinates
	/*!
	\pre btGeneric6DofConstraint.buildJacobian must be called previously.
	*/
	btVector3 getAxis(int axis_index) const
	{
		return m_calculatedAxis[axis_index];
	}

	//! Get the relative Euler angle
	/*!
	\pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
	*/
	btScalar getAngle(int axis_index) const
	{
		return m_calculatedAxisAngleDiff[axis_index];
	}

	//! Get the relative position of the constraint pivot
	/*!
	\pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
	*/
	btScalar getRelativePivotPosition(int axisIndex) const
	{
		return m_calculatedLinearDiff[axisIndex];
	}

	void setFrames(const btTransform& frameA, const btTransform& frameB)
	{
		m_frameInA = frameA;
		m_frameInB = frameB;
		buildJacobian();
		calculateTransforms();
	}

	//! Test angular limit.
	/*!
	Calculates angular correction and returns true if limit needs to be corrected.
	\pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
	*/
	bool testAngularLimitMotor(int axis_index)
	{
		btScalar angle = m_calculatedAxisAngleDiff[axis_index];
		angle = btAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit);
		m_angularLimits[axis_index].m_currentPosition = angle;
		//test limits
		m_angularLimits[axis_index].testLimitValue(angle);
		return m_angularLimits[axis_index].needApplyTorques();
	}

	void setLinearLowerLimit(const btVector3& linearLower)
	{
		m_linearLimits.m_lowerLimit = linearLower;
	}

	void getLinearLowerLimit(btVector3 & linearLower) const
	{
		linearLower = m_linearLimits.m_lowerLimit;
	}

	void setLinearUpperLimit(const btVector3& linearUpper)
	{
		m_linearLimits.m_upperLimit = linearUpper;
	}

	void getLinearUpperLimit(btVector3 & linearUpper) const
	{
		linearUpper = m_linearLimits.m_upperLimit;
	}

	void setAngularLowerLimit(const btVector3& angularLower)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[i].m_loLimit = btNormalizeAngle(angularLower[i]);
	}

	void getAngularLowerLimit(btVector3 & angularLower) const
	{
		for (int i = 0; i < 3; i++)
			angularLower[i] = m_angularLimits[i].m_loLimit;
	}

	void setAngularUpperLimit(const btVector3& angularUpper)
	{
		for (int i = 0; i < 3; i++)
			m_angularLimits[i].m_hiLimit = btNormalizeAngle(angularUpper[i]);
	}

	void getAngularUpperLimit(btVector3 & angularUpper) const
	{
		for (int i = 0; i < 3; i++)
			angularUpper[i] = m_angularLimits[i].m_hiLimit;
	}

	//! Retrieves the angular limit informacion
	btRotationalLimitMotor* getRotationalLimitMotor(int index)
	{
		return &m_angularLimits[index];
	}

	//! Retrieves the  limit informacion
	btTranslationalLimitMotor* getTranslationalLimitMotor()
	{
		return &m_linearLimits;
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

	//! Test limit
	/*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 are linear, next 3 are angular
    */
	bool isLimited(int limitIndex) const
	{
		if (limitIndex < 3)
		{
			return m_linearLimits.isLimited(limitIndex);
		}
		return m_angularLimits[limitIndex - 3].isLimited();
	}

	virtual void calcAnchorPos(void) // overridable
	{
		btScalar imA = m_rbA.getInvMass();
		btScalar imB = m_rbB.getInvMass();
		btScalar weight;
		if (imB == btScalar(0.0))
		{
			weight = btScalar(1.0);
		}
		else
		{
			weight = imA / (imA + imB);
		}
		const btVector3& pA = m_calculatedTransformA.getOrigin();
		const btVector3& pB = m_calculatedTransformB.getOrigin();
		m_AnchorPos = pA * weight + pB * (btScalar(1.0) - weight);
		return;
	}

	int get_limit_motor_info2(btRotationalLimitMotor * limot,
							  const btTransform& transA, const btTransform& transB, const btVector3& linVelA, const btVector3& linVelB, const btVector3& angVelA, const btVector3& angVelB,
							  btConstraintInfo2* info, int row, btVector3& ax1, int rotational, int rotAllowed = false)
	{
		int srow = row * info->rowskip;
		bool powered = limot->m_enableMotor;
		int limit = limot->m_currentLimit;
		if (powered || limit)
		{  // if the joint is powered, or has joint limits, add in the extra row
			btScalar* J1 = rotational ? info->m_J1angularAxis : info->m_J1linearAxis;
			btScalar* J2 = rotational ? info->m_J2angularAxis : info->m_J2linearAxis;
			J1[srow + 0] = ax1[0];
			J1[srow + 1] = ax1[1];
			J1[srow + 2] = ax1[2];

			J2[srow + 0] = -ax1[0];
			J2[srow + 1] = -ax1[1];
			J2[srow + 2] = -ax1[2];

			if ((!rotational))
			{
				if (m_useOffsetForConstraintFrame)
				{
					btVector3 tmpA, tmpB, relA, relB;
					// get vector from bodyB to frameB in WCS
					relB = m_calculatedTransformB.getOrigin() - transB.getOrigin();
					// get its projection to constraint axis
					btVector3 projB = ax1 * relB.dot(ax1);
					// get vector directed from bodyB to constraint axis (and orthogonal to it)
					btVector3 orthoB = relB - projB;
					// same for bodyA
					relA = m_calculatedTransformA.getOrigin() - transA.getOrigin();
					btVector3 projA = ax1 * relA.dot(ax1);
					btVector3 orthoA = relA - projA;
					// get desired offset between frames A and B along constraint axis
					btScalar desiredOffs = limot->m_currentPosition - limot->m_currentLimitError;
					// desired vector from projection of center of bodyA to projection of center of bodyB to constraint axis
					btVector3 totalDist = projA + ax1 * desiredOffs - projB;
					// get offset vectors relA and relB
					relA = orthoA + totalDist * m_factA;
					relB = orthoB - totalDist * m_factB;
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
				else
				{
					btVector3 ltd;  // Linear Torque Decoupling vector
					btVector3 c = m_calculatedTransformB.getOrigin() - transA.getOrigin();
					ltd = c.cross(ax1);
					info->m_J1angularAxis[srow + 0] = ltd[0];
					info->m_J1angularAxis[srow + 1] = ltd[1];
					info->m_J1angularAxis[srow + 2] = ltd[2];

					c = m_calculatedTransformB.getOrigin() - transB.getOrigin();
					ltd = -c.cross(ax1);
					info->m_J2angularAxis[srow + 0] = ltd[0];
					info->m_J2angularAxis[srow + 1] = ltd[1];
					info->m_J2angularAxis[srow + 2] = ltd[2];
				}
			}
			// if we're limited low and high simultaneously, the joint motor is
			// ineffective
			if (limit && (limot->m_loLimit == limot->m_hiLimit)) powered = false;
			info->m_constraintError[srow] = btScalar(0.f);
			if (powered)
			{
				info->cfm[srow] = limot->m_normalCFM;
				if (!limit)
				{
					btScalar tag_vel = rotational ? limot->m_targetVelocity : -limot->m_targetVelocity;

					btScalar mot_fact = getMotorFactor(limot->m_currentPosition,
													   limot->m_loLimit,
													   limot->m_hiLimit,
													   tag_vel,
													   info->fps * limot->m_stopERP);
					info->m_constraintError[srow] += mot_fact * limot->m_targetVelocity;
					info->m_lowerLimit[srow] = -limot->m_maxMotorForce / info->fps;
					info->m_upperLimit[srow] = limot->m_maxMotorForce / info->fps;
				}
			}
			if (limit)
			{
				btScalar k = info->fps * limot->m_stopERP;
				if (!rotational)
				{
					info->m_constraintError[srow] += k * limot->m_currentLimitError;
				}
				else
				{
					info->m_constraintError[srow] += -k * limot->m_currentLimitError;
				}
				info->cfm[srow] = limot->m_stopCFM;
				if (limot->m_loLimit == limot->m_hiLimit)
				{  // limited low and high simultaneously
					info->m_lowerLimit[srow] = -SIMD_INFINITY;
					info->m_upperLimit[srow] = SIMD_INFINITY;
				}
				else
				{
					if (limit == 1)
					{
						info->m_lowerLimit[srow] = 0;
						info->m_upperLimit[srow] = SIMD_INFINITY;
					}
					else
					{
						info->m_lowerLimit[srow] = -SIMD_INFINITY;
						info->m_upperLimit[srow] = 0;
					}
					// deal with bounce
					if (limot->m_bounce > 0)
					{
						// calculate joint velocity
						btScalar vel;
						if (rotational)
						{
							vel = angVelA.dot(ax1);
							//make sure that if no body -> angVelB == zero vec
							//                        if (body1)
							vel -= angVelB.dot(ax1);
						}
						else
						{
							vel = linVelA.dot(ax1);
							//make sure that if no body -> angVelB == zero vec
							//                        if (body1)
							vel -= linVelB.dot(ax1);
						}
						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if (limit == 1)
						{
							if (vel < 0)
							{
								btScalar newc = -limot->m_bounce * vel;
								if (newc > info->m_constraintError[srow])
									info->m_constraintError[srow] = newc;
							}
						}
						else
						{
							if (vel > 0)
							{
								btScalar newc = -limot->m_bounce * vel;
								if (newc < info->m_constraintError[srow])
									info->m_constraintError[srow] = newc;
							}
						}
					}
				}
			}
			return 1;
		}
		else
			return 0;
	}

	// access for UseFrameOffset
	bool getUseFrameOffset() const { return m_useOffsetForConstraintFrame; }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }

	bool getUseLinearReferenceFrameA() const { return m_useLinearReferenceFrameA; }
	void setUseLinearReferenceFrameA(bool linearReferenceFrameA) { m_useLinearReferenceFrameA = linearReferenceFrameA; }

	// override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
	// If no axis is provided, it uses the default axis for this constraint.
	virtual void setParam(int num, btScalar value, int axis = -1)
	{
		if ((axis >= 0) && (axis < 3))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					m_linearLimits.m_stopERP[axis] = value;
					m_flags |= BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
					break;
				case BT_CONSTRAINT_STOP_CFM:
					m_linearLimits.m_stopCFM[axis] = value;
					m_flags |= BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
					break;
				case BT_CONSTRAINT_CFM:
					m_linearLimits.m_normalCFM[axis] = value;
					m_flags |= BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
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
					m_flags |= BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
					break;
				case BT_CONSTRAINT_STOP_CFM:
					m_angularLimits[axis - 3].m_stopCFM = value;
					m_flags |= BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
					break;
				case BT_CONSTRAINT_CFM:
					m_angularLimits[axis - 3].m_normalCFM = value;
					m_flags |= BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
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
	// return the local value of parameter
	virtual btScalar getParam(int num, int axis = -1) const
	{
		btScalar retVal = 0;
		if ((axis >= 0) && (axis < 3))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
					retVal = m_linearLimits.m_stopERP[axis];
					break;
				case BT_CONSTRAINT_STOP_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
					retVal = m_linearLimits.m_stopCFM[axis];
					break;
				case BT_CONSTRAINT_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
					retVal = m_linearLimits.m_normalCFM[axis];
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
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
					retVal = m_angularLimits[axis - 3].m_stopERP;
					break;
				case BT_CONSTRAINT_STOP_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
					retVal = m_angularLimits[axis - 3].m_stopCFM;
					break;
				case BT_CONSTRAINT_CFM:
					btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
					retVal = m_angularLimits[axis - 3].m_normalCFM;
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
	virtual int getFlags() const
	{
		return m_flags;
	}

	virtual int calculateSerializeBufferSize() const;

	// fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

struct btGeneric6DofConstraintData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformFloatData m_rbAFrame;  // constraint axii. Assumes z is hinge axis.
	btTransformFloatData m_rbBFrame;

	btVector3FloatData m_linearUpperLimit;
	btVector3FloatData m_linearLowerLimit;

	btVector3FloatData m_angularUpperLimit;
	btVector3FloatData m_angularLowerLimit;

	int m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;
};

struct btGeneric6DofConstraintDoubleData2
{
	btTypedConstraintDoubleData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame;  // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;

	btVector3DoubleData m_linearUpperLimit;
	btVector3DoubleData m_linearLowerLimit;

	btVector3DoubleData m_angularUpperLimit;
	btVector3DoubleData m_angularLowerLimit;

	int m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;
};

SIMD_FORCE_INLINE int btGeneric6DofConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btGeneric6DofConstraintData2);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btGeneric6DofConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btGeneric6DofConstraintData2* dof = (btGeneric6DofConstraintData2*)dataBuffer;
	btTypedConstraint::serialize(&dof->m_typeConstraintData, serializer);

	m_frameInA.serialize(dof->m_rbAFrame);
	m_frameInB.serialize(dof->m_rbBFrame);

	int i;
	for (i = 0; i < 3; i++)
	{
		dof->m_angularLowerLimit.m_floats[i] = m_angularLimits[i].m_loLimit;
		dof->m_angularUpperLimit.m_floats[i] = m_angularLimits[i].m_hiLimit;
		dof->m_linearLowerLimit.m_floats[i] = m_linearLimits.m_lowerLimit[i];
		dof->m_linearUpperLimit.m_floats[i] = m_linearLimits.m_upperLimit[i];
	}

	dof->m_useLinearReferenceFrameA = m_useLinearReferenceFrameA ? 1 : 0;
	dof->m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame ? 1 : 0;

	return btGeneric6DofConstraintDataName;
}

#endif  //BT_GENERIC_6DOF_CONSTRAINT_H
