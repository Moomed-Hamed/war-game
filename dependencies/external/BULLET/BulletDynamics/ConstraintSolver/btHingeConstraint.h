// Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios //

#ifndef BT_HINGECONSTRAINT_H
#define BT_HINGECONSTRAINT_H

#include "btTypedConstraint.h"

#define _BT_USE_CENTER_LIMIT_ 1
#define HINGE_USE_OBSOLETE_SOLVER false
#define HINGE_USE_FRAME_OFFSET true

#define btHingeConstraintData btHingeConstraintFloatData
#define btHingeConstraintDataName "btHingeConstraintFloatData"

static btVector3 vHinge(0, 0, btScalar(1));

static inline btScalar btNormalizeAnglePositive(btScalar angle)
{
	return btFmod(btFmod(angle, btScalar(2.0 * SIMD_PI)) + btScalar(2.0 * SIMD_PI), btScalar(2.0 * SIMD_PI));
}

static btScalar btShortestAngularDistance(btScalar accAngle, btScalar curAngle)
{
	btScalar result = btNormalizeAngle(btNormalizeAnglePositive(btNormalizeAnglePositive(curAngle) -
																btNormalizeAnglePositive(accAngle)));
	return result;
}

static btScalar btShortestAngleUpdate(btScalar accAngle, btScalar curAngle)
{
	btScalar tol(0.3);
	btScalar result = btShortestAngularDistance(accAngle, curAngle);

	if (btFabs(result) > tol)
		return curAngle;
	else
		return accAngle + result;

	return curAngle;
}

enum btHingeFlags
{
	BT_HINGE_FLAGS_CFM_STOP = 1,
	BT_HINGE_FLAGS_ERP_STOP = 2,
	BT_HINGE_FLAGS_CFM_NORM = 4,
	BT_HINGE_FLAGS_ERP_NORM = 8
};

/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// axis defines the orientation of the hinge axis
ATTRIBUTE_ALIGNED16(class)
btHingeConstraint : public btTypedConstraint
{
	btJacobianEntry m_jac[3];     //3 orthogonal linear constraints
	btJacobianEntry m_jacAng[3];  //2 orthogonal angular constraints+ 1 for limit/motor

	btTransform m_rbAFrame;  // constraint axii. Assumes z is hinge axis.
	btTransform m_rbBFrame;

	btScalar m_motorTargetVelocity;
	btScalar m_maxMotorImpulse;

	btAngularLimit m_limit;

	btScalar m_kHinge;

	btScalar m_accLimitImpulse;
	btScalar m_hingeAngle;
	btScalar m_referenceSign;

	bool m_angularOnly;
	bool m_enableAngularMotor;
	bool m_useSolveConstraintObsolete;
	bool m_useOffsetForConstraintFrame;
	bool m_useReferenceFrameA;

	btScalar m_accMotorImpulse;

	int m_flags;
	btScalar m_normalCFM;
	btScalar m_normalERP;
	btScalar m_stopCFM;
	btScalar m_stopERP;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btHingeConstraint(btRigidBody & rbA, btRigidBody & rbB, const btVector3& pivotInA, const btVector3& pivotInB, const btVector3& axisInA, const btVector3& axisInB, bool useReferenceFrameA = false)
		: btTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA, rbB),
#ifdef _BT_USE_CENTER_LIMIT_
		  m_limit(),
#endif
		  m_angularOnly(false),
		  m_enableAngularMotor(false),
		  m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
		  m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
		  m_useReferenceFrameA(useReferenceFrameA),
		  m_flags(0),
		  m_normalCFM(0),
		  m_normalERP(0),
		  m_stopCFM(0),
		  m_stopERP(0)
	{
		m_rbAFrame.getOrigin() = pivotInA;

		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		btVector3 rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(0);

		btVector3 rbAxisA2;
		btScalar projection = axisInA.dot(rbAxisA1);
		if (projection >= 1.0f - SIMD_EPSILON)
		{
			rbAxisA1 = -rbA.getCenterOfMassTransform().getBasis().getColumn(2);
			rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);
		}
		else if (projection <= -1.0f + SIMD_EPSILON)
		{
			rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(2);
			rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);
		}
		else
		{
			rbAxisA2 = axisInA.cross(rbAxisA1);
			rbAxisA1 = rbAxisA2.cross(axisInA);
		}

		m_rbAFrame.getBasis().setValue(rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(),
									   rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(),
									   rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ());

		btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
		btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
		btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);

		m_rbBFrame.getOrigin() = pivotInB;
		m_rbBFrame.getBasis().setValue(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(),
									   rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(),
									   rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ());

#ifndef _BT_USE_CENTER_LIMIT_
		//start with free
		m_lowerLimit = btScalar(1.0f);
		m_upperLimit = btScalar(-1.0f);
		m_biasFactor = 0.3f;
		m_relaxationFactor = 1.0f;
		m_limitSoftness = 0.9f;
		m_solveLimit = false;
#endif
		m_referenceSign = m_useReferenceFrameA ? btScalar(-1.f) : btScalar(1.f);
	}

	btHingeConstraint(btRigidBody & rbA, const btVector3& pivotInA, const btVector3& axisInA, bool useReferenceFrameA = false)
		: btTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA),
#ifdef _BT_USE_CENTER_LIMIT_
		  m_limit(),
#endif
		  m_angularOnly(false),
		  m_enableAngularMotor(false),
		  m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
		  m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
		  m_useReferenceFrameA(useReferenceFrameA),
		  m_flags(0),
		  m_normalCFM(0),
		  m_normalERP(0),
		  m_stopCFM(0),
		  m_stopERP(0)
	{
		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		// fixed axis in worldspace
		btVector3 rbAxisA1, rbAxisA2;
		btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);

		m_rbAFrame.getOrigin() = pivotInA;
		m_rbAFrame.getBasis().setValue(rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(),
									   rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(),
									   rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ());

		btVector3 axisInB = rbA.getCenterOfMassTransform().getBasis() * axisInA;

		btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
		btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
		btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);

		m_rbBFrame.getOrigin() = rbA.getCenterOfMassTransform()(pivotInA);
		m_rbBFrame.getBasis().setValue(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(),
									   rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(),
									   rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ());

#ifndef _BT_USE_CENTER_LIMIT_
		//start with free
		m_lowerLimit = btScalar(1.0f);
		m_upperLimit = btScalar(-1.0f);
		m_biasFactor = 0.3f;
		m_relaxationFactor = 1.0f;
		m_limitSoftness = 0.9f;
		m_solveLimit = false;
#endif
		m_referenceSign = m_useReferenceFrameA ? btScalar(-1.f) : btScalar(1.f);
	}

	btHingeConstraint(btRigidBody & rbA, btRigidBody & rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA = false)
		: btTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA, rbB), m_rbAFrame(rbAFrame), m_rbBFrame(rbBFrame),
#ifdef _BT_USE_CENTER_LIMIT_
		  m_limit(),
#endif
		  m_angularOnly(false),
		  m_enableAngularMotor(false),
		  m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
		  m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
		  m_useReferenceFrameA(useReferenceFrameA),
		  m_flags(0),
		  m_normalCFM(0),
		  m_normalERP(0),
		  m_stopCFM(0),
		  m_stopERP(0)
	{
#ifndef _BT_USE_CENTER_LIMIT_
		//start with free
		m_lowerLimit = btScalar(1.0f);
		m_upperLimit = btScalar(-1.0f);
		m_biasFactor = 0.3f;
		m_relaxationFactor = 1.0f;
		m_limitSoftness = 0.9f;
		m_solveLimit = false;
#endif
		m_referenceSign = m_useReferenceFrameA ? btScalar(-1.f) : btScalar(1.f);
	}

	btHingeConstraint(btRigidBody & rbA, const btTransform& rbAFrame, bool useReferenceFrameA = false)
		: btTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA), m_rbAFrame(rbAFrame), m_rbBFrame(rbAFrame),
#ifdef _BT_USE_CENTER_LIMIT_
		  m_limit(),
#endif
		  m_angularOnly(false),
		  m_enableAngularMotor(false),
		  m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
		  m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
		  m_useReferenceFrameA(useReferenceFrameA),
		  m_flags(0),
		  m_normalCFM(0),
		  m_normalERP(0),
		  m_stopCFM(0),
		  m_stopERP(0)
	{
		///not providing rigidbody B means implicitly using worldspace for body B

		m_rbBFrame.getOrigin() = m_rbA.getCenterOfMassTransform()(m_rbAFrame.getOrigin());
#ifndef _BT_USE_CENTER_LIMIT_
		//start with free
		m_lowerLimit = btScalar(1.0f);
		m_upperLimit = btScalar(-1.0f);
		m_biasFactor = 0.3f;
		m_relaxationFactor = 1.0f;
		m_limitSoftness = 0.9f;
		m_solveLimit = false;
#endif
		m_referenceSign = m_useReferenceFrameA ? btScalar(-1.f) : btScalar(1.f);
	}

	virtual void buildJacobian()
	{
		if (m_useSolveConstraintObsolete)
		{
			m_appliedImpulse = btScalar(0.);
			m_accMotorImpulse = btScalar(0.);

			if (!m_angularOnly)
			{
				btVector3 pivotAInW = m_rbA.getCenterOfMassTransform() * m_rbAFrame.getOrigin();
				btVector3 pivotBInW = m_rbB.getCenterOfMassTransform() * m_rbBFrame.getOrigin();
				btVector3 relPos = pivotBInW - pivotAInW;

				btVector3 normal[3];
				if (relPos.length2() > SIMD_EPSILON)
				{
					normal[0] = relPos.normalized();
				}
				else
				{
					normal[0].setValue(btScalar(1.0), 0, 0);
				}

				btPlaneSpace1(normal[0], normal[1], normal[2]);

				for (int i = 0; i < 3; i++)
				{
					new (&m_jac[i]) btJacobianEntry(
						m_rbA.getCenterOfMassTransform().getBasis().transpose(),
						m_rbB.getCenterOfMassTransform().getBasis().transpose(),
						pivotAInW - m_rbA.getCenterOfMassPosition(),
						pivotBInW - m_rbB.getCenterOfMassPosition(),
						normal[i],
						m_rbA.getInvInertiaDiagLocal(),
						m_rbA.getInvMass(),
						m_rbB.getInvInertiaDiagLocal(),
						m_rbB.getInvMass());
				}
			}

			//calculate two perpendicular jointAxis, orthogonal to hingeAxis
			//these two jointAxis require equal angular velocities for both bodies

			//this is unused for now, it's a todo
			btVector3 jointAxis0local;
			btVector3 jointAxis1local;

			btPlaneSpace1(m_rbAFrame.getBasis().getColumn(2), jointAxis0local, jointAxis1local);

			btVector3 jointAxis0 = getRigidBodyA().getCenterOfMassTransform().getBasis() * jointAxis0local;
			btVector3 jointAxis1 = getRigidBodyA().getCenterOfMassTransform().getBasis() * jointAxis1local;
			btVector3 hingeAxisWorld = getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);

			new (&m_jacAng[0]) btJacobianEntry(jointAxis0,
											   m_rbA.getCenterOfMassTransform().getBasis().transpose(),
											   m_rbB.getCenterOfMassTransform().getBasis().transpose(),
											   m_rbA.getInvInertiaDiagLocal(),
											   m_rbB.getInvInertiaDiagLocal());

			new (&m_jacAng[1]) btJacobianEntry(jointAxis1,
											   m_rbA.getCenterOfMassTransform().getBasis().transpose(),
											   m_rbB.getCenterOfMassTransform().getBasis().transpose(),
											   m_rbA.getInvInertiaDiagLocal(),
											   m_rbB.getInvInertiaDiagLocal());

			new (&m_jacAng[2]) btJacobianEntry(hingeAxisWorld,
											   m_rbA.getCenterOfMassTransform().getBasis().transpose(),
											   m_rbB.getCenterOfMassTransform().getBasis().transpose(),
											   m_rbA.getInvInertiaDiagLocal(),
											   m_rbB.getInvInertiaDiagLocal());

			// clear accumulator
			m_accLimitImpulse = btScalar(0.);

			// test angular limit
			testLimit(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());

			//Compute K = J*W*J' for hinge axis
			btVector3 axisA = getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);
			m_kHinge = 1.0f / (getRigidBodyA().computeAngularImpulseDenominator(axisA) +
							   getRigidBodyB().computeAngularImpulseDenominator(axisA));
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
			info->m_numConstraintRows = 5;  // Fixed 3 linear + 2 angular
			info->nub = 1;
			//always add the row, to avoid computation (data is not available yet)
			//prepare constraint
			testLimit(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
			if (getSolveLimit() || getEnableAngularMotor())
			{
				info->m_numConstraintRows++;  // limit 3rd anguar as well
				info->nub--;
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
			//always add the 'limit' row, to avoid computation (data is not available yet)
			info->m_numConstraintRows = 6;  // Fixed 3 linear + 2 angular
			info->nub = 0;
		}
	}

	virtual void getInfo2(btConstraintInfo2 * info)
	{
		if (m_useOffsetForConstraintFrame)
			getInfo2InternalUsingFrameOffset(info, m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform(), m_rbA.getAngularVelocity(), m_rbB.getAngularVelocity());
		else
			getInfo2Internal(info, m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform(), m_rbA.getAngularVelocity(), m_rbB.getAngularVelocity());
	}

	void getInfo2NonVirtual(btConstraintInfo2 * info, const btTransform& transA, const btTransform& transB, const btVector3& angVelA, const btVector3& angVelB)
	{
		///the regular (virtual) implementation getInfo2 already performs 'testLimit' during getInfo1, so we need to do it now
		testLimit(transA, transB);

		getInfo2Internal(info, transA, transB, angVelA, angVelB);
	}

	void getInfo2Internal(btConstraintInfo2 * info, const btTransform& transA, const btTransform& transB, const btVector3& angVelA, const btVector3& angVelB)
	{
		btAssert(!m_useSolveConstraintObsolete);
		int i, skip = info->rowskip;
		// transforms in world space
		btTransform trA = transA * m_rbAFrame;
		btTransform trB = transB * m_rbBFrame;
		// pivot point
		btVector3 pivotAInW = trA.getOrigin();
		btVector3 pivotBInW = trB.getOrigin();

		if (!m_angularOnly)
		{
			info->m_J1linearAxis[0] = 1;
			info->m_J1linearAxis[skip + 1] = 1;
			info->m_J1linearAxis[2 * skip + 2] = 1;

			info->m_J2linearAxis[0] = -1;
			info->m_J2linearAxis[skip + 1] = -1;
			info->m_J2linearAxis[2 * skip + 2] = -1;
		}

		btVector3 a1 = pivotAInW - transA.getOrigin();
		{
			btVector3* angular0 = (btVector3*)(info->m_J1angularAxis);
			btVector3* angular1 = (btVector3*)(info->m_J1angularAxis + skip);
			btVector3* angular2 = (btVector3*)(info->m_J1angularAxis + 2 * skip);
			btVector3 a1neg = -a1;
			a1neg.getSkewSymmetricMatrix(angular0, angular1, angular2);
		}
		btVector3 a2 = pivotBInW - transB.getOrigin();
		{
			btVector3* angular0 = (btVector3*)(info->m_J2angularAxis);
			btVector3* angular1 = (btVector3*)(info->m_J2angularAxis + skip);
			btVector3* angular2 = (btVector3*)(info->m_J2angularAxis + 2 * skip);
			a2.getSkewSymmetricMatrix(angular0, angular1, angular2);
		}
		// linear RHS
		btScalar normalErp = (m_flags & BT_HINGE_FLAGS_ERP_NORM) ? m_normalERP : info->erp;

		btScalar k = info->fps * normalErp;
		if (!m_angularOnly)
		{
			for (i = 0; i < 3; i++)
			{
				info->m_constraintError[i * skip] = k * (pivotBInW[i] - pivotAInW[i]);
			}
		}
		// make rotations around X and Y equal
		// the hinge axis should be the only unconstrained
		// rotational axis, the angular velocity of the two bodies perpendicular to
		// the hinge axis should be equal. thus the constraint equations are
		//    p*w1 - p*w2 = 0
		//    q*w1 - q*w2 = 0
		// where p and q are unit vectors normal to the hinge axis, and w1 and w2
		// are the angular velocity vectors of the two bodies.
		// get hinge axis (Z)
		btVector3 ax1 = trA.getBasis().getColumn(2);
		// get 2 orthos to hinge axis (X, Y)
		btVector3 p = trA.getBasis().getColumn(0);
		btVector3 q = trA.getBasis().getColumn(1);
		// set the two hinge angular rows
		int s3 = 3 * info->rowskip;
		int s4 = 4 * info->rowskip;

		info->m_J1angularAxis[s3 + 0] = p[0];
		info->m_J1angularAxis[s3 + 1] = p[1];
		info->m_J1angularAxis[s3 + 2] = p[2];
		info->m_J1angularAxis[s4 + 0] = q[0];
		info->m_J1angularAxis[s4 + 1] = q[1];
		info->m_J1angularAxis[s4 + 2] = q[2];

		info->m_J2angularAxis[s3 + 0] = -p[0];
		info->m_J2angularAxis[s3 + 1] = -p[1];
		info->m_J2angularAxis[s3 + 2] = -p[2];
		info->m_J2angularAxis[s4 + 0] = -q[0];
		info->m_J2angularAxis[s4 + 1] = -q[1];
		info->m_J2angularAxis[s4 + 2] = -q[2];
		// compute the right hand side of the constraint equation. set relative
		// body velocities along p and q to bring the hinge back into alignment.
		// if ax1,ax2 are the unit length hinge axes as computed from body1 and
		// body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
		// if `theta' is the angle between ax1 and ax2, we need an angular velocity
		// along u to cover angle erp*theta in one step :
		//   |angular_velocity| = angle/time = erp*theta / stepsize
		//                      = (erp*fps) * theta
		//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
		//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
		// ...as ax1 and ax2 are unit length. if theta is smallish,
		// theta ~= sin(theta), so
		//    angular_velocity  = (erp*fps) * (ax1 x ax2)
		// ax1 x ax2 is in the plane space of ax1, so we project the angular
		// velocity to p and q to find the right hand side.
		btVector3 ax2 = trB.getBasis().getColumn(2);
		btVector3 u = ax1.cross(ax2);
		info->m_constraintError[s3] = k * u.dot(p);
		info->m_constraintError[s4] = k * u.dot(q);
		// check angular limits
		int nrow = 4;  // last filled row
		int srow;
		btScalar limit_err = btScalar(0.0);
		int limit = 0;
		if (getSolveLimit())
		{
#ifdef _BT_USE_CENTER_LIMIT_
			limit_err = m_limit.getCorrection() * m_referenceSign;
#else
			limit_err = m_correction * m_referenceSign;
#endif
			limit = (limit_err > btScalar(0.0)) ? 1 : 2;
		}
		// if the hinge has joint limits or motor, add in the extra row
		bool powered = getEnableAngularMotor();
		if (limit || powered)
		{
			nrow++;
			srow = nrow * info->rowskip;
			info->m_J1angularAxis[srow + 0] = ax1[0];
			info->m_J1angularAxis[srow + 1] = ax1[1];
			info->m_J1angularAxis[srow + 2] = ax1[2];

			info->m_J2angularAxis[srow + 0] = -ax1[0];
			info->m_J2angularAxis[srow + 1] = -ax1[1];
			info->m_J2angularAxis[srow + 2] = -ax1[2];

			btScalar lostop = getLowerLimit();
			btScalar histop = getUpperLimit();
			if (limit && (lostop == histop))
			{  // the joint motor is ineffective
				powered = false;
			}
			info->m_constraintError[srow] = btScalar(0.0f);
			btScalar currERP = (m_flags & BT_HINGE_FLAGS_ERP_STOP) ? m_stopERP : normalErp;
			if (powered)
			{
				if (m_flags & BT_HINGE_FLAGS_CFM_NORM)
				{
					info->cfm[srow] = m_normalCFM;
				}
				btScalar mot_fact = getMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info->fps * currERP);
				info->m_constraintError[srow] += mot_fact * m_motorTargetVelocity * m_referenceSign;
				info->m_lowerLimit[srow] = -m_maxMotorImpulse;
				info->m_upperLimit[srow] = m_maxMotorImpulse;
			}
			if (limit)
			{
				k = info->fps * currERP;
				info->m_constraintError[srow] += k * limit_err;
				if (m_flags & BT_HINGE_FLAGS_CFM_STOP)
				{
					info->cfm[srow] = m_stopCFM;
				}
				if (lostop == histop)
				{
					// limited low and high simultaneously
					info->m_lowerLimit[srow] = -SIMD_INFINITY;
					info->m_upperLimit[srow] = SIMD_INFINITY;
				}
				else if (limit == 1)
				{  // low limit
					info->m_lowerLimit[srow] = 0;
					info->m_upperLimit[srow] = SIMD_INFINITY;
				}
				else
				{  // high limit
					info->m_lowerLimit[srow] = -SIMD_INFINITY;
					info->m_upperLimit[srow] = 0;
				}
				// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
#ifdef _BT_USE_CENTER_LIMIT_
				btScalar bounce = m_limit.getRelaxationFactor();
#else
				btScalar bounce = m_relaxationFactor;
#endif
				if (bounce > btScalar(0.0))
				{
					btScalar vel = angVelA.dot(ax1);
					vel -= angVelB.dot(ax1);
					// only apply bounce if the velocity is incoming, and if the
					// resulting c[] exceeds what we already have.
					if (limit == 1)
					{  // low limit
						if (vel < 0)
						{
							btScalar newc = -bounce * vel;
							if (newc > info->m_constraintError[srow])
							{
								info->m_constraintError[srow] = newc;
							}
						}
					}
					else
					{  // high limit - all those computations are reversed
						if (vel > 0)
						{
							btScalar newc = -bounce * vel;
							if (newc < info->m_constraintError[srow])
							{
								info->m_constraintError[srow] = newc;
							}
						}
					}
				}
#ifdef _BT_USE_CENTER_LIMIT_
				info->m_constraintError[srow] *= m_limit.getBiasFactor();
#else
				info->m_constraintError[srow] *= m_biasFactor;
#endif
			}  // if(limit)
		}      // if angular limit or powered
	}
	void getInfo2InternalUsingFrameOffset(btConstraintInfo2 * info, const btTransform& transA, const btTransform& transB, const btVector3& angVelA, const btVector3& angVelB)
	{
		btAssert(!m_useSolveConstraintObsolete);
		int i, s = info->rowskip;
		// transforms in world space
		btTransform trA = transA * m_rbAFrame;
		btTransform trB = transB * m_rbBFrame;
		// pivot point
//	btVector3 pivotAInW = trA.getOrigin();
//	btVector3 pivotBInW = trB.getOrigin();
#if 1
		// difference between frames in WCS
		btVector3 ofs = trB.getOrigin() - trA.getOrigin();
		// now get weight factors depending on masses
		btScalar miA = getRigidBodyA().getInvMass();
		btScalar miB = getRigidBodyB().getInvMass();
		bool hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
		btScalar miS = miA + miB;
		btScalar factA, factB;
		if (miS > btScalar(0.f))
		{
			factA = miB / miS;
		}
		else
		{
			factA = btScalar(0.5f);
		}
		factB = btScalar(1.0f) - factA;
		// get the desired direction of hinge axis
		// as weighted sum of Z-orthos of frameA and frameB in WCS
		btVector3 ax1A = trA.getBasis().getColumn(2);
		btVector3 ax1B = trB.getBasis().getColumn(2);
		btVector3 ax1 = ax1A * factA + ax1B * factB;
		if (ax1.length2() < SIMD_EPSILON)
		{
			factA = 0.f;
			factB = 1.f;
			ax1 = ax1A * factA + ax1B * factB;
		}
		ax1.normalize();
		// fill first 3 rows
		// we want: velA + wA x relA == velB + wB x relB
		btTransform bodyA_trans = transA;
		btTransform bodyB_trans = transB;
		int s0 = 0;
		int s1 = s;
		int s2 = s * 2;
		int nrow = 2;  // last filled row
		btVector3 tmpA, tmpB, relA, relB, p, q;
		// get vector from bodyB to frameB in WCS
		relB = trB.getOrigin() - bodyB_trans.getOrigin();
		// get its projection to hinge axis
		btVector3 projB = ax1 * relB.dot(ax1);
		// get vector directed from bodyB to hinge axis (and orthogonal to it)
		btVector3 orthoB = relB - projB;
		// same for bodyA
		relA = trA.getOrigin() - bodyA_trans.getOrigin();
		btVector3 projA = ax1 * relA.dot(ax1);
		btVector3 orthoA = relA - projA;
		btVector3 totalDist = projA - projB;
		// get offset vectors relA and relB
		relA = orthoA + totalDist * factA;
		relB = orthoB - totalDist * factB;
		// now choose average ortho to hinge axis
		p = orthoB * factA + orthoA * factB;
		btScalar len2 = p.length2();
		if (len2 > SIMD_EPSILON)
		{
			p /= btSqrt(len2);
		}
		else
		{
			p = trA.getBasis().getColumn(1);
		}
		// make one more ortho
		q = ax1.cross(p);
		// fill three rows
		tmpA = relA.cross(p);
		tmpB = relB.cross(p);
		for (i = 0; i < 3; i++) info->m_J1angularAxis[s0 + i] = tmpA[i];
		for (i = 0; i < 3; i++) info->m_J2angularAxis[s0 + i] = -tmpB[i];
		tmpA = relA.cross(q);
		tmpB = relB.cross(q);
		if (hasStaticBody && getSolveLimit())
		{  // to make constraint between static and dynamic objects more rigid
			// remove wA (or wB) from equation if angular limit is hit
			tmpB *= factB;
			tmpA *= factA;
		}
		for (i = 0; i < 3; i++) info->m_J1angularAxis[s1 + i] = tmpA[i];
		for (i = 0; i < 3; i++) info->m_J2angularAxis[s1 + i] = -tmpB[i];
		tmpA = relA.cross(ax1);
		tmpB = relB.cross(ax1);
		if (hasStaticBody)
		{  // to make constraint between static and dynamic objects more rigid
			// remove wA (or wB) from equation
			tmpB *= factB;
			tmpA *= factA;
		}
		for (i = 0; i < 3; i++) info->m_J1angularAxis[s2 + i] = tmpA[i];
		for (i = 0; i < 3; i++) info->m_J2angularAxis[s2 + i] = -tmpB[i];

		btScalar normalErp = (m_flags & BT_HINGE_FLAGS_ERP_NORM) ? m_normalERP : info->erp;
		btScalar k = info->fps * normalErp;

		if (!m_angularOnly)
		{
			for (i = 0; i < 3; i++) info->m_J1linearAxis[s0 + i] = p[i];
			for (i = 0; i < 3; i++) info->m_J1linearAxis[s1 + i] = q[i];
			for (i = 0; i < 3; i++) info->m_J1linearAxis[s2 + i] = ax1[i];

			for (i = 0; i < 3; i++) info->m_J2linearAxis[s0 + i] = -p[i];
			for (i = 0; i < 3; i++) info->m_J2linearAxis[s1 + i] = -q[i];
			for (i = 0; i < 3; i++) info->m_J2linearAxis[s2 + i] = -ax1[i];

			// compute three elements of right hand side

			btScalar rhs = k * p.dot(ofs);
			info->m_constraintError[s0] = rhs;
			rhs = k * q.dot(ofs);
			info->m_constraintError[s1] = rhs;
			rhs = k * ax1.dot(ofs);
			info->m_constraintError[s2] = rhs;
		}
		// the hinge axis should be the only unconstrained
		// rotational axis, the angular velocity of the two bodies perpendicular to
		// the hinge axis should be equal. thus the constraint equations are
		//    p*w1 - p*w2 = 0
		//    q*w1 - q*w2 = 0
		// where p and q are unit vectors normal to the hinge axis, and w1 and w2
		// are the angular velocity vectors of the two bodies.
		int s3 = 3 * s;
		int s4 = 4 * s;
		info->m_J1angularAxis[s3 + 0] = p[0];
		info->m_J1angularAxis[s3 + 1] = p[1];
		info->m_J1angularAxis[s3 + 2] = p[2];
		info->m_J1angularAxis[s4 + 0] = q[0];
		info->m_J1angularAxis[s4 + 1] = q[1];
		info->m_J1angularAxis[s4 + 2] = q[2];

		info->m_J2angularAxis[s3 + 0] = -p[0];
		info->m_J2angularAxis[s3 + 1] = -p[1];
		info->m_J2angularAxis[s3 + 2] = -p[2];
		info->m_J2angularAxis[s4 + 0] = -q[0];
		info->m_J2angularAxis[s4 + 1] = -q[1];
		info->m_J2angularAxis[s4 + 2] = -q[2];
		// compute the right hand side of the constraint equation. set relative
		// body velocities along p and q to bring the hinge back into alignment.
		// if ax1A,ax1B are the unit length hinge axes as computed from bodyA and
		// bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
		// if "theta" is the angle between ax1 and ax2, we need an angular velocity
		// along u to cover angle erp*theta in one step :
		//   |angular_velocity| = angle/time = erp*theta / stepsize
		//                      = (erp*fps) * theta
		//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
		//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
		// ...as ax1 and ax2 are unit length. if theta is smallish,
		// theta ~= sin(theta), so
		//    angular_velocity  = (erp*fps) * (ax1 x ax2)
		// ax1 x ax2 is in the plane space of ax1, so we project the angular
		// velocity to p and q to find the right hand side.
		k = info->fps * normalErp;  //??

		btVector3 u = ax1A.cross(ax1B);
		info->m_constraintError[s3] = k * u.dot(p);
		info->m_constraintError[s4] = k * u.dot(q);
#endif
		// check angular limits
		nrow = 4;  // last filled row
		int srow;
		btScalar limit_err = btScalar(0.0);
		int limit = 0;
		if (getSolveLimit())
		{
#ifdef _BT_USE_CENTER_LIMIT_
			limit_err = m_limit.getCorrection() * m_referenceSign;
#else
			limit_err = m_correction * m_referenceSign;
#endif
			limit = (limit_err > btScalar(0.0)) ? 1 : 2;
		}
		// if the hinge has joint limits or motor, add in the extra row
		bool powered = getEnableAngularMotor();
		if (limit || powered)
		{
			nrow++;
			srow = nrow * info->rowskip;
			info->m_J1angularAxis[srow + 0] = ax1[0];
			info->m_J1angularAxis[srow + 1] = ax1[1];
			info->m_J1angularAxis[srow + 2] = ax1[2];

			info->m_J2angularAxis[srow + 0] = -ax1[0];
			info->m_J2angularAxis[srow + 1] = -ax1[1];
			info->m_J2angularAxis[srow + 2] = -ax1[2];

			btScalar lostop = getLowerLimit();
			btScalar histop = getUpperLimit();
			if (limit && (lostop == histop))
			{  // the joint motor is ineffective
				powered = false;
			}
			info->m_constraintError[srow] = btScalar(0.0f);
			btScalar currERP = (m_flags & BT_HINGE_FLAGS_ERP_STOP) ? m_stopERP : normalErp;
			if (powered)
			{
				if (m_flags & BT_HINGE_FLAGS_CFM_NORM)
				{
					info->cfm[srow] = m_normalCFM;
				}
				btScalar mot_fact = getMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info->fps * currERP);
				info->m_constraintError[srow] += mot_fact * m_motorTargetVelocity * m_referenceSign;
				info->m_lowerLimit[srow] = -m_maxMotorImpulse;
				info->m_upperLimit[srow] = m_maxMotorImpulse;
			}
			if (limit)
			{
				k = info->fps * currERP;
				info->m_constraintError[srow] += k * limit_err;
				if (m_flags & BT_HINGE_FLAGS_CFM_STOP)
				{
					info->cfm[srow] = m_stopCFM;
				}
				if (lostop == histop)
				{
					// limited low and high simultaneously
					info->m_lowerLimit[srow] = -SIMD_INFINITY;
					info->m_upperLimit[srow] = SIMD_INFINITY;
				}
				else if (limit == 1)
				{  // low limit
					info->m_lowerLimit[srow] = 0;
					info->m_upperLimit[srow] = SIMD_INFINITY;
				}
				else
				{  // high limit
					info->m_lowerLimit[srow] = -SIMD_INFINITY;
					info->m_upperLimit[srow] = 0;
				}
				// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
#ifdef _BT_USE_CENTER_LIMIT_
				btScalar bounce = m_limit.getRelaxationFactor();
#else
				btScalar bounce = m_relaxationFactor;
#endif
				if (bounce > btScalar(0.0))
				{
					btScalar vel = angVelA.dot(ax1);
					vel -= angVelB.dot(ax1);
					// only apply bounce if the velocity is incoming, and if the
					// resulting c[] exceeds what we already have.
					if (limit == 1)
					{  // low limit
						if (vel < 0)
						{
							btScalar newc = -bounce * vel;
							if (newc > info->m_constraintError[srow])
							{
								info->m_constraintError[srow] = newc;
							}
						}
					}
					else
					{  // high limit - all those computations are reversed
						if (vel > 0)
						{
							btScalar newc = -bounce * vel;
							if (newc < info->m_constraintError[srow])
							{
								info->m_constraintError[srow] = newc;
							}
						}
					}
				}
#ifdef _BT_USE_CENTER_LIMIT_
				info->m_constraintError[srow] *= m_limit.getBiasFactor();
#else
				info->m_constraintError[srow] *= m_biasFactor;
#endif
			}  // if(limit)
		}      // if angular limit or powered
	}

	void updateRHS(btScalar timeStep)
	{
		(void)timeStep;
	}

	const btRigidBody& getRigidBodyA() const
	{
		return m_rbA;
	}
	const btRigidBody& getRigidBodyB() const
	{
		return m_rbB;
	}

	btRigidBody& getRigidBodyA()
	{
		return m_rbA;
	}

	btRigidBody& getRigidBodyB()
	{
		return m_rbB;
	}

	btTransform& getFrameOffsetA()
	{
		return m_rbAFrame;
	}

	btTransform& getFrameOffsetB()
	{
		return m_rbBFrame;
	}

	void setFrames(const btTransform& frameA, const btTransform& frameB)
	{
		m_rbAFrame = frameA;
		m_rbBFrame = frameB;
		buildJacobian();
	}

	void setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}

	void enableAngularMotor(bool enableMotor, btScalar targetVelocity, btScalar maxMotorImpulse)
	{
		m_enableAngularMotor = enableMotor;
		m_motorTargetVelocity = targetVelocity;
		m_maxMotorImpulse = maxMotorImpulse;
	}

	// extra motor API, including ability to set a target rotation (as opposed to angular velocity)
	// note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
	//       maintain a given angular target.
	void enableMotor(bool enableMotor) { m_enableAngularMotor = enableMotor; }
	void setMaxMotorImpulse(btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; }
	void setMotorTargetVelocity(btScalar motorTargetVelocity) { m_motorTargetVelocity = motorTargetVelocity; }
	void setMotorTarget(const btQuaternion& qAinB, btScalar dt) // qAinB is rotation of body A wrt body B
	{
		// convert target from body to constraint space
		btQuaternion qConstraint = m_rbBFrame.getRotation().inverse() * qAinB * m_rbAFrame.getRotation();
		qConstraint.normalize();

		// extract "pure" hinge component
		btVector3 vNoHinge = quatRotate(qConstraint, vHinge);
		vNoHinge.normalize();
		btQuaternion qNoHinge = shortestArcQuat(vHinge, vNoHinge);
		btQuaternion qHinge = qNoHinge.inverse() * qConstraint;
		qHinge.normalize();

		// compute angular target, clamped to limits
		btScalar targetAngle = qHinge.getAngle();
		if (targetAngle > SIMD_PI)  // long way around. flip quat and recalculate.
		{
			qHinge = -(qHinge);
			targetAngle = qHinge.getAngle();
		}
		if (qHinge.getZ() < 0)
			targetAngle = -targetAngle;

		setMotorTarget(targetAngle, dt);
	}
	void setMotorTarget(btScalar targetAngle, btScalar dt)
	{
#ifdef _BT_USE_CENTER_LIMIT_
		m_limit.fit(targetAngle);
#else
		if (m_lowerLimit < m_upperLimit)
		{
			if (targetAngle < m_lowerLimit)
				targetAngle = m_lowerLimit;
			else if (targetAngle > m_upperLimit)
				targetAngle = m_upperLimit;
		}
#endif
		// compute angular velocity
		btScalar curAngle = getHingeAngle(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
		btScalar dAngle = targetAngle - curAngle;
		m_motorTargetVelocity = dAngle / dt;
	}

	void setLimit(btScalar low, btScalar high, btScalar _softness = 0.9f, btScalar _biasFactor = 0.3f, btScalar _relaxationFactor = 1.0f)
	{
#ifdef _BT_USE_CENTER_LIMIT_
		m_limit.set(low, high, _softness, _biasFactor, _relaxationFactor);
#else
		m_lowerLimit = btNormalizeAngle(low);
		m_upperLimit = btNormalizeAngle(high);
		m_limitSoftness = _softness;
		m_biasFactor = _biasFactor;
		m_relaxationFactor = _relaxationFactor;
#endif
	}

	btScalar getLimitSoftness() const
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.getSoftness();
#else
		return m_limitSoftness;
#endif
	}

	btScalar getLimitBiasFactor() const
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.getBiasFactor();
#else
		return m_biasFactor;
#endif
	}

	btScalar getLimitRelaxationFactor() const
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.getRelaxationFactor();
#else
		return m_relaxationFactor;
#endif
	}

	void setAxis(btVector3 & axisInA)
	{
		btVector3 rbAxisA1, rbAxisA2;
		btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
		btVector3 pivotInA = m_rbAFrame.getOrigin();
		//		m_rbAFrame.getOrigin() = pivotInA;
		m_rbAFrame.getBasis().setValue(rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(),
									   rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(),
									   rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ());

		btVector3 axisInB = m_rbA.getCenterOfMassTransform().getBasis() * axisInA;

		btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
		btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
		btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);

		m_rbBFrame.getOrigin() = m_rbB.getCenterOfMassTransform().inverse()(m_rbA.getCenterOfMassTransform()(pivotInA));

		m_rbBFrame.getBasis().setValue(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(),
									   rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(),
									   rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ());
		m_rbBFrame.getBasis() = m_rbB.getCenterOfMassTransform().getBasis().inverse() * m_rbBFrame.getBasis();
	}

	bool hasLimit() const
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.getHalfRange() > 0;
#else
		return m_lowerLimit <= m_upperLimit;
#endif
	}

	btScalar getLowerLimit() const
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.getLow();
#else
		return m_lowerLimit;
#endif
	}

	btScalar getUpperLimit() const
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.getHigh();
#else
		return m_upperLimit;
#endif
	}

	///The getHingeAngle gives the hinge angle in range [-PI,PI]
	btScalar getHingeAngle()
	{
		return getHingeAngle(m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
	}

	btScalar getHingeAngle(const btTransform& transA, const btTransform& transB)
	{
		const btVector3 refAxis0 = transA.getBasis() * m_rbAFrame.getBasis().getColumn(0);
		const btVector3 refAxis1 = transA.getBasis() * m_rbAFrame.getBasis().getColumn(1);
		const btVector3 swingAxis = transB.getBasis() * m_rbBFrame.getBasis().getColumn(1);
		//	btScalar angle = btAtan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
		btScalar angle = btAtan2(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
		return m_referenceSign * angle;
	}

	void testLimit(const btTransform& transA, const btTransform& transB)
	{
		// Compute limit information
		m_hingeAngle = getHingeAngle(transA, transB);
#ifdef _BT_USE_CENTER_LIMIT_
		m_limit.test(m_hingeAngle);
#else
		m_correction = btScalar(0.);
		m_limitSign = btScalar(0.);
		m_solveLimit = false;
		if (m_lowerLimit <= m_upperLimit)
		{
			m_hingeAngle = btAdjustAngleToLimits(m_hingeAngle, m_lowerLimit, m_upperLimit);
			if (m_hingeAngle <= m_lowerLimit)
			{
				m_correction = (m_lowerLimit - m_hingeAngle);
				m_limitSign = 1.0f;
				m_solveLimit = true;
			}
			else if (m_hingeAngle >= m_upperLimit)
			{
				m_correction = m_upperLimit - m_hingeAngle;
				m_limitSign = -1.0f;
				m_solveLimit = true;
			}
		}
#endif
		return;
	}

	const btTransform& getAFrame() const { return m_rbAFrame; };
	const btTransform& getBFrame() const { return m_rbBFrame; };

	btTransform& getAFrame() { return m_rbAFrame; };
	btTransform& getBFrame() { return m_rbBFrame; };

	inline int getSolveLimit()
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.isLimit();
#else
		return m_solveLimit;
#endif
	}

	inline btScalar getLimitSign()
	{
#ifdef _BT_USE_CENTER_LIMIT_
		return m_limit.getSign();
#else
		return m_limitSign;
#endif
	}

	inline bool getAngularOnly()
	{
		return m_angularOnly;
	}
	inline bool getEnableAngularMotor()
	{
		return m_enableAngularMotor;
	}
	inline btScalar getMotorTargetVelocity()
	{
		return m_motorTargetVelocity;
	}
	inline btScalar getMaxMotorImpulse()
	{
		return m_maxMotorImpulse;
	}
	// access for UseFrameOffset
	bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }
	// access for UseReferenceFrameA
	bool getUseReferenceFrameA() const { return m_useReferenceFrameA; }
	void setUseReferenceFrameA(bool useReferenceFrameA) { m_useReferenceFrameA = useReferenceFrameA; }

	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5).
	///If no axis is provided, it uses the default axis for this constraint.
	virtual void setParam(int num, btScalar value, int axis = -1)
	{
		if ((axis == -1) || (axis == 5))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					m_stopERP = value;
					m_flags |= BT_HINGE_FLAGS_ERP_STOP;
					break;
				case BT_CONSTRAINT_STOP_CFM:
					m_stopCFM = value;
					m_flags |= BT_HINGE_FLAGS_CFM_STOP;
					break;
				case BT_CONSTRAINT_CFM:
					m_normalCFM = value;
					m_flags |= BT_HINGE_FLAGS_CFM_NORM;
					break;
				case BT_CONSTRAINT_ERP:
					m_normalERP = value;
					m_flags |= BT_HINGE_FLAGS_ERP_NORM;
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
	///return the local value of parameter
	virtual btScalar getParam(int num, int axis = -1) const
	{
		btScalar retVal = 0;
		if ((axis == -1) || (axis == 5))
		{
			switch (num)
			{
				case BT_CONSTRAINT_STOP_ERP:
					btAssertConstrParams(m_flags & BT_HINGE_FLAGS_ERP_STOP);
					retVal = m_stopERP;
					break;
				case BT_CONSTRAINT_STOP_CFM:
					btAssertConstrParams(m_flags & BT_HINGE_FLAGS_CFM_STOP);
					retVal = m_stopCFM;
					break;
				case BT_CONSTRAINT_CFM:
					btAssertConstrParams(m_flags & BT_HINGE_FLAGS_CFM_NORM);
					retVal = m_normalCFM;
					break;
				case BT_CONSTRAINT_ERP:
					btAssertConstrParams(m_flags & BT_HINGE_FLAGS_ERP_NORM);
					retVal = m_normalERP;
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

	virtual int getFlags() const
	{
		return m_flags;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

//only for backward compatibility
#ifdef BT_BACKWARDS_COMPATIBLE_SERIALIZATION
///this structure is not used, except for loading pre-2.82 .bullet files
struct btHingeConstraintDoubleData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame;  // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	int m_useReferenceFrameA;
	int m_angularOnly;
	int m_enableAngularMotor;
	float m_motorTargetVelocity;
	float m_maxMotorImpulse;

	float m_lowerLimit;
	float m_upperLimit;
	float m_limitSoftness;
	float m_biasFactor;
	float m_relaxationFactor;
};
#endif  //BT_BACKWARDS_COMPATIBLE_SERIALIZATION

///The getAccumulatedHingeAngle returns the accumulated hinge angle, taking rotation across the -PI/PI boundary into account
ATTRIBUTE_ALIGNED16(class)
btHingeAccumulatedAngleConstraint : public btHingeConstraint
{
protected:
	btScalar m_accumulatedAngle;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btHingeAccumulatedAngleConstraint(btRigidBody & rbA, btRigidBody & rbB, const btVector3& pivotInA, const btVector3& pivotInB, const btVector3& axisInA, const btVector3& axisInB, bool useReferenceFrameA = false)
		: btHingeConstraint(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB, useReferenceFrameA)
	{
		m_accumulatedAngle = getHingeAngle();
	}

	btHingeAccumulatedAngleConstraint(btRigidBody & rbA, const btVector3& pivotInA, const btVector3& axisInA, bool useReferenceFrameA = false)
		: btHingeConstraint(rbA, pivotInA, axisInA, useReferenceFrameA)
	{
		m_accumulatedAngle = getHingeAngle();
	}

	btHingeAccumulatedAngleConstraint(btRigidBody & rbA, btRigidBody & rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA = false)
		: btHingeConstraint(rbA, rbB, rbAFrame, rbBFrame, useReferenceFrameA)
	{
		m_accumulatedAngle = getHingeAngle();
	}

	btHingeAccumulatedAngleConstraint(btRigidBody & rbA, const btTransform& rbAFrame, bool useReferenceFrameA = false)
		: btHingeConstraint(rbA, rbAFrame, useReferenceFrameA)
	{
		m_accumulatedAngle = getHingeAngle();
	}
	btScalar getAccumulatedHingeAngle()
	{
		btScalar hingeAngle = getHingeAngle();
		m_accumulatedAngle = btShortestAngleUpdate(m_accumulatedAngle, hingeAngle);
		return m_accumulatedAngle;
	}
	void setAccumulatedHingeAngle(btScalar accAngle)
	{
		m_accumulatedAngle = accAngle;
	}
	virtual void getInfo1(btConstraintInfo1 * info)
	{
		//update m_accumulatedAngle
		btScalar curHingeAngle = getHingeAngle();
		m_accumulatedAngle = btShortestAngleUpdate(m_accumulatedAngle, curHingeAngle);

		btHingeConstraint::getInfo1(info);
	}
};

struct btHingeConstraintFloatData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformFloatData m_rbAFrame;  // constraint axii. Assumes z is hinge axis.
	btTransformFloatData m_rbBFrame;
	int m_useReferenceFrameA;
	int m_angularOnly;

	int m_enableAngularMotor;
	float m_motorTargetVelocity;
	float m_maxMotorImpulse;

	float m_lowerLimit;
	float m_upperLimit;
	float m_limitSoftness;
	float m_biasFactor;
	float m_relaxationFactor;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btHingeConstraintDoubleData2
{
	btTypedConstraintDoubleData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame;  // constraint axii. Assumes z is hinge axis.
	btTransformDoubleData m_rbBFrame;
	int m_useReferenceFrameA;
	int m_angularOnly;
	int m_enableAngularMotor;
	double m_motorTargetVelocity;
	double m_maxMotorImpulse;

	double m_lowerLimit;
	double m_upperLimit;
	double m_limitSoftness;
	double m_biasFactor;
	double m_relaxationFactor;
	char m_padding1[4];
};

SIMD_FORCE_INLINE int btHingeConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btHingeConstraintData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btHingeConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btHingeConstraintData* hingeData = (btHingeConstraintData*)dataBuffer;
	btTypedConstraint::serialize(&hingeData->m_typeConstraintData, serializer);

	m_rbAFrame.serialize(hingeData->m_rbAFrame);
	m_rbBFrame.serialize(hingeData->m_rbBFrame);

	hingeData->m_angularOnly = m_angularOnly;
	hingeData->m_enableAngularMotor = m_enableAngularMotor;
	hingeData->m_maxMotorImpulse = float(m_maxMotorImpulse);
	hingeData->m_motorTargetVelocity = float(m_motorTargetVelocity);
	hingeData->m_useReferenceFrameA = m_useReferenceFrameA;
#ifdef _BT_USE_CENTER_LIMIT_
	hingeData->m_lowerLimit = float(m_limit.getLow());
	hingeData->m_upperLimit = float(m_limit.getHigh());
	hingeData->m_limitSoftness = float(m_limit.getSoftness());
	hingeData->m_biasFactor = float(m_limit.getBiasFactor());
	hingeData->m_relaxationFactor = float(m_limit.getRelaxationFactor());
#else
	hingeData->m_lowerLimit = float(m_lowerLimit);
	hingeData->m_upperLimit = float(m_upperLimit);
	hingeData->m_limitSoftness = float(m_limitSoftness);
	hingeData->m_biasFactor = float(m_biasFactor);
	hingeData->m_relaxationFactor = float(m_relaxationFactor);
#endif

	// Fill padding with zeros to appease msan.
#ifdef BT_USE_DOUBLE_PRECISION
	hingeData->m_padding1[0] = 0;
	hingeData->m_padding1[1] = 0;
	hingeData->m_padding1[2] = 0;
	hingeData->m_padding1[3] = 0;
#endif

	return btHingeConstraintDataName;
}

#endif  //BT_HINGECONSTRAINT_H
