#ifndef B3_SOLVER_CONSTRAINT_H
#define B3_SOLVER_CONSTRAINT_H

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "Bullet3Common/b3AlignedObjectArray.h"

#include "b3SolverBody.h"

///1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
B3_ATTRIBUTE_ALIGNED16(struct)
b3SolverConstraint
{
	B3_DECLARE_ALIGNED_ALLOCATOR();

	b3Vector3 m_relpos1CrossNormal;
	b3Vector3 m_contactNormal;

	b3Vector3 m_relpos2CrossNormal;
	//b3Vector3		m_contactNormal2;//usually m_contactNormal2 == -m_contactNormal

	b3Vector3 m_angularComponentA;
	b3Vector3 m_angularComponentB;

	mutable b3SimdScalar m_appliedPushImpulse;
	mutable b3SimdScalar m_appliedImpulse;
	int m_padding1;
	int m_padding2;
	b3Scalar m_friction;
	b3Scalar m_jacDiagABInv;
	b3Scalar m_rhs;
	b3Scalar m_cfm;

	b3Scalar m_lowerLimit;
	b3Scalar m_upperLimit;
	b3Scalar m_rhsPenetration;
	union {
		void* m_originalContactPoint;
		b3Scalar m_unusedPadding4;
	};

	int m_overrideNumSolverIterations;
	int m_frictionIndex;
	int m_solverBodyIdA;
	int m_solverBodyIdB;

	enum b3SolverConstraintType
	{
		B3_SOLVER_CONTACT_1D = 0,
		B3_SOLVER_FRICTION_1D
	};
};

typedef b3AlignedObjectArray<b3SolverConstraint> b3ConstraintArray;

#endif  //B3_SOLVER_CONSTRAINT_H
