#ifndef BT_MULTIBODY_SOLVER_CONSTRAINT_H
#define BT_MULTIBODY_SOLVER_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

class btMultiBody;
class btMultiBodyConstraint;
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

///1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
ATTRIBUTE_ALIGNED16(struct)
btMultiBodySolverConstraint
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btMultiBodySolverConstraint() : m_solverBodyIdA(-1), m_multiBodyA(0), m_linkA(-1), m_solverBodyIdB(-1), m_multiBodyB(0), m_linkB(-1), m_orgConstraint(0), m_orgDofIndex(-1)
	{
	}

	int m_deltaVelAindex;  //more generic version of m_relpos1CrossNormal/m_contactNormal1
	int m_jacAindex;
	int m_deltaVelBindex;
	int m_jacBindex;

	btVector3 m_relpos1CrossNormal;
	btVector3 m_contactNormal1;
	btVector3 m_relpos2CrossNormal;
	btVector3 m_contactNormal2;  //usually m_contactNormal2 == -m_contactNormal1, but not always

	btVector3 m_angularComponentA;
	btVector3 m_angularComponentB;

	mutable btSimdScalar m_appliedPushImpulse;
	mutable btSimdScalar m_appliedImpulse;

	btScalar m_friction;
	btScalar m_jacDiagABInv;
	btScalar m_rhs;
	btScalar m_cfm;

	btScalar m_lowerLimit;
	btScalar m_upperLimit;
	btScalar m_rhsPenetration;
	union {
		void* m_originalContactPoint;
		btScalar m_unusedPadding4;
	};

	int m_overrideNumSolverIterations;
	int m_frictionIndex;

	int m_solverBodyIdA;
	btMultiBody* m_multiBodyA;
	int m_linkA;

	int m_solverBodyIdB;
	btMultiBody* m_multiBodyB;
	int m_linkB;

	//for writing back applied impulses
	btMultiBodyConstraint* m_orgConstraint;
	int m_orgDofIndex;

	enum btSolverConstraintType
	{
		BT_SOLVER_CONTACT_1D = 0,
		BT_SOLVER_FRICTION_1D
	};
};

typedef btAlignedObjectArray<btMultiBodySolverConstraint> btMultiBodyConstraintArray;

#endif  //BT_MULTIBODY_SOLVER_CONSTRAINT_H
