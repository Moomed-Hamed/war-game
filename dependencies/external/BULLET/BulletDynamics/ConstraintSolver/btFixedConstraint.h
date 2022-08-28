#ifndef BT_FIXED_CONSTRAINT_H
#define BT_FIXED_CONSTRAINT_H

#include "btGeneric6DofSpring2Constraint.h"

ATTRIBUTE_ALIGNED16(class)
btFixedConstraint : public btGeneric6DofSpring2Constraint
{
public:
	btFixedConstraint(btRigidBody & rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB) : btGeneric6DofSpring2Constraint(rbA, rbB, frameInA, frameInB)
	{
		setAngularLowerLimit(btVector3(0, 0, 0));
		setAngularUpperLimit(btVector3(0, 0, 0));
		setLinearLowerLimit (btVector3(0, 0, 0));
		setLinearUpperLimit (btVector3(0, 0, 0));
	}

	virtual ~btFixedConstraint() {}
};

#endif  //BT_FIXED_CONSTRAINT_H
