#ifndef BT_MULTIBODY_JOINT_FEEDBACK_H
#define BT_MULTIBODY_JOINT_FEEDBACK_H

#include "LinearMath/btSpatialAlgebra.h"

struct btMultiBodyJointFeedback
{
	btSpatialForceVector m_reactionForces;
};

#endif  //BT_MULTIBODY_JOINT_FEEDBACK_H
