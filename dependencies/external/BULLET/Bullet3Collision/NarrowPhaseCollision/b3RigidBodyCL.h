#ifndef B3_RIGID_BODY_CL
#define B3_RIGID_BODY_CL

#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"

inline float b3GetInvMass(const b3RigidBodyData& body)
{
	return body.m_invMass;
}

#endif  //B3_RIGID_BODY_CL
