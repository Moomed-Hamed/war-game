#ifndef BULLET_DYNAMICS_COMMON_H
#define BULLET_DYNAMICS_COMMON_H

///Common headerfile includes for Bullet Dynamics, including Collision Detection
#include "btBulletCollisionCommon.h"

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "BulletDynamics/Dynamics/btSimpleDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
#include "BulletDynamics/ConstraintSolver/btUniversalConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHinge2Constraint.h"
#include "BulletDynamics/ConstraintSolver/btGearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btFixedConstraint.h"

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

///Vehicle simulation, with wheel contact simulated by raycasts
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#endif  //BULLET_DYNAMICS_COMMON_H
