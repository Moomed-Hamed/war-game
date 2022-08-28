#ifndef _BT_ACTION_INTERFACE_H
#define _BT_ACTION_INTERFACE_H

class btIDebugDraw;
class btCollisionWorld;

#include "LinearMath/btScalar.h"
#include "btRigidBody.h"

///Basic interface to allow actions such as vehicles and characters to be updated inside a btDynamicsWorld
class btActionInterface
{
protected:
	static btRigidBody& getFixedBody();

public:
	virtual ~btActionInterface() {}

	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep) = 0;

	virtual void debugDraw(btIDebugDraw* debugDrawer) = 0;
};

#endif  //_BT_ACTION_INTERFACE_H
