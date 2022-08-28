#ifndef BT_MOTIONSTATE_H
#define BT_MOTIONSTATE_H

#include "btTransform.h"

///The btMotionState interface class allows the dynamics world to synchronize and interpolate the updated world transforms with graphics
///For optimizations, potentially only moving objects get synchronized (using setWorldPosition/setWorldOrientation)
class btMotionState
{
public:
	virtual ~btMotionState()
	{
	}

	virtual void getWorldTransform(btTransform& worldTrans) const = 0;

	//Bullet only calls the update of worldtransform for active objects
	virtual void setWorldTransform(const btTransform& worldTrans) = 0;
};

#endif  //BT_MOTIONSTATE_H
