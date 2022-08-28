#ifndef __BT_ACTIVATING_COLLISION_ALGORITHM_H
#define __BT_ACTIVATING_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

///This class is not enabled yet (work-in-progress) to more aggressively activate objects.
class btActivatingCollisionAlgorithm : public btCollisionAlgorithm
{
protected:
	btActivatingCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci) : btCollisionAlgorithm(ci) {}
	btActivatingCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		: btCollisionAlgorithm(ci) {}

public:
	virtual ~btActivatingCollisionAlgorithm() {}
};
#endif  //__BT_ACTIVATING_COLLISION_ALGORITHM_H
