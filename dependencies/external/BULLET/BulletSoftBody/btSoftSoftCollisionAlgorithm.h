#ifndef BT_SOFT_SOFT_COLLISION_ALGORITHM_H
#define BT_SOFT_SOFT_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "btSoftBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

class btPersistentManifold;
class btSoftBody;

///collision detection between two btSoftBody shapes
class btSoftSoftCollisionAlgorithm : public btCollisionAlgorithm
{
	bool m_ownManifold;
	btPersistentManifold* m_manifoldPtr;

public:
	btSoftSoftCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci) : btCollisionAlgorithm(ci) {}

	virtual void processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& /*dispatchInfo*/, btManifoldResult* /*resultOut*/)
	{
		btSoftBody* soft0 = (btSoftBody*)body0Wrap->getCollisionObject();
		btSoftBody* soft1 = (btSoftBody*)body1Wrap->getCollisionObject();
		soft0->getSoftBodySolver()->processCollision(soft0, soft1);
	}

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		return 1.f; // not yet
	}

	virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
			manifoldArray.push_back(m_manifoldPtr);
	}

	btSoftSoftCollisionAlgorithm(btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		: btCollisionAlgorithm(ci) {}

	virtual ~btSoftSoftCollisionAlgorithm() {}

	struct CreateFunc : public btCollisionAlgorithmCreateFunc
	{
		virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		{
			int bbsize = sizeof(btSoftSoftCollisionAlgorithm);
			void* ptr = ci.m_dispatcher1->allocateCollisionAlgorithm(bbsize);
			return new (ptr) btSoftSoftCollisionAlgorithm(0, ci, body0Wrap, body1Wrap);
		}
	};
};

#endif  //BT_SOFT_SOFT_COLLISION_ALGORITHM_H
