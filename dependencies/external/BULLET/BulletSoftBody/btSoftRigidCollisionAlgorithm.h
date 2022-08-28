#ifndef BT_SOFT_RIGID_COLLISION_ALGORITHM_H
#define BT_SOFT_RIGID_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "btSoftBody.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "LinearMath/btQuickprof.h" // for BT_PROFILE i think

// TODO: include all the shapes that the softbody can collide with
// alternatively, implement special case collision algorithms (just like for rigid collision shapes)

/// btSoftRigidCollisionAlgorithm  provides collision detection between btSoftBody and btRigidBody
class btSoftRigidCollisionAlgorithm : public btCollisionAlgorithm
{
	bool m_isSwapped; // for rigid versus soft (instead of soft versus rigid), we use this swapped boolean

public:
	btSoftRigidCollisionAlgorithm(btPersistentManifold* /*mf*/, const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper*, const btCollisionObjectWrapper*, bool isSwapped)
		: btCollisionAlgorithm(ci),
		  //m_ownManifold(false),
		  //m_manifoldPtr(mf),
		  m_isSwapped(isSwapped)
	{
	}

	virtual ~btSoftRigidCollisionAlgorithm() {}

	virtual void processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		BT_PROFILE("btSoftRigidCollisionAlgorithm::processCollision");
		(void)dispatchInfo;
		(void)resultOut;
		//printf("btSoftRigidCollisionAlgorithm\n");
		btSoftBody* softBody = m_isSwapped ? (btSoftBody*)body1Wrap->getCollisionObject() : (btSoftBody*)body0Wrap->getCollisionObject();
		const btCollisionObjectWrapper* rigidCollisionObjectWrap = m_isSwapped ? body0Wrap : body1Wrap;

		if (softBody->m_collisionDisabledObjects.findLinearSearch(rigidCollisionObjectWrap->getCollisionObject()) == softBody->m_collisionDisabledObjects.size())
			softBody->getSoftBodySolver()->processCollision(softBody, rigidCollisionObjectWrap);
	}

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		return 1.f; //not yet
	}

	virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
	{
		//we don't add any manifolds
	}

	struct CreateFunc : public btCollisionAlgorithmCreateFunc
	{
		virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btSoftRigidCollisionAlgorithm));
			if (!m_swapped)
			{
				return new (mem) btSoftRigidCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
			}
			else
			{
				return new (mem) btSoftRigidCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
			}
		}
	};
};

#endif  //BT_SOFT_RIGID_COLLISION_ALGORITHM_H
