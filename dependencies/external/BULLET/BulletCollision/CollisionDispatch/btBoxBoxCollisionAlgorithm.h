#ifndef BT_BOX_BOX__COLLISION_ALGORITHM_H
#define BT_BOX_BOX__COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "btBoxBoxDetector.h"

class btPersistentManifold;

///box-box collision detection
class btBoxBoxCollisionAlgorithm : public btActivatingCollisionAlgorithm
{
	bool m_ownManifold;
	btPersistentManifold* m_manifoldPtr;

public:
	btBoxBoxCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
		: btActivatingCollisionAlgorithm(ci) {}

	virtual void processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		if (!m_manifoldPtr)
			return;

		const btBoxShape* box0 = (btBoxShape*)body0Wrap->getCollisionShape();
		const btBoxShape* box1 = (btBoxShape*)body1Wrap->getCollisionShape();

		/// report a contact. internally this will be kept persistent, and contact reduction is done
		resultOut->setPersistentManifold(m_manifoldPtr);

		btDiscreteCollisionDetectorInterface::ClosestPointInput input;
		input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
		input.m_transformA = body0Wrap->getWorldTransform();
		input.m_transformB = body1Wrap->getWorldTransform();

		btBoxBoxDetector detector(box0, box1);
		detector.getClosestPoints(input, *resultOut, dispatchInfo.m_debugDraw);

		//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
		if (m_ownManifold)
		{
			resultOut->refreshContactPoints();
		}
	}

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		return 1.f; // not yet
	}

	btBoxBoxCollisionAlgorithm(btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		: btActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
		  m_ownManifold(false),
		  m_manifoldPtr(mf)
	{
		if (!m_manifoldPtr && m_dispatcher->needsCollision(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject()))
		{
			m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
			m_ownManifold = true;
		}
	}

	virtual ~btBoxBoxCollisionAlgorithm()
	{
		if (m_ownManifold)
		{
			if (m_manifoldPtr)
				m_dispatcher->releaseManifold(m_manifoldPtr);
		}
	}

	virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	struct CreateFunc : public btCollisionAlgorithmCreateFunc
	{
		virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		{
			int bbsize = sizeof(btBoxBoxCollisionAlgorithm);
			void* ptr = ci.m_dispatcher1->allocateCollisionAlgorithm(bbsize);
			return new (ptr) btBoxBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap);
		}
	};
};

#endif  //BT_BOX_BOX__COLLISION_ALGORITHM_H
