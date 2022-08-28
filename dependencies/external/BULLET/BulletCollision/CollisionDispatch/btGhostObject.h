#ifndef BT_GHOST_OBJECT_H
#define BT_GHOST_OBJECT_H

#include "btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"

class btConvexShape;
class btDispatcher;

///The btGhostObject can keep track of all objects that are overlapping
///By default, this overlap is based on the AABB
///This is useful for creating a character controller, collision sensors/triggers, explosions etc.
///We plan on adding rayTest and other queries for the btGhostObject
ATTRIBUTE_ALIGNED16(class)
btGhostObject : public btCollisionObject
{
protected:
	btAlignedObjectArray<btCollisionObject*> m_overlappingObjects;

public:
	btGhostObject()
	{
		m_internalType = CO_GHOST_OBJECT;
	}

	virtual ~btGhostObject()
	{
		///btGhostObject should have been removed from the world, so no overlapping objects
		btAssert(!m_overlappingObjects.size());
	}

	void convexSweepTest(const class btConvexShape* castShape, const btTransform& convexFromWorld, const btTransform& convexToWorld, btCollisionWorld::ConvexResultCallback& resultCallback, btScalar allowedCcdPenetration = 0.f) const
	{
		btTransform convexFromTrans, convexToTrans;
		convexFromTrans = convexFromWorld;
		convexToTrans = convexToWorld;
		btVector3 castShapeAabbMin, castShapeAabbMax;
		/* Compute AABB that encompasses angular movement */
		{
			btVector3 linVel, angVel;
			btTransformUtil::calculateVelocity(convexFromTrans, convexToTrans, 1.0, linVel, angVel);
			btTransform R;
			R.setIdentity();
			R.setRotation(convexFromTrans.getRotation());
			castShape->calculateTemporalAabb(R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);
		}

		/// go over all objects, and if the ray intersects their aabb + cast shape aabb,
		// do a ray-shape query using convexCaster (CCD)
		int i;
		for (i = 0; i < m_overlappingObjects.size(); i++)
		{
			btCollisionObject* collisionObject = m_overlappingObjects[i];
			//only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
			{
				//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
				btVector3 collisionObjectAabbMin, collisionObjectAabbMax;
				collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(), collisionObjectAabbMin, collisionObjectAabbMax);
				AabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
				btScalar hitLambda = btScalar(1.);  //could use resultCallback.m_closestHitFraction, but needs testing
				btVector3 hitNormal;
				if (btRayAabb(convexFromWorld.getOrigin(), convexToWorld.getOrigin(), collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal))
				{
					btCollisionWorld::objectQuerySingle(castShape, convexFromTrans, convexToTrans,
														collisionObject,
														collisionObject->getCollisionShape(),
														collisionObject->getWorldTransform(),
														resultCallback,
														allowedCcdPenetration);
				}
			}
		}
	}

	void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, btCollisionWorld::RayResultCallback& resultCallback) const
	{
		btTransform rayFromTrans;
		rayFromTrans.setIdentity();
		rayFromTrans.setOrigin(rayFromWorld);
		btTransform rayToTrans;
		rayToTrans.setIdentity();
		rayToTrans.setOrigin(rayToWorld);

		int i;
		for (i = 0; i < m_overlappingObjects.size(); i++)
		{
			btCollisionObject* collisionObject = m_overlappingObjects[i];
			//only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
			{
				btCollisionWorld::rayTestSingle(rayFromTrans, rayToTrans,
												collisionObject,
												collisionObject->getCollisionShape(),
												collisionObject->getWorldTransform(),
												resultCallback);
			}
		}
	}

	///this method is mainly for expert/internal use only.
	virtual void addOverlappingObjectInternal(btBroadphaseProxy * otherProxy, btBroadphaseProxy* thisProxy = 0)
	{
		btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
		btAssert(otherObject);
		///if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
		int index = m_overlappingObjects.findLinearSearch(otherObject);
		if (index == m_overlappingObjects.size())
		{
			//not found
			m_overlappingObjects.push_back(otherObject);
		}
	}
	///this method is mainly for expert/internal use only.
	virtual void removeOverlappingObjectInternal(btBroadphaseProxy * otherProxy, btDispatcher * dispatcher, btBroadphaseProxy* thisProxy = 0)
	{
		btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
		btAssert(otherObject);
		int index = m_overlappingObjects.findLinearSearch(otherObject);
		if (index < m_overlappingObjects.size())
		{
			m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size() - 1];
			m_overlappingObjects.pop_back();
		}
	}

	int getNumOverlappingObjects() const
	{
		return m_overlappingObjects.size();
	}

	btCollisionObject* getOverlappingObject(int index)
	{
		return m_overlappingObjects[index];
	}

	const btCollisionObject* getOverlappingObject(int index) const
	{
		return m_overlappingObjects[index];
	}

	btAlignedObjectArray<btCollisionObject*>& getOverlappingPairs()
	{
		return m_overlappingObjects;
	}

	const btAlignedObjectArray<btCollisionObject*> getOverlappingPairs() const
	{
		return m_overlappingObjects;
	}

	//
	// internal cast
	//

	static const btGhostObject* upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType() == CO_GHOST_OBJECT)
			return (const btGhostObject*)colObj;
		return 0;
	}
	static btGhostObject* upcast(btCollisionObject * colObj)
	{
		if (colObj->getInternalType() == CO_GHOST_OBJECT)
			return (btGhostObject*)colObj;
		return 0;
	}
};

class btPairCachingGhostObject : public btGhostObject
{
	btHashedOverlappingPairCache* m_hashPairCache;

public:
	btPairCachingGhostObject()
	{
		m_hashPairCache = new (btAlignedAlloc(sizeof(btHashedOverlappingPairCache), 16)) btHashedOverlappingPairCache();
	}

	virtual ~btPairCachingGhostObject()
	{
		m_hashPairCache->~btHashedOverlappingPairCache();
		btAlignedFree(m_hashPairCache);
	}

	///this method is mainly for expert/internal use only.
	virtual void addOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btBroadphaseProxy* thisProxy = 0)
	{
		btBroadphaseProxy* actualThisProxy = thisProxy ? thisProxy : getBroadphaseHandle();
		btAssert(actualThisProxy);

		btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
		btAssert(otherObject);
		int index = m_overlappingObjects.findLinearSearch(otherObject);
		if (index == m_overlappingObjects.size())
		{
			m_overlappingObjects.push_back(otherObject);
			m_hashPairCache->addOverlappingPair(actualThisProxy, otherProxy);
		}
	}

	virtual void removeOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btDispatcher* dispatcher, btBroadphaseProxy* thisProxy1 = 0)
	{
		btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
		btBroadphaseProxy* actualThisProxy = thisProxy1 ? thisProxy1 : getBroadphaseHandle();
		btAssert(actualThisProxy);

		btAssert(otherObject);
		int index = m_overlappingObjects.findLinearSearch(otherObject);
		if (index < m_overlappingObjects.size())
		{
			m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size() - 1];
			m_overlappingObjects.pop_back();
			m_hashPairCache->removeOverlappingPair(actualThisProxy, otherProxy, dispatcher);
		}
	}

	btHashedOverlappingPairCache* getOverlappingPairCache()
	{
		return m_hashPairCache;
	}
};

///The btGhostPairCallback interfaces and forwards adding and removal of overlapping pairs from the btBroadphaseInterface to btGhostObject.
class btGhostPairCallback : public btOverlappingPairCallback
{
public:
	btGhostPairCallback()
	{
	}

	virtual ~btGhostPairCallback()
	{
	}

	virtual btBroadphasePair* addOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
	{
		btCollisionObject* colObj0 = (btCollisionObject*)proxy0->m_clientObject;
		btCollisionObject* colObj1 = (btCollisionObject*)proxy1->m_clientObject;
		btGhostObject* ghost0 = btGhostObject::upcast(colObj0);
		btGhostObject* ghost1 = btGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->addOverlappingObjectInternal(proxy1, proxy0);
		if (ghost1)
			ghost1->addOverlappingObjectInternal(proxy0, proxy1);
		return 0;
	}

	virtual void* removeOverlappingPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, btDispatcher* dispatcher)
	{
		btCollisionObject* colObj0 = (btCollisionObject*)proxy0->m_clientObject;
		btCollisionObject* colObj1 = (btCollisionObject*)proxy1->m_clientObject;
		btGhostObject* ghost0 = btGhostObject::upcast(colObj0);
		btGhostObject* ghost1 = btGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->removeOverlappingObjectInternal(proxy1, dispatcher, proxy0);
		if (ghost1)
			ghost1->removeOverlappingObjectInternal(proxy0, dispatcher, proxy1);
		return 0;
	}

	virtual void removeOverlappingPairsContainingProxy(btBroadphaseProxy* /*proxy0*/, btDispatcher* /*dispatcher*/)
	{
		btAssert(0);
		//need to keep track of all ghost objects and call them here
		//m_hashPairCache->removeOverlappingPairsContainingProxy(proxy0,dispatcher);
	}
};

#endif
