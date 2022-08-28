#ifndef BT_SOFTBODY_RIGIDBODY_COLLISION_CONFIGURATION
#define BT_SOFTBODY_RIGIDBODY_COLLISION_CONFIGURATION

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "btSoftRigidCollisionAlgorithm.h"
#include "btSoftBodyConcaveCollisionAlgorithm.h"
#include "btSoftSoftCollisionAlgorithm.h"
#include "LinearMath/btPoolAllocator.h"

class btVoronoiSimplexSolver;
class btGjkEpaPenetrationDepthSolver;

#define ENABLE_SOFTBODY_CONCAVE_COLLISIONS 1

///btSoftBodyRigidBodyCollisionConfiguration add softbody interaction on top of btDefaultCollisionConfiguration
class btSoftBodyRigidBodyCollisionConfiguration : public btDefaultCollisionConfiguration
{
	//default CreationFunctions, filling the m_doubleDispatch table
	btCollisionAlgorithmCreateFunc* m_softSoftCreateFunc;
	btCollisionAlgorithmCreateFunc* m_softRigidConvexCreateFunc;
	btCollisionAlgorithmCreateFunc* m_swappedSoftRigidConvexCreateFunc;
	btCollisionAlgorithmCreateFunc* m_softRigidConcaveCreateFunc;
	btCollisionAlgorithmCreateFunc* m_swappedSoftRigidConcaveCreateFunc;

public:
	btSoftBodyRigidBodyCollisionConfiguration(const btDefaultCollisionConstructionInfo& constructionInfo = btDefaultCollisionConstructionInfo())
		: btDefaultCollisionConfiguration(constructionInfo)
	{
		void* mem;

		mem = btAlignedAlloc(sizeof(btSoftSoftCollisionAlgorithm::CreateFunc), 16);
		m_softSoftCreateFunc = new (mem) btSoftSoftCollisionAlgorithm::CreateFunc;

		mem = btAlignedAlloc(sizeof(btSoftRigidCollisionAlgorithm::CreateFunc), 16);
		m_softRigidConvexCreateFunc = new (mem) btSoftRigidCollisionAlgorithm::CreateFunc;

		mem = btAlignedAlloc(sizeof(btSoftRigidCollisionAlgorithm::CreateFunc), 16);
		m_swappedSoftRigidConvexCreateFunc = new (mem) btSoftRigidCollisionAlgorithm::CreateFunc;
		m_swappedSoftRigidConvexCreateFunc->m_swapped = true;

#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS
		mem = btAlignedAlloc(sizeof(btSoftBodyConcaveCollisionAlgorithm::CreateFunc), 16);
		m_softRigidConcaveCreateFunc = new (mem) btSoftBodyConcaveCollisionAlgorithm::CreateFunc;

		mem = btAlignedAlloc(sizeof(btSoftBodyConcaveCollisionAlgorithm::CreateFunc), 16);
		m_swappedSoftRigidConcaveCreateFunc = new (mem) btSoftBodyConcaveCollisionAlgorithm::SwappedCreateFunc;
		m_swappedSoftRigidConcaveCreateFunc->m_swapped = true;
#endif

		//replace pool by a new one, with potential larger size

		if (m_ownsCollisionAlgorithmPool && m_collisionAlgorithmPool)
		{
			int curElemSize = m_collisionAlgorithmPool->getElementSize();
			///calculate maximum element size, big enough to fit any collision algorithm in the memory pool

			int maxSize0 = sizeof(btSoftSoftCollisionAlgorithm);
			int maxSize1 = sizeof(btSoftRigidCollisionAlgorithm);
			int maxSize2 = sizeof(btSoftBodyConcaveCollisionAlgorithm);

			int collisionAlgorithmMaxElementSize = btMax(maxSize0, maxSize1);
			collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize, maxSize2);

			if (collisionAlgorithmMaxElementSize > curElemSize)
			{
				m_collisionAlgorithmPool->~btPoolAllocator();
				btAlignedFree(m_collisionAlgorithmPool);
				void* mem = btAlignedAlloc(sizeof(btPoolAllocator), 16);
				m_collisionAlgorithmPool = new (mem) btPoolAllocator(collisionAlgorithmMaxElementSize, constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
			}
		}
	}

	virtual ~btSoftBodyRigidBodyCollisionConfiguration()
	{
		m_softSoftCreateFunc->~btCollisionAlgorithmCreateFunc();
		btAlignedFree(m_softSoftCreateFunc);

		m_softRigidConvexCreateFunc->~btCollisionAlgorithmCreateFunc();
		btAlignedFree(m_softRigidConvexCreateFunc);

		m_swappedSoftRigidConvexCreateFunc->~btCollisionAlgorithmCreateFunc();
		btAlignedFree(m_swappedSoftRigidConvexCreateFunc);

#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS
		m_softRigidConcaveCreateFunc->~btCollisionAlgorithmCreateFunc();
		btAlignedFree(m_softRigidConcaveCreateFunc);

		m_swappedSoftRigidConcaveCreateFunc->~btCollisionAlgorithmCreateFunc();
		btAlignedFree(m_swappedSoftRigidConcaveCreateFunc);
#endif
	}

	///creation of soft-soft and soft-rigid, and otherwise fallback to base class implementation
	virtual btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1)
	{
		///try to handle the softbody interactions first

		if ((proxyType0 == SOFTBODY_SHAPE_PROXYTYPE) && (proxyType1 == SOFTBODY_SHAPE_PROXYTYPE))
		{
			return m_softSoftCreateFunc;
		}

		///softbody versus convex
		if (proxyType0 == SOFTBODY_SHAPE_PROXYTYPE && btBroadphaseProxy::isConvex(proxyType1))
		{
			return m_softRigidConvexCreateFunc;
		}

		///convex versus soft body
		if (btBroadphaseProxy::isConvex(proxyType0) && proxyType1 == SOFTBODY_SHAPE_PROXYTYPE)
		{
			return m_swappedSoftRigidConvexCreateFunc;
		}

#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS
		///softbody versus convex
		if (proxyType0 == SOFTBODY_SHAPE_PROXYTYPE && btBroadphaseProxy::isConcave(proxyType1))
		{
			return m_softRigidConcaveCreateFunc;
		}

		///convex versus soft body
		if (btBroadphaseProxy::isConcave(proxyType0) && proxyType1 == SOFTBODY_SHAPE_PROXYTYPE)
		{
			return m_swappedSoftRigidConcaveCreateFunc;
		}
#endif

		///fallback to the regular rigid collision shape
		return btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(proxyType0, proxyType1);
	}
};

#endif  //BT_SOFTBODY_RIGIDBODY_COLLISION_CONFIGURATION
