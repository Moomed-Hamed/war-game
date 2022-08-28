#ifndef BT_SIMPLE_BROADPHASE_H
#define BT_SIMPLE_BROADPHASE_H

#include "btOverlappingPairCache.h"

struct btSimpleBroadphaseProxy : public btBroadphaseProxy
{
	int m_nextFree;

	btSimpleBroadphaseProxy(){};

	btSimpleBroadphaseProxy(const btVector3& minpt, const btVector3& maxpt, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask)
		: btBroadphaseProxy(minpt, maxpt, userPtr, collisionFilterGroup, collisionFilterMask)
	{
		(void)shapeType;
	}

	SIMD_FORCE_INLINE void SetNextFree(int next) { m_nextFree = next; }
	SIMD_FORCE_INLINE int GetNextFree() const { return m_nextFree; }
};

///The SimpleBroadphase is just a unit-test for btAxisSweep3, bt32BitAxisSweep3, or btDbvtBroadphase, so use those classes instead.
///It is a brute force aabb culling broadphase based on O(n^2) aabb checks
class btSimpleBroadphase : public btBroadphaseInterface
{
protected:
	int m_numHandles;  // number of active handles
	int m_maxHandles;  // max number of handles
	int m_LastHandleIndex;

	btSimpleBroadphaseProxy* m_pHandles;  // handles pool

	void* m_pHandlesRawPtr;
	int m_firstFreeHandle;  // free handles list

	int allocHandle()
	{
		btAssert(m_numHandles < m_maxHandles);
		int freeHandle = m_firstFreeHandle;
		m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
		m_numHandles++;
		if (freeHandle > m_LastHandleIndex)
		{
			m_LastHandleIndex = freeHandle;
		}
		return freeHandle;
	}

	void freeHandle(btSimpleBroadphaseProxy* proxy)
	{
		int handle = int(proxy - m_pHandles);
		btAssert(handle >= 0 && handle < m_maxHandles);
		if (handle == m_LastHandleIndex)
		{
			m_LastHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		proxy->m_clientObject = 0;

		m_numHandles--;
	}

	btOverlappingPairCache* m_pairCache;
	bool m_ownsPairCache;

	int m_invalidPair;

	inline btSimpleBroadphaseProxy* getSimpleProxyFromProxy(btBroadphaseProxy* proxy)
	{
		btSimpleBroadphaseProxy* proxy0 = static_cast<btSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	inline const btSimpleBroadphaseProxy* getSimpleProxyFromProxy(btBroadphaseProxy* proxy) const
	{
		const btSimpleBroadphaseProxy* proxy0 = static_cast<const btSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(btDispatcher* dispatcher) {} // not yet

	void validate()
	{
		for (int i = 0    ; i < m_numHandles; i++) {
		for (int j = i + 1; j < m_numHandles; j++)
		{
			btAssert(&m_pHandles[i] != &m_pHandles[j]);
		} }
	}

protected:
public:
	btSimpleBroadphase(int maxProxies = 16384, btOverlappingPairCache* overlappingPairCache = 0)
		: m_pairCache(overlappingPairCache), m_ownsPairCache(false), m_invalidPair(0)
	{
		if (!overlappingPairCache)
		{
			void* mem = btAlignedAlloc(sizeof(btHashedOverlappingPairCache), 16);
			m_pairCache = new (mem) btHashedOverlappingPairCache();
			m_ownsPairCache = true;
		}

		// allocate handles buffer and put all handles on free list
		m_pHandlesRawPtr = btAlignedAlloc(sizeof(btSimpleBroadphaseProxy) * maxProxies, 16);
		m_pHandles = new (m_pHandlesRawPtr) btSimpleBroadphaseProxy[maxProxies];
		m_maxHandles = maxProxies;
		m_numHandles = 0;
		m_firstFreeHandle = 0;
		m_LastHandleIndex = -1;

		for (int i = m_firstFreeHandle; i < maxProxies; i++)
		{
			m_pHandles[i].SetNextFree(i + 1);
			m_pHandles[i].m_uniqueId = i + 2;  //any UID will do, we just avoid too trivial values (0,1) for debugging purposes
		}
		m_pHandles[maxProxies - 1].SetNextFree(0);
	}
	virtual ~btSimpleBroadphase()
	{
		btAlignedFree(m_pHandlesRawPtr);

		if (m_ownsPairCache)
		{
			m_pairCache->~btOverlappingPairCache();
			btAlignedFree(m_pairCache);
		}
	}

	static bool aabbOverlap(btSimpleBroadphaseProxy* proxy0, btSimpleBroadphaseProxy* proxy1)
	{
		return proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] && proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0] &&
			   proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] && proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1] &&
			   proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] && proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2];
	}

	virtual btBroadphaseProxy* createProxy(const btVector3& aabbMin, const btVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher* dispatcher)
	{
		if (m_numHandles >= m_maxHandles)
		{
			btAssert(0);
			return 0;  //should never happen, but don't let the game crash ;-)
		}
		btAssert(aabbMin[0] <= aabbMax[0] && aabbMin[1] <= aabbMax[1] && aabbMin[2] <= aabbMax[2]);

		int newHandleIndex = allocHandle();
		btSimpleBroadphaseProxy* proxy = new (&m_pHandles[newHandleIndex]) btSimpleBroadphaseProxy(aabbMin, aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask);

		return proxy;
	}

	virtual void calculateOverlappingPairs(btDispatcher* dispatcher)
	{
		//first check for new overlapping pairs
		int i, j;
		if (m_numHandles >= 0)
		{
			int new_largest_index = -1;
			for (i = 0; i <= m_LastHandleIndex; i++)
			{
				btSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
				if (!proxy0->m_clientObject)
				{
					continue;
				}
				new_largest_index = i;
				for (j = i + 1; j <= m_LastHandleIndex; j++)
				{
					btSimpleBroadphaseProxy* proxy1 = &m_pHandles[j];
					btAssert(proxy0 != proxy1);
					if (!proxy1->m_clientObject)
					{
						continue;
					}

					btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
					btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);

					if (aabbOverlap(p0, p1))
					{
						if (!m_pairCache->findPair(proxy0, proxy1))
						{
							m_pairCache->addOverlappingPair(proxy0, proxy1);
						}
					}
					else
					{
						if (!m_pairCache->hasDeferredRemoval())
						{
							if (m_pairCache->findPair(proxy0, proxy1))
							{
								m_pairCache->removeOverlappingPair(proxy0, proxy1, dispatcher);
							}
						}
					}
				}
			}

			m_LastHandleIndex = new_largest_index;

			if (m_ownsPairCache && m_pairCache->hasDeferredRemoval())
			{
				btBroadphasePairArray& overlappingPairArray = m_pairCache->getOverlappingPairArray();

				//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
				overlappingPairArray.quickSort(btBroadphasePairSortPredicate());

				overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
				m_invalidPair = 0;

				btBroadphasePair previousPair;
				previousPair.m_pProxy0 = 0;
				previousPair.m_pProxy1 = 0;
				previousPair.m_algorithm = 0;

				for (i = 0; i < overlappingPairArray.size(); i++)
				{
					btBroadphasePair& pair = overlappingPairArray[i];

					bool isDuplicate = (pair == previousPair);

					previousPair = pair;

					bool needsRemoval = false;

					if (!isDuplicate)
					{
						bool hasOverlap = testAabbOverlap(pair.m_pProxy0, pair.m_pProxy1);

						if (hasOverlap)
						{
							needsRemoval = false;  //callback->processOverlap(pair);
						}
						else
						{
							needsRemoval = true;
						}
					}
					else
					{
						//remove duplicate
						needsRemoval = true;
						//should have no algorithm
						btAssert(!pair.m_algorithm);
					}

					if (needsRemoval)
					{
						m_pairCache->cleanOverlappingPair(pair, dispatcher);

						//		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
						//		m_overlappingPairArray.pop_back();
						pair.m_pProxy0 = 0;
						pair.m_pProxy1 = 0;
						m_invalidPair++;
					}
				}

				// clean invalid pairs : perform a sort, to sort 'invalid' pairs to the end
				overlappingPairArray.quickSort(btBroadphasePairSortPredicate());
				overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
				m_invalidPair = 0;
			}
		}
	}

	virtual void destroyProxy(btBroadphaseProxy* proxyOrg, btDispatcher* dispatcher)
	{
		m_pairCache->removeOverlappingPairsContainingProxy(proxyOrg, dispatcher);

		btSimpleBroadphaseProxy* proxy0 = static_cast<btSimpleBroadphaseProxy*>(proxyOrg);
		freeHandle(proxy0);
	}
	virtual void setAabb(btBroadphaseProxy* proxy, const btVector3& aabbMin, const btVector3& aabbMax, btDispatcher* /*dispatcher*/)
	{
		btSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
		sbp->m_aabbMin = aabbMin;
		sbp->m_aabbMax = aabbMax;
	}
	virtual void getAabb(btBroadphaseProxy* proxy, btVector3& aabbMin, btVector3& aabbMax) const
	{
		const btSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
		aabbMin = sbp->m_aabbMin;
		aabbMax = sbp->m_aabbMax;
	}

	virtual void rayTest(const btVector3& rayFrom, const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin = btVector3(0, 0, 0), const btVector3& aabbMax = btVector3(0, 0, 0))
	{
		for (int i = 0; i <= m_LastHandleIndex; i++)
		{
			btSimpleBroadphaseProxy* proxy = &m_pHandles[i];
			if (!proxy->m_clientObject)
			{
				continue;
			}
			rayCallback.process(proxy);
		}
	}
	virtual void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback)
	{
		for (int i = 0; i <= m_LastHandleIndex; i++)
		{
			btSimpleBroadphaseProxy* proxy = &m_pHandles[i];
			if (!proxy->m_clientObject)
			{
				continue;
			}
			if (TestAabbAgainstAabb2(aabbMin, aabbMax, proxy->m_aabbMin, proxy->m_aabbMax))
			{
				callback.process(proxy);
			}
		}
	}

	btOverlappingPairCache* getOverlappingPairCache()
	{
		return m_pairCache;
	}
	const btOverlappingPairCache* getOverlappingPairCache() const
	{
		return m_pairCache;
	}

	bool testAabbOverlap(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1)
	{
		btSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
		btSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);
		return aabbOverlap(p0, p1);
	}

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(btVector3& aabbMin, btVector3& aabbMax) const
	{
		aabbMin.setValue(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
		aabbMax.setValue(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
	}

	virtual void printStats(){}
};

#endif  //BT_SIMPLE_BROADPHASE_H
