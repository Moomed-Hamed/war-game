// b3DynamicBvhBroadphase implementation by Nathanael Presson

#include "b3DynamicBvhBroadphase.h"
#include "b3OverlappingPair.h"

//
// Profiling
//

#define b3SPC(_value_)

//
// Helpers
//

//
template <typename T>
static inline void b3ListAppend(T* item, T*& list)
{
	item->links[0] = 0;
	item->links[1] = list;
	if (list) list->links[0] = item;
	list = item;
}

//
template <typename T>
static inline void b3ListRemove(T* item, T*& list)
{
	if (item->links[0])
		item->links[0]->links[1] = item->links[1];
	else
		list = item->links[1];
	if (item->links[1]) item->links[1]->links[0] = item->links[0];
}

//
template <typename T>
static inline int b3ListCount(T* root)
{
	int n = 0;
	while (root)
	{
		++n;
		root = root->links[1];
	}
	return (n);
}

//
template <typename T>
static inline void b3Clear(T& value)
{
	static const struct ZeroDummy : T
	{
	} zerodummy;
	value = zerodummy;
}

//
// Colliders
//

/* Tree collider	*/
struct b3DbvtTreeCollider : b3DynamicBvh::ICollide
{
	b3DynamicBvhBroadphase* pbp;
	b3DbvtProxy* proxy;
	b3DbvtTreeCollider(b3DynamicBvhBroadphase* p) : pbp(p) {}
	void Process(const b3DbvtNode* na, const b3DbvtNode* nb)
	{
		if (na != nb)
		{
			b3DbvtProxy* pa = (b3DbvtProxy*)na->data;
			b3DbvtProxy* pb = (b3DbvtProxy*)nb->data;
#if B3_DBVT_BP_SORTPAIRS
			if (pa->m_uniqueId > pb->m_uniqueId)
				b3Swap(pa, pb);
#endif
			pbp->m_paircache->addOverlappingPair(pa->getUid(), pb->getUid());
			++pbp->m_newpairs;
		}
	}
	void Process(const b3DbvtNode* n)
	{
		Process(n, proxy->leaf);
	}
};

//
// b3DynamicBvhBroadphase
//

//
b3DynamicBvhBroadphase::b3DynamicBvhBroadphase(int proxyCapacity, b3OverlappingPairCache* paircache)
{
	m_deferedcollide = false;
	m_needcleanup = true;
	m_releasepaircache = (paircache != 0) ? false : true;
	m_prediction = 0;
	m_stageCurrent = 0;
	m_fixedleft = 0;
	m_fupdates = 1;
	m_dupdates = 0;
	m_cupdates = 10;
	m_newpairs = 1;
	m_updates_call = 0;
	m_updates_done = 0;
	m_updates_ratio = 0;
	m_paircache = paircache ? paircache : new (b3AlignedAlloc(sizeof(b3HashedOverlappingPairCache), 16)) b3HashedOverlappingPairCache();

	m_pid = 0;
	m_cid = 0;
	for (int i = 0; i <= STAGECOUNT; ++i)
	{
		m_stageRoots[i] = 0;
	}

	m_proxies.resize(proxyCapacity);
}

//
b3DynamicBvhBroadphase::~b3DynamicBvhBroadphase()
{
	if (m_releasepaircache)
	{
		m_paircache->~b3OverlappingPairCache();
		b3AlignedFree(m_paircache);
	}
}

//
b3BroadphaseProxy* b3DynamicBvhBroadphase::createProxy(const b3Vector3& aabbMin,
													   const b3Vector3& aabbMax,
													   int objectId,
													   void* userPtr,
													   int collisionFilterGroup,
													   int collisionFilterMask)
{
	b3DbvtProxy* mem = &m_proxies[objectId];
	b3DbvtProxy* proxy = new (mem) b3DbvtProxy(aabbMin, aabbMax, userPtr,
											   collisionFilterGroup,
											   collisionFilterMask);

	b3DbvtAabbMm aabb = b3DbvtVolume::FromMM(aabbMin, aabbMax);

	//bproxy->aabb			=	b3DbvtVolume::FromMM(aabbMin,aabbMax);
	proxy->stage = m_stageCurrent;
	proxy->m_uniqueId = objectId;
	proxy->leaf = m_sets[0].insert(aabb, proxy);
	b3ListAppend(proxy, m_stageRoots[m_stageCurrent]);
	if (!m_deferedcollide)
	{
		b3DbvtTreeCollider collider(this);
		collider.proxy = proxy;
		m_sets[0].collideTV(m_sets[0].m_root, aabb, collider);
		m_sets[1].collideTV(m_sets[1].m_root, aabb, collider);
	}
	return (proxy);
}

//
void b3DynamicBvhBroadphase::destroyProxy(b3BroadphaseProxy* absproxy,
										  b3Dispatcher* dispatcher)
{
	b3DbvtProxy* proxy = (b3DbvtProxy*)absproxy;
	if (proxy->stage == STAGECOUNT)
		m_sets[1].remove(proxy->leaf);
	else
		m_sets[0].remove(proxy->leaf);
	b3ListRemove(proxy, m_stageRoots[proxy->stage]);
	m_paircache->removeOverlappingPairsContainingProxy(proxy->getUid(), dispatcher);

	m_needcleanup = true;
}

void b3DynamicBvhBroadphase::getAabb(int objectId, b3Vector3& aabbMin, b3Vector3& aabbMax) const
{
	const b3DbvtProxy* proxy = &m_proxies[objectId];
	aabbMin = proxy->m_aabbMin;
	aabbMax = proxy->m_aabbMax;
}

struct BroadphaseRayTester : b3DynamicBvh::ICollide
{
	b3BroadphaseRayCallback& m_rayCallback;
	BroadphaseRayTester(b3BroadphaseRayCallback& orgCallback)
		: m_rayCallback(orgCallback)
	{
	}
	void Process(const b3DbvtNode* leaf)
	{
		b3DbvtProxy* proxy = (b3DbvtProxy*)leaf->data;
		m_rayCallback.process(proxy);
	}
};

void b3DynamicBvhBroadphase::rayTest(const b3Vector3& rayFrom, const b3Vector3& rayTo, b3BroadphaseRayCallback& rayCallback, const b3Vector3& aabbMin, const b3Vector3& aabbMax)
{
	BroadphaseRayTester callback(rayCallback);

	m_sets[0].rayTestInternal(m_sets[0].m_root,
							  rayFrom,
							  rayTo,
							  rayCallback.m_rayDirectionInverse,
							  rayCallback.m_signs,
							  rayCallback.m_lambda_max,
							  aabbMin,
							  aabbMax,
							  callback);

	m_sets[1].rayTestInternal(m_sets[1].m_root,
							  rayFrom,
							  rayTo,
							  rayCallback.m_rayDirectionInverse,
							  rayCallback.m_signs,
							  rayCallback.m_lambda_max,
							  aabbMin,
							  aabbMax,
							  callback);
}

struct BroadphaseAabbTester : b3DynamicBvh::ICollide
{
	b3BroadphaseAabbCallback& m_aabbCallback;
	BroadphaseAabbTester(b3BroadphaseAabbCallback& orgCallback)
		: m_aabbCallback(orgCallback)
	{
	}
	void Process(const b3DbvtNode* leaf)
	{
		b3DbvtProxy* proxy = (b3DbvtProxy*)leaf->data;
		m_aabbCallback.process(proxy);
	}
};

void b3DynamicBvhBroadphase::aabbTest(const b3Vector3& aabbMin, const b3Vector3& aabbMax, b3BroadphaseAabbCallback& aabbCallback)
{
	BroadphaseAabbTester callback(aabbCallback);

	const B3_ATTRIBUTE_ALIGNED16(b3DbvtVolume) bounds = b3DbvtVolume::FromMM(aabbMin, aabbMax);
	//process all children, that overlap with  the given AABB bounds
	m_sets[0].collideTV(m_sets[0].m_root, bounds, callback);
	m_sets[1].collideTV(m_sets[1].m_root, bounds, callback);
}

//
void b3DynamicBvhBroadphase::setAabb(int objectId,
									 const b3Vector3& aabbMin,
									 const b3Vector3& aabbMax,
									 b3Dispatcher* /*dispatcher*/)
{
	b3DbvtProxy* proxy = &m_proxies[objectId];
	//	b3DbvtProxy*						proxy=(b3DbvtProxy*)absproxy;
	B3_ATTRIBUTE_ALIGNED16(b3DbvtVolume)
	aabb = b3DbvtVolume::FromMM(aabbMin, aabbMax);

	{
		bool docollide = false;
		if (proxy->stage == STAGECOUNT)
		{ /* fixed -> dynamic set	*/
			m_sets[1].remove(proxy->leaf);
			proxy->leaf = m_sets[0].insert(aabb, proxy);
			docollide = true;
		}
		else
		{ /* dynamic set */
			++m_updates_call;
			if (b3Intersect(proxy->leaf->volume, aabb))
			{ /* Moving */

				const b3Vector3 delta = aabbMin - proxy->m_aabbMin;
				b3Vector3 velocity(((proxy->m_aabbMax - proxy->m_aabbMin) / 2) * m_prediction);
				if (delta[0] < 0) velocity[0] = -velocity[0];
				if (delta[1] < 0) velocity[1] = -velocity[1];
				if (delta[2] < 0) velocity[2] = -velocity[2];
				if (m_sets[0].update(proxy->leaf, aabb, velocity, B3_DBVT_BP_MARGIN))
				{
					++m_updates_done;
					docollide = true;
				}
			}
			else
			{ /* Teleporting			*/
				m_sets[0].update(proxy->leaf, aabb);
				++m_updates_done;
				docollide = true;
			}
		}
		b3ListRemove(proxy, m_stageRoots[proxy->stage]);
		proxy->m_aabbMin = aabbMin;
		proxy->m_aabbMax = aabbMax;
		proxy->stage = m_stageCurrent;
		b3ListAppend(proxy, m_stageRoots[m_stageCurrent]);
		if (docollide)
		{
			m_needcleanup = true;
			if (!m_deferedcollide)
			{
				b3DbvtTreeCollider collider(this);
				m_sets[1].collideTTpersistentStack(m_sets[1].m_root, proxy->leaf, collider);
				m_sets[0].collideTTpersistentStack(m_sets[0].m_root, proxy->leaf, collider);
			}
		}
	}
}

//
void b3DynamicBvhBroadphase::setAabbForceUpdate(b3BroadphaseProxy* absproxy,
												const b3Vector3& aabbMin,
												const b3Vector3& aabbMax,
												b3Dispatcher* /*dispatcher*/)
{
	b3DbvtProxy* proxy = (b3DbvtProxy*)absproxy;
	B3_ATTRIBUTE_ALIGNED16(b3DbvtVolume)
	aabb = b3DbvtVolume::FromMM(aabbMin, aabbMax);
	bool docollide = false;
	if (proxy->stage == STAGECOUNT)
	{ /* fixed -> dynamic set	*/
		m_sets[1].remove(proxy->leaf);
		proxy->leaf = m_sets[0].insert(aabb, proxy);
		docollide = true;
	}
	else
	{ /* dynamic set */
		++m_updates_call;
		/* Teleporting */
		m_sets[0].update(proxy->leaf, aabb);
		++m_updates_done;
		docollide = true;
	}
	b3ListRemove(proxy, m_stageRoots[proxy->stage]);
	proxy->m_aabbMin = aabbMin;
	proxy->m_aabbMax = aabbMax;
	proxy->stage = m_stageCurrent;
	b3ListAppend(proxy, m_stageRoots[m_stageCurrent]);
	if (docollide)
	{
		m_needcleanup = true;
		if (!m_deferedcollide)
		{
			b3DbvtTreeCollider collider(this);
			m_sets[1].collideTTpersistentStack(m_sets[1].m_root, proxy->leaf, collider);
			m_sets[0].collideTTpersistentStack(m_sets[0].m_root, proxy->leaf, collider);
		}
	}
}

//
void b3DynamicBvhBroadphase::calculateOverlappingPairs(b3Dispatcher* dispatcher)
{
	collide(dispatcher);
	performDeferredRemoval(dispatcher);
}

void b3DynamicBvhBroadphase::performDeferredRemoval(b3Dispatcher* dispatcher)
{
	if (m_paircache->hasDeferredRemoval())
	{
		b3BroadphasePairArray& overlappingPairArray = m_paircache->getOverlappingPairArray();

		//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
		overlappingPairArray.quickSort(b3BroadphasePairSortPredicate());

		int invalidPair = 0;

		int i;

		b3BroadphasePair previousPair = b3MakeBroadphasePair(-1, -1);

		for (i = 0; i < overlappingPairArray.size(); i++)
		{
			b3BroadphasePair& pair = overlappingPairArray[i];

			bool isDuplicate = (pair == previousPair);

			previousPair = pair;

			bool needsRemoval = false;

			if (!isDuplicate)
			{
				//important to perform AABB check that is consistent with the broadphase
				b3DbvtProxy* pa = &m_proxies[pair.x];
				b3DbvtProxy* pb = &m_proxies[pair.y];
				bool hasOverlap = b3Intersect(pa->leaf->volume, pb->leaf->volume);

				if (hasOverlap)
				{
					needsRemoval = false;
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
			}

			if (needsRemoval)
			{
				m_paircache->cleanOverlappingPair(pair, dispatcher);

				pair.x = -1;
				pair.y = -1;
				invalidPair++;
			}
		}

		//perform a sort, to sort 'invalid' pairs to the end
		overlappingPairArray.quickSort(b3BroadphasePairSortPredicate());
		overlappingPairArray.resize(overlappingPairArray.size() - invalidPair);
	}
}

//
void b3DynamicBvhBroadphase::collide(b3Dispatcher* dispatcher)
{
	/*printf("---------------------------------------------------------\n");
	printf("m_sets[0].m_leaves=%d\n",m_sets[0].m_leaves);
	printf("m_sets[1].m_leaves=%d\n",m_sets[1].m_leaves);
	printf("numPairs = %d\n",getOverlappingPairCache()->getNumOverlappingPairs());
	{
		int i;
		for (i=0;i<getOverlappingPairCache()->getNumOverlappingPairs();i++)
		{
			printf("pair[%d]=(%d,%d),",i,getOverlappingPairCache()->getOverlappingPairArray()[i].m_pProxy0->getUid(),
				getOverlappingPairCache()->getOverlappingPairArray()[i].m_pProxy1->getUid());
		}
		printf("\n");
	}
*/

	b3SPC(m_profiling.m_total);
	/* optimize				*/
	m_sets[0].optimizeIncremental(1 + (m_sets[0].m_leaves * m_dupdates) / 100);
	if (m_fixedleft)
	{
		const int count = 1 + (m_sets[1].m_leaves * m_fupdates) / 100;
		m_sets[1].optimizeIncremental(1 + (m_sets[1].m_leaves * m_fupdates) / 100);
		m_fixedleft = b3Max<int>(0, m_fixedleft - count);
	}
	/* dynamic -> fixed set	*/
	m_stageCurrent = (m_stageCurrent + 1) % STAGECOUNT;
	b3DbvtProxy* current = m_stageRoots[m_stageCurrent];
	if (current)
	{
		b3DbvtTreeCollider collider(this);
		do
		{
			b3DbvtProxy* next = current->links[1];
			b3ListRemove(current, m_stageRoots[current->stage]);
			b3ListAppend(current, m_stageRoots[STAGECOUNT]);

			m_sets[0].remove(current->leaf);
			B3_ATTRIBUTE_ALIGNED16(b3DbvtVolume)
			curAabb = b3DbvtVolume::FromMM(current->m_aabbMin, current->m_aabbMax);
			current->leaf = m_sets[1].insert(curAabb, current);
			current->stage = STAGECOUNT;
			current = next;
		} while (current);
		m_fixedleft = m_sets[1].m_leaves;
		m_needcleanup = true;
	}
	/* collide dynamics		*/
	{
		b3DbvtTreeCollider collider(this);
		if (m_deferedcollide)
		{
			b3SPC(m_profiling.m_fdcollide);
			m_sets[0].collideTTpersistentStack(m_sets[0].m_root, m_sets[1].m_root, collider);
		}
		if (m_deferedcollide)
		{
			b3SPC(m_profiling.m_ddcollide);
			m_sets[0].collideTTpersistentStack(m_sets[0].m_root, m_sets[0].m_root, collider);
		}
	}
	/* clean up				*/
	if (m_needcleanup)
	{
		b3SPC(m_profiling.m_cleanup);
		b3BroadphasePairArray& pairs = m_paircache->getOverlappingPairArray();
		if (pairs.size() > 0)
		{
			int ni = b3Min(pairs.size(), b3Max<int>(m_newpairs, (pairs.size() * m_cupdates) / 100));
			for (int i = 0; i < ni; ++i)
			{
				b3BroadphasePair& p = pairs[(m_cid + i) % pairs.size()];
				b3DbvtProxy* pa = &m_proxies[p.x];
				b3DbvtProxy* pb = &m_proxies[p.y];
				if (!b3Intersect(pa->leaf->volume, pb->leaf->volume))
				{
					m_paircache->removeOverlappingPair(pa->getUid(), pb->getUid(), dispatcher);
					--ni;
					--i;
				}
			}
			if (pairs.size() > 0)
				m_cid = (m_cid + ni) % pairs.size();
			else
				m_cid = 0;
		}
	}
	++m_pid;
	m_newpairs = 1;
	m_needcleanup = false;
	if (m_updates_call > 0)
	{
		m_updates_ratio = m_updates_done / (b3Scalar)m_updates_call;
	}
	else
	{
		m_updates_ratio = 0;
	}
	m_updates_done /= 2;
	m_updates_call /= 2;
}

//
void b3DynamicBvhBroadphase::optimize()
{
	m_sets[0].optimizeTopDown();
	m_sets[1].optimizeTopDown();
}

//
b3OverlappingPairCache* b3DynamicBvhBroadphase::getOverlappingPairCache()
{
	return (m_paircache);
}

//
const b3OverlappingPairCache* b3DynamicBvhBroadphase::getOverlappingPairCache() const
{
	return (m_paircache);
}

//
void b3DynamicBvhBroadphase::getBroadphaseAabb(b3Vector3& aabbMin, b3Vector3& aabbMax) const
{
	B3_ATTRIBUTE_ALIGNED16(b3DbvtVolume)
	bounds;

	if (!m_sets[0].empty())
		if (!m_sets[1].empty())
			b3Merge(m_sets[0].m_root->volume,
					m_sets[1].m_root->volume, bounds);
		else
			bounds = m_sets[0].m_root->volume;
	else if (!m_sets[1].empty())
		bounds = m_sets[1].m_root->volume;
	else
		bounds = b3DbvtVolume::FromCR(b3MakeVector3(0, 0, 0), 0);
	aabbMin = bounds.Mins();
	aabbMax = bounds.Maxs();
}

void b3DynamicBvhBroadphase::resetPool(b3Dispatcher* dispatcher)
{
	int totalObjects = m_sets[0].m_leaves + m_sets[1].m_leaves;
	if (!totalObjects)
	{
		//reset internal dynamic tree data structures
		m_sets[0].clear();
		m_sets[1].clear();

		m_deferedcollide = false;
		m_needcleanup = true;
		m_stageCurrent = 0;
		m_fixedleft = 0;
		m_fupdates = 1;
		m_dupdates = 0;
		m_cupdates = 10;
		m_newpairs = 1;
		m_updates_call = 0;
		m_updates_done = 0;
		m_updates_ratio = 0;

		m_pid = 0;
		m_cid = 0;
		for (int i = 0; i <= STAGECOUNT; ++i)
		{
			m_stageRoots[i] = 0;
		}
	}
}

//
void b3DynamicBvhBroadphase::printStats()
{
}