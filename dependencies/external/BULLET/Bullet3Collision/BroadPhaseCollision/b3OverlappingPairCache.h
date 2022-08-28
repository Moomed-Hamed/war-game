#ifndef B3_OVERLAPPING_PAIR_CACHE_H
#define B3_OVERLAPPING_PAIR_CACHE_H

#include "Bullet3Common/shared/b3Int2.h"
#include "Bullet3Common/b3AlignedObjectArray.h"

class b3Dispatcher;
#include "b3OverlappingPair.h"

static int b3g_overlappingPairs = 0;
static int b3g_removePairs = 0;
static int b3g_addedPairs = 0;
static int b3g_findPairs = 0;

typedef b3AlignedObjectArray<b3BroadphasePair> b3BroadphasePairArray;

struct b3OverlapCallback
{
	virtual ~b3OverlapCallback()
	{
	}
	//return true for deletion of the pair
	virtual bool processOverlap(b3BroadphasePair& pair) = 0;
};

struct b3OverlapFilterCallback
{
	virtual ~b3OverlapFilterCallback()
	{
	}
	// return true when pairs need collision
	virtual bool needBroadphaseCollision(int proxy0, int proxy1) const = 0;
};

extern int b3g_removePairs;
extern int b3g_addedPairs;
extern int b3g_findPairs;

const int B3_NULL_PAIR = 0xffffffff;

///The b3OverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the b3BroadphaseInterface broadphases.
///The b3HashedOverlappingPairCache and b3SortedOverlappingPairCache classes are two implementations.
class b3OverlappingPairCache
{
public:
	virtual ~b3OverlappingPairCache() {}  // this is needed so we can get to the derived class destructor

	virtual b3BroadphasePair* getOverlappingPairArrayPtr() = 0;

	virtual const b3BroadphasePair* getOverlappingPairArrayPtr() const = 0;

	virtual b3BroadphasePairArray& getOverlappingPairArray() = 0;

	virtual void cleanOverlappingPair(b3BroadphasePair& pair, b3Dispatcher* dispatcher) = 0;

	virtual int getNumOverlappingPairs() const = 0;

	virtual void cleanProxyFromPairs(int proxy, b3Dispatcher* dispatcher) = 0;

	virtual void setOverlapFilterCallback(b3OverlapFilterCallback* callback) = 0;

	virtual void processAllOverlappingPairs(b3OverlapCallback*, b3Dispatcher* dispatcher) = 0;

	virtual b3BroadphasePair* findPair(int proxy0, int proxy1) = 0;

	virtual bool hasDeferredRemoval() = 0;

	//virtual	void	setInternalGhostPairCallback(b3OverlappingPairCallback* ghostPairCallback)=0;

	virtual b3BroadphasePair* addOverlappingPair(int proxy0, int proxy1) = 0;
	virtual void* removeOverlappingPair(int proxy0, int proxy1, b3Dispatcher* dispatcher) = 0;
	virtual void removeOverlappingPairsContainingProxy(int /*proxy0*/, b3Dispatcher* /*dispatcher*/) = 0;

	virtual void sortOverlappingPairs(b3Dispatcher* dispatcher) = 0;
};

/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com
class b3HashedOverlappingPairCache : public b3OverlappingPairCache
{
	b3BroadphasePairArray m_overlappingPairArray;
	b3OverlapFilterCallback* m_overlapFilterCallback;
	//	bool		m_blockedForChanges;

public:
	b3HashedOverlappingPairCache() : m_overlapFilterCallback(0)
	{
		int initialAllocatedSize = 2;
		m_overlappingPairArray.reserve(initialAllocatedSize);
		growTables();
	}
	virtual ~b3HashedOverlappingPairCache() {}

	virtual void removeOverlappingPairsContainingProxy(int proxy, b3Dispatcher* dispatcher)
	{
		class RemovePairCallback : public b3OverlapCallback
		{
			int m_obsoleteProxy;

		public:
			RemovePairCallback(int obsoleteProxy)
				: m_obsoleteProxy(obsoleteProxy)
			{
			}
			virtual bool processOverlap(b3BroadphasePair& pair)
			{
				return ((pair.x == m_obsoleteProxy) ||
						(pair.y == m_obsoleteProxy));
			}
		};

		RemovePairCallback removeCallback(proxy);

		processAllOverlappingPairs(&removeCallback, dispatcher);
	}

	virtual void* removeOverlappingPair(int proxy0, int proxy1, b3Dispatcher* dispatcher)
	{
		b3g_removePairs++;
		if (proxy0 > proxy1)
			b3Swap(proxy0, proxy1);
		int proxyId1 = proxy0;
		int proxyId2 = proxy1;

		int hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity() - 1));

		b3BroadphasePair* pair = internalFindPair(proxy0, proxy1, hash);
		if (pair == NULL)
		{
			return 0;
		}

		cleanOverlappingPair(*pair, dispatcher);

		int pairIndex = int(pair - &m_overlappingPairArray[0]);
		b3Assert(pairIndex < m_overlappingPairArray.size());

		// Remove the pair from the hash table.
		int index = m_hashTable[hash];
		b3Assert(index != B3_NULL_PAIR);

		int previous = B3_NULL_PAIR;
		while (index != pairIndex)
		{
			previous = index;
			index = m_next[index];
		}

		if (previous != B3_NULL_PAIR)
		{
			b3Assert(m_next[previous] == pairIndex);
			m_next[previous] = m_next[pairIndex];
		}
		else
		{
			m_hashTable[hash] = m_next[pairIndex];
		}

		// We now move the last pair into spot of the
		// pair being removed. We need to fix the hash
		// table indices to support the move.

		int lastPairIndex = m_overlappingPairArray.size() - 1;

		// If the removed pair is the last pair, we are done.
		if (lastPairIndex == pairIndex)
		{
			m_overlappingPairArray.pop_back();
			return 0;
		}

		// Remove the last pair from the hash table.
		const b3BroadphasePair* last = &m_overlappingPairArray[lastPairIndex];
		/* missing swap here too, Nat. */
		int lastHash = static_cast<int>(getHash(static_cast<unsigned int>(last->x), static_cast<unsigned int>(last->y)) & (m_overlappingPairArray.capacity() - 1));

		index = m_hashTable[lastHash];
		b3Assert(index != B3_NULL_PAIR);

		previous = B3_NULL_PAIR;
		while (index != lastPairIndex)
		{
			previous = index;
			index = m_next[index];
		}

		if (previous != B3_NULL_PAIR)
		{
			b3Assert(m_next[previous] == lastPairIndex);
			m_next[previous] = m_next[lastPairIndex];
		}
		else
		{
			m_hashTable[lastHash] = m_next[lastPairIndex];
		}

		// Copy the last pair into the remove pair's spot.
		m_overlappingPairArray[pairIndex] = m_overlappingPairArray[lastPairIndex];

		// Insert the last pair into the hash table
		m_next[pairIndex] = m_hashTable[lastHash];
		m_hashTable[lastHash] = pairIndex;

		m_overlappingPairArray.pop_back();

		return 0;
	}

	B3_FORCE_INLINE bool needsBroadphaseCollision(int proxy0, int proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback->needBroadphaseCollision(proxy0, proxy1);

		bool collides = true;  //(proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		//collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

		return collides;
	}

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	virtual b3BroadphasePair* addOverlappingPair(int proxy0, int proxy1)
	{
		b3g_addedPairs++;

		if (!needsBroadphaseCollision(proxy0, proxy1))
			return 0;

		return internalAddPair(proxy0, proxy1);
	}

	void cleanProxyFromPairs(int proxy, b3Dispatcher* dispatcher)
	{
		class CleanPairCallback : public b3OverlapCallback
		{
			int m_cleanProxy;
			b3OverlappingPairCache* m_pairCache;
			b3Dispatcher* m_dispatcher;

		public:
			CleanPairCallback(int cleanProxy, b3OverlappingPairCache* pairCache, b3Dispatcher* dispatcher)
				: m_cleanProxy(cleanProxy),
				  m_pairCache(pairCache),
				  m_dispatcher(dispatcher)
			{
			}
			virtual bool processOverlap(b3BroadphasePair& pair)
			{
				if ((pair.x == m_cleanProxy) ||
					(pair.y == m_cleanProxy))
				{
					m_pairCache->cleanOverlappingPair(pair, m_dispatcher);
				}
				return false;
			}
		};

		CleanPairCallback cleanPairs(proxy, this, dispatcher);

		processAllOverlappingPairs(&cleanPairs, dispatcher);
	}

	virtual void processAllOverlappingPairs(b3OverlapCallback* callback, b3Dispatcher* dispatcher)
	{
		//	printf("m_overlappingPairArray.size()=%d\n",m_overlappingPairArray.size());
		for (int i = 0; i < m_overlappingPairArray.size();)
		{
			b3BroadphasePair* pair = &m_overlappingPairArray[i];
			if (callback->processOverlap(*pair))
			{
				removeOverlappingPair(pair->x, pair->y, dispatcher);

				b3g_overlappingPairs--;
			}
			else
			{
				i++;
			}
		}
	}

	virtual b3BroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	const b3BroadphasePair* getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}

	b3BroadphasePairArray& getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	const b3BroadphasePairArray& getOverlappingPairArray() const
	{
		return m_overlappingPairArray;
	}

	void cleanOverlappingPair(b3BroadphasePair& pair, b3Dispatcher* dispatcher) {}

	b3BroadphasePair* findPair(int proxy0, int proxy1)
	{
		b3g_findPairs++;
		if (proxy0 > proxy1)
			b3Swap(proxy0, proxy1);
		int proxyId1 = proxy0;
		int proxyId2 = proxy1;

		int hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity() - 1));

		if (hash >= m_hashTable.size())
		{
			return NULL;
		}

		int index = m_hashTable[hash];
		while (index != B3_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
		{
			index = m_next[index];
		}

		if (index == B3_NULL_PAIR)
		{
			return NULL;
		}

		b3Assert(index < m_overlappingPairArray.size());

		return &m_overlappingPairArray[index];
	}

	int GetCount() const { return m_overlappingPairArray.size(); }
	//	b3BroadphasePair* GetPairs() { return m_pairs; }

	b3OverlapFilterCallback* getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	void setOverlapFilterCallback(b3OverlapFilterCallback* callback)
	{
		m_overlapFilterCallback = callback;
	}

	int getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}

private:
	b3BroadphasePair* internalAddPair(int proxy0, int proxy1)
	{
		if (proxy0 > proxy1)
			b3Swap(proxy0, proxy1);
		int proxyId1 = proxy0;
		int proxyId2 = proxy1;

		int hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity() - 1));  // New hash value with new mask

		b3BroadphasePair* pair = internalFindPair(proxy0, proxy1, hash);
		if (pair != NULL)
		{
			return pair;
		}

		int count = m_overlappingPairArray.size();
		int oldCapacity = m_overlappingPairArray.capacity();
		pair = &m_overlappingPairArray.expandNonInitializing();

		int newCapacity = m_overlappingPairArray.capacity();

		if (oldCapacity < newCapacity)
		{
			growTables();
			//hash with new capacity
			hash = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity() - 1));
		}

		*pair = b3MakeBroadphasePair(proxy0, proxy1);

		m_next[count] = m_hashTable[hash];
		m_hashTable[hash] = count;

		return pair;
	}

	void growTables()
	{
		int newCapacity = m_overlappingPairArray.capacity();

		if (m_hashTable.size() < newCapacity)
		{
			//grow hashtable and next table
			int curHashtableSize = m_hashTable.size();

			m_hashTable.resize(newCapacity);
			m_next.resize(newCapacity);

			int i;

			for (i = 0; i < newCapacity; ++i)
			{
				m_hashTable[i] = B3_NULL_PAIR;
			}
			for (i = 0; i < newCapacity; ++i)
			{
				m_next[i] = B3_NULL_PAIR;
			}

			for (i = 0; i < curHashtableSize; i++)
			{
				const b3BroadphasePair& pair = m_overlappingPairArray[i];
				int proxyId1 = pair.x;
				int proxyId2 = pair.y;

				int hashValue = static_cast<int>(getHash(static_cast<unsigned int>(proxyId1), static_cast<unsigned int>(proxyId2)) & (m_overlappingPairArray.capacity() - 1));  // New hash value with new mask
				m_next[i] = m_hashTable[hashValue];
				m_hashTable[hashValue] = i;
			}
		}
	}

	B3_FORCE_INLINE bool equalsPair(const b3BroadphasePair& pair, int proxyId1, int proxyId2)
	{
		return pair.x == proxyId1 && pair.y == proxyId2;
	}

	/*
	// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
	// This assumes proxyId1 and proxyId2 are 16-bit.
	B3_FORCE_INLINE int getHash(int proxyId1, int proxyId2)
	{
		int key = (proxyId2 << 16) | proxyId1;
		key = ~key + (key << 15);
		key = key ^ (key >> 12);
		key = key + (key << 2);
		key = key ^ (key >> 4);
		key = key * 2057;
		key = key ^ (key >> 16);
		return key;
	}
	*/

	B3_FORCE_INLINE unsigned int getHash(unsigned int proxyId1, unsigned int proxyId2)
	{
		int key = static_cast<int>(((unsigned int)proxyId1) | (((unsigned int)proxyId2) << 16));
		// Thomas Wang's hash

		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return static_cast<unsigned int>(key);
	}

	B3_FORCE_INLINE b3BroadphasePair* internalFindPair(int proxy0, int proxy1, int hash)
	{
		int proxyId1 = proxy0;
		int proxyId2 = proxy1;

		int index = m_hashTable[hash];

		while (index != B3_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
		{
			index = m_next[index];
		}

		if (index == B3_NULL_PAIR)
		{
			return NULL;
		}

		b3Assert(index < m_overlappingPairArray.size());

		return &m_overlappingPairArray[index];
	}

	virtual bool hasDeferredRemoval()
	{
		return false;
	}

	/*	virtual	void	setInternalGhostPairCallback(b3OverlappingPairCallback* ghostPairCallback)
	{
		m_ghostPairCallback = ghostPairCallback;
	}
	*/

	virtual void sortOverlappingPairs(b3Dispatcher* dispatcher)
	{
		///need to keep hashmap in sync with pair address, so rebuild all
		b3BroadphasePairArray tmpPairs;
		int i;
		for (i = 0; i < m_overlappingPairArray.size(); i++)
		{
			tmpPairs.push_back(m_overlappingPairArray[i]);
		}

		for (i = 0; i < tmpPairs.size(); i++)
		{
			removeOverlappingPair(tmpPairs[i].x, tmpPairs[i].y, dispatcher);
		}

		for (i = 0; i < m_next.size(); i++)
		{
			m_next[i] = B3_NULL_PAIR;
		}

		tmpPairs.quickSort(b3BroadphasePairSortPredicate());

		for (i = 0; i < tmpPairs.size(); i++)
		{
			addOverlappingPair(tmpPairs[i].x, tmpPairs[i].y);
		}
	}

protected:
	b3AlignedObjectArray<int> m_hashTable;
	b3AlignedObjectArray<int> m_next;
};

///b3SortedOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or b3SimpleBroadphase
class b3SortedOverlappingPairCache : public b3OverlappingPairCache
{
protected:
	//avoid brute-force finding all the time
	b3BroadphasePairArray m_overlappingPairArray;

	//during the dispatch, check that user doesn't destroy/create proxy
	bool m_blockedForChanges;

	///by default, do the removal during the pair traversal
	bool m_hasDeferredRemoval;

	//if set, use the callback instead of the built in filter in needBroadphaseCollision
	b3OverlapFilterCallback* m_overlapFilterCallback;

	//		b3OverlappingPairCallback*	m_ghostPairCallback;

public:
	b3SortedOverlappingPairCache() : m_blockedForChanges(false), m_hasDeferredRemoval(true), m_overlapFilterCallback(0)
	{
		int initialAllocatedSize = 2;
		m_overlappingPairArray.reserve(initialAllocatedSize);
	}
	virtual ~b3SortedOverlappingPairCache() {}

	virtual void processAllOverlappingPairs(b3OverlapCallback* callback, b3Dispatcher* dispatcher)
	{
		int i;

		for (i = 0; i < m_overlappingPairArray.size();)
		{
			b3BroadphasePair* pair = &m_overlappingPairArray[i];
			if (callback->processOverlap(*pair))
			{
				cleanOverlappingPair(*pair, dispatcher);
				pair->x = -1;
				pair->y = -1;
				m_overlappingPairArray.swap(i, m_overlappingPairArray.size() - 1);
				m_overlappingPairArray.pop_back();
				b3g_overlappingPairs--;
			}
			else
			{
				i++;
			}
		}
	}

	void* removeOverlappingPair(int proxy0, int proxy1, b3Dispatcher* dispatcher)
	{
		if (!hasDeferredRemoval())
		{
			b3BroadphasePair findPair = b3MakeBroadphasePair(proxy0, proxy1);

			int findIndex = m_overlappingPairArray.findLinearSearch(findPair);
			if (findIndex < m_overlappingPairArray.size())
			{
				b3g_overlappingPairs--;
				b3BroadphasePair& pair = m_overlappingPairArray[findIndex];

				cleanOverlappingPair(pair, dispatcher);
				//if (m_ghostPairCallback)
				//	m_ghostPairCallback->removeOverlappingPair(proxy0, proxy1,dispatcher);

				m_overlappingPairArray.swap(findIndex, m_overlappingPairArray.capacity() - 1);
				m_overlappingPairArray.pop_back();
				return 0;
			}
		}

		return 0;
	}

	void cleanOverlappingPair(b3BroadphasePair& pair, b3Dispatcher* dispatcher) {}

	b3BroadphasePair* addOverlappingPair(int proxy0, int proxy1)
	{
		//don't add overlap with own
		b3Assert(proxy0 != proxy1);

		if (!needsBroadphaseCollision(proxy0, proxy1))
			return 0;

		b3BroadphasePair* pair = &m_overlappingPairArray.expandNonInitializing();
		*pair = b3MakeBroadphasePair(proxy0, proxy1);

		b3g_overlappingPairs++;
		b3g_addedPairs++;

		//	if (m_ghostPairCallback)
		//		m_ghostPairCallback->addOverlappingPair(proxy0, proxy1);
		return pair;
	}

	///this findPair becomes really slow. Either sort the list to speedup the query, or
	///use a different solution. It is mainly used for Removing overlapping pairs. Removal could be delayed.
	///we could keep a linked list in each proxy, and store pair in one of the proxies (with lowest memory address)
	///Also we can use a 2D bitmap, which can be useful for a future GPU implementation
	b3BroadphasePair* findPair(int proxy0, int proxy1)
	{
		if (!needsBroadphaseCollision(proxy0, proxy1))
			return 0;

		b3BroadphasePair tmpPair = b3MakeBroadphasePair(proxy0, proxy1);
		int findIndex = m_overlappingPairArray.findLinearSearch(tmpPair);

		if (findIndex < m_overlappingPairArray.size())
		{
			//b3Assert(it != m_overlappingPairSet.end());
			b3BroadphasePair* pair = &m_overlappingPairArray[findIndex];
			return pair;
		}
		return 0;
	}

	void cleanProxyFromPairs(int proxy, b3Dispatcher* dispatcher)
	{
		class CleanPairCallback : public b3OverlapCallback
		{
			int m_cleanProxy;
			b3OverlappingPairCache* m_pairCache;
			b3Dispatcher* m_dispatcher;

		public:
			CleanPairCallback(int cleanProxy, b3OverlappingPairCache* pairCache, b3Dispatcher* dispatcher)
				: m_cleanProxy(cleanProxy),
				  m_pairCache(pairCache),
				  m_dispatcher(dispatcher)
			{
			}
			virtual bool processOverlap(b3BroadphasePair& pair)
			{
				if ((pair.x == m_cleanProxy) ||
					(pair.y == m_cleanProxy))
				{
					m_pairCache->cleanOverlappingPair(pair, m_dispatcher);
				}
				return false;
			}
		};

		CleanPairCallback cleanPairs(proxy, this, dispatcher);

		processAllOverlappingPairs(&cleanPairs, dispatcher);
	}

	virtual void removeOverlappingPairsContainingProxy(int proxy, b3Dispatcher* dispatcher)
	{
		class RemovePairCallback : public b3OverlapCallback
		{
			int m_obsoleteProxy;

		public:
			RemovePairCallback(int obsoleteProxy)
				: m_obsoleteProxy(obsoleteProxy)
			{
			}
			virtual bool processOverlap(b3BroadphasePair& pair)
			{
				return ((pair.x == m_obsoleteProxy) || (pair.y == m_obsoleteProxy));
			}
		};

		RemovePairCallback removeCallback(proxy);

		processAllOverlappingPairs(&removeCallback, dispatcher);
	}

	inline bool needsBroadphaseCollision(int proxy0, int proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback->needBroadphaseCollision(proxy0, proxy1);

		bool collides = true;  //(proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		//collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

		return collides;
	}

	b3BroadphasePairArray& getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	const b3BroadphasePairArray& getOverlappingPairArray() const
	{
		return m_overlappingPairArray;
	}

	b3BroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	const b3BroadphasePair* getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}

	int getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}

	b3OverlapFilterCallback* getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	void setOverlapFilterCallback(b3OverlapFilterCallback* callback)
	{
		m_overlapFilterCallback = callback;
	}

	virtual bool hasDeferredRemoval()
	{
		return m_hasDeferredRemoval;
	}

	/*		virtual	void	setInternalGhostPairCallback(b3OverlappingPairCallback* ghostPairCallback)
		{
			m_ghostPairCallback = ghostPairCallback;
		}
		*/
	virtual void sortOverlappingPairs(b3Dispatcher* dispatcher) {} // should already be sorted
};

///b3NullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.
class b3NullPairCache : public b3OverlappingPairCache
{
	b3BroadphasePairArray m_overlappingPairArray;

public:
	virtual b3BroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}
	const b3BroadphasePair* getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}
	b3BroadphasePairArray& getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	virtual void cleanOverlappingPair(b3BroadphasePair& /*pair*/, b3Dispatcher* /*dispatcher*/)
	{
	}

	virtual int getNumOverlappingPairs() const
	{
		return 0;
	}

	virtual void cleanProxyFromPairs(int /*proxy*/, b3Dispatcher* /*dispatcher*/)
	{
	}

	virtual void setOverlapFilterCallback(b3OverlapFilterCallback* /*callback*/)
	{
	}

	virtual void processAllOverlappingPairs(b3OverlapCallback*, b3Dispatcher* /*dispatcher*/)
	{
	}

	virtual b3BroadphasePair* findPair(int /*proxy0*/, int /*proxy1*/)
	{
		return 0;
	}

	virtual bool hasDeferredRemoval()
	{
		return true;
	}

	//	virtual	void	setInternalGhostPairCallback(b3OverlappingPairCallback* /* ghostPairCallback */)
	//	{
	//
	//	}

	virtual b3BroadphasePair* addOverlappingPair(int /*proxy0*/, int /*proxy1*/)
	{
		return 0;
	}

	virtual void* removeOverlappingPair(int /*proxy0*/, int /*proxy1*/, b3Dispatcher* /*dispatcher*/)
	{
		return 0;
	}

	virtual void removeOverlappingPairsContainingProxy(int /*proxy0*/, b3Dispatcher* /*dispatcher*/)
	{
	}

	virtual void sortOverlappingPairs(b3Dispatcher* dispatcher)
	{
		(void)dispatcher;
	}
};

#endif  //B3_OVERLAPPING_PAIR_CACHE_H
