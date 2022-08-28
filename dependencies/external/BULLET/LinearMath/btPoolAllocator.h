#ifndef _BT_POOL_ALLOCATOR_H
#define _BT_POOL_ALLOCATOR_H

#include "btScalar.h"
#include "btAlignedAllocator.h"
#include "btThreads.h"

///The btPoolAllocator class allows to efficiently allocate a large pool of objects, instead of dynamically allocating them separately.
class btPoolAllocator
{
	int m_elemSize;
	int m_maxElements;
	int m_freeCount;
	void* m_firstFree;
	unsigned char* m_pool;
	btSpinMutex m_mutex;  // only used if BT_THREADSAFE

public:
	btPoolAllocator(int elemSize, int maxElements)
		: m_elemSize(elemSize),
		  m_maxElements(maxElements)
	{
		m_pool = (unsigned char*)btAlignedAlloc(static_cast<unsigned int>(m_elemSize * m_maxElements), 16);

		unsigned char* p = m_pool;
		m_firstFree = p;
		m_freeCount = m_maxElements;
		int count = m_maxElements;
		while (--count)
		{
			*(void**)p = (p + m_elemSize);
			p += m_elemSize;
		}
		*(void**)p = 0;
	}

	~btPoolAllocator()
	{
		btAlignedFree(m_pool);
	}

	int getFreeCount() const
	{
		return m_freeCount;
	}

	int getUsedCount() const
	{
		return m_maxElements - m_freeCount;
	}

	int getMaxCount() const
	{
		return m_maxElements;
	}

	void* allocate(int size)
	{
		// release mode fix
		(void)size;
		btMutexLock(&m_mutex);
		btAssert(!size || size <= m_elemSize);
		//btAssert(m_freeCount>0);  // should return null if all full
		void* result = m_firstFree;
		if (NULL != m_firstFree)
		{
			m_firstFree = *(void**)m_firstFree;
			--m_freeCount;
		}
		btMutexUnlock(&m_mutex);
		return result;
	}

	bool validPtr(void* ptr)
	{
		if (ptr)
		{
			if (((unsigned char*)ptr >= m_pool && (unsigned char*)ptr < m_pool + m_maxElements * m_elemSize))
			{
				return true;
			}
		}
		return false;
	}

	void freeMemory(void* ptr)
	{
		if (ptr)
		{
			btAssert((unsigned char*)ptr >= m_pool && (unsigned char*)ptr < m_pool + m_maxElements * m_elemSize);

			btMutexLock(&m_mutex);
			*(void**)ptr = m_firstFree;
			m_firstFree = ptr;
			++m_freeCount;
			btMutexUnlock(&m_mutex);
		}
	}

	int getElementSize() const
	{
		return m_elemSize;
	}

	unsigned char* getPoolAddress()
	{
		return m_pool;
	}

	const unsigned char* getPoolAddress() const
	{
		return m_pool;
	}
};

#endif  //_BT_POOL_ALLOCATOR_H
