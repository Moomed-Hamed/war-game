/*
StackAlloc extracted from GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

#ifndef BT_STACK_ALLOC
#define BT_STACK_ALLOC

#include "btScalar.h"  //for btAssert
#include "btAlignedAllocator.h"

///The btBlock class is an internal structure for the btStackAlloc memory allocator.
struct btBlock
{
	btBlock* previous;
	unsigned char* address;
};

///The StackAlloc class provides some fast stack-based memory allocator (LIFO last-in first-out)
class btStackAlloc
{
public:
	btStackAlloc(unsigned int size)
	{
		ctor();
		create(size);
	}
	~btStackAlloc() { destroy(); }

	inline void create(unsigned int size)
	{
		destroy();
		data = (unsigned char*)btAlignedAlloc(size, 16);
		totalsize = size;
	}
	inline void destroy()
	{
		btAssert(usedsize == 0);
		//Raise(L"StackAlloc is still in use");

		if (usedsize == 0)
		{
			if (!ischild && data)
				btAlignedFree(data);

			data = 0;
			usedsize = 0;
		}
	}

	int getAvailableMemory() const
	{
		return static_cast<int>(totalsize - usedsize);
	}

	unsigned char* allocate(unsigned int size)
	{
		const unsigned int nus(usedsize + size);
		if (nus < totalsize)
		{
			usedsize = nus;
			return (data + (usedsize - size));
		}
		btAssert(0);
		//&& (L"Not enough memory"));

		return (0);
	}
	SIMD_FORCE_INLINE btBlock* beginBlock()
	{
		btBlock* pb = (btBlock*)allocate(sizeof(btBlock));
		pb->previous = current;
		pb->address = data + usedsize;
		current = pb;
		return (pb);
	}
	SIMD_FORCE_INLINE void endBlock(btBlock* block)
	{
		btAssert(block == current);
		//Raise(L"Unmatched blocks");
		if (block == current)
		{
			current = block->previous;
			usedsize = (unsigned int)((block->address - data) - sizeof(btBlock));
		}
	}

private:
	void ctor()
	{
		data = 0;
		totalsize = 0;
		usedsize = 0;
		current = 0;
		ischild = false;
	}
	unsigned char* data;
	unsigned int totalsize;
	unsigned int usedsize;
	btBlock* current;
	bool ischild;
};

#endif  //BT_STACK_ALLOC
