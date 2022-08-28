#include "b3AlignedAllocator.h"

static void *b3AllocDefault(size_t size)
{
	return malloc(size);
}

static void b3FreeDefault(void *ptr)
{
	free(ptr);
}

static b3AllocFunc *b3s_allocFunc = b3AllocDefault;
static b3FreeFunc *b3s_freeFunc = b3FreeDefault;

static inline void *b3AlignedAllocDefault(size_t size, int alignment)
{
	void *ret;
	char *real;
	real = (char *)b3s_allocFunc(size + sizeof(void *) + (alignment - 1));
	if (real)
	{
		ret = b3AlignPointer(real + sizeof(void *), alignment);
		*((void **)(ret)-1) = (void *)(real);
	}
	else
	{
		ret = (void *)(real);
	}
	return (ret);
}

static inline void b3AlignedFreeDefault(void *ptr)
{
	void *real;

	if (ptr)
	{
		real = *((void **)(ptr)-1);
		b3s_freeFunc(real);
	}
}

static b3AlignedAllocFunc *b3s_alignedAllocFunc = b3AlignedAllocDefault;
static b3AlignedFreeFunc *b3s_alignedFreeFunc = b3AlignedFreeDefault;

void b3AlignedAllocSetCustomAligned(b3AlignedAllocFunc *allocFunc, b3AlignedFreeFunc *freeFunc)
{
	b3s_alignedAllocFunc = allocFunc ? allocFunc : b3AlignedAllocDefault;
	b3s_alignedFreeFunc = freeFunc ? freeFunc : b3AlignedFreeDefault;
}

void b3AlignedAllocSetCustom(b3AllocFunc *allocFunc, b3FreeFunc *freeFunc)
{
	b3s_allocFunc = allocFunc ? allocFunc : b3AllocDefault;
	b3s_freeFunc = freeFunc ? freeFunc : b3FreeDefault;
}

void *b3AlignedAllocInternal(size_t size, int alignment)
{
	void *ptr;
	ptr = b3s_alignedAllocFunc(size, alignment);
	//	b3Printf("b3AlignedAllocInternal %d, %x\n",size,ptr);
	return ptr;
}

void b3AlignedFreeInternal(void *ptr)
{
	if (!ptr)
	{
		return;
	}

	//	b3Printf("b3AlignedFreeInternal %x\n",ptr);
	b3s_alignedFreeFunc(ptr);
}