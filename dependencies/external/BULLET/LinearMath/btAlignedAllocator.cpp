#include "btAlignedAllocator.h"
#include <string.h>

static void *btAllocDefault(size_t size)
{
  char* data = (char*) malloc(size);
  memset(data,0,size);//keep msan happy
  return data;
}

static void btFreeDefault(void *ptr)
{
	free(ptr);
}

static btAllocFunc *sAllocFunc = btAllocDefault;
static btFreeFunc *sFreeFunc = btFreeDefault;

static inline void *btAlignedAllocDefault(size_t size, int alignment)
{
	void *ret;
	char *real;
	real = (char *)sAllocFunc(size + sizeof(void *) + (alignment - 1));
	if (real)
	{
		ret = btAlignPointer(real + sizeof(void *), alignment);
		*((void **)(ret)-1) = (void *)(real);
	}
	else
	{
		ret = (void *)(real);
	}
	//keep msan happy
	memset((char *)ret, 0, size);
	return (ret);
}

static inline void btAlignedFreeDefault(void *ptr)
{
	void *real;

	if (ptr)
	{
		real = *((void **)(ptr)-1);
		sFreeFunc(real);
	}
}

static btAlignedAllocFunc *sAlignedAllocFunc = btAlignedAllocDefault;
static btAlignedFreeFunc *sAlignedFreeFunc = btAlignedFreeDefault;

void btAlignedAllocSetCustomAligned(btAlignedAllocFunc *allocFunc, btAlignedFreeFunc *freeFunc)
{
	sAlignedAllocFunc = allocFunc ? allocFunc : btAlignedAllocDefault;
	sAlignedFreeFunc = freeFunc ? freeFunc : btAlignedFreeDefault;
}

void btAlignedAllocSetCustom(btAllocFunc *allocFunc, btFreeFunc *freeFunc)
{
	sAllocFunc = allocFunc ? allocFunc : btAllocDefault;
	sFreeFunc = freeFunc ? freeFunc : btFreeDefault;
}

void *btAlignedAllocInternal(size_t size, int alignment)
{
	void *ptr;
	ptr = sAlignedAllocFunc(size, alignment);
	//	printf("btAlignedAllocInternal %d, %x\n",size,ptr);
	return ptr;
}

void btAlignedFreeInternal(void *ptr)
{
	if (!ptr)
	{
		return;
	}

	//	printf("btAlignedFreeInternal %x\n",ptr);
	sAlignedFreeFunc(ptr);
}
