#include "btVector3.h"

#define float4 __m128

#define LOG2_ARRAY_SIZE 6
#define STACK_ARRAY_COUNT (1UL << LOG2_ARRAY_SIZE)

#include <emmintrin.h>

long _maxdot_large(const float *vv, const float *vec, unsigned long count, float *dotResult);
long _maxdot_large(const float *vv, const float *vec, unsigned long count, float *dotResult)
{
	const float4 *vertices = (const float4 *)vv;
	static const unsigned char indexTable[16] = {(unsigned char)-1, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0};
	float4 dotMax = btAssign128(-BT_INFINITY, -BT_INFINITY, -BT_INFINITY, -BT_INFINITY);
	float4 vvec = _mm_loadu_ps(vec);
	float4 vHi = btCastiTo128f(_mm_shuffle_epi32(btCastfTo128i(vvec), 0xaa));  /// zzzz
	float4 vLo = _mm_movelh_ps(vvec, vvec);                                    /// xyxy

	long maxIndex = -1L;

	size_t segment = 0;
	float4 stack_array[STACK_ARRAY_COUNT];

	size_t index;
	float4 max;
	// Faster loop without cleanup code for full tiles
	for (segment = 0; segment + STACK_ARRAY_COUNT * 4 <= count; segment += STACK_ARRAY_COUNT * 4)
	{
		max = dotMax;

		for (index = 0; index < STACK_ARRAY_COUNT; index += 4)
		{  // do four dot products at a time. Carefully avoid touching the w element.
			float4 v0 = vertices[0];
			float4 v1 = vertices[1];
			float4 v2 = vertices[2];
			float4 v3 = vertices[3];
			vertices += 4;

			float4 lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			float4 hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			float4 lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			float4 hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			float4 z = _mm_shuffle_ps(hi0, hi1, 0x88);
			float4 x = _mm_shuffle_ps(lo0, lo1, 0x88);
			float4 y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 1] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 2] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 3] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			// It is too costly to keep the index of the max here. We will look for it again later.  We save a lot of work this way.
		}

		// If we found a new max
		if (0xf != _mm_movemask_ps((float4)_mm_cmpeq_ps(max, dotMax)))
		{
			// copy the new max across all lanes of our max accumulator
			max = _mm_max_ps(max, (float4)_mm_shuffle_ps(max, max, 0x4e));
			max = _mm_max_ps(max, (float4)_mm_shuffle_ps(max, max, 0xb1));

			dotMax = max;

			// find first occurrence of that max
			size_t test;
			for (index = 0; 0 == (test = _mm_movemask_ps(_mm_cmpeq_ps(stack_array[index], max))); index++)  // local_count must be a multiple of 4
			{
			}
			// record where it is.
			maxIndex = 4 * index + segment + indexTable[test];
		}
	}

	// account for work we've already done
	count -= segment;

	// Deal with the last < STACK_ARRAY_COUNT vectors
	max = dotMax;
	index = 0;

	if (btUnlikely(count > 16))
	{
		for (; index + 4 <= count / 4; index += 4)
		{  // do four dot products at a time. Carefully avoid touching the w element.
			float4 v0 = vertices[0];
			float4 v1 = vertices[1];
			float4 v2 = vertices[2];
			float4 v3 = vertices[3];
			vertices += 4;

			float4 lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			float4 hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			float4 lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			float4 hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			float4 z = _mm_shuffle_ps(hi0, hi1, 0x88);
			float4 x = _mm_shuffle_ps(lo0, lo1, 0x88);
			float4 y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 1] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 2] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 3] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan

			// It is too costly to keep the index of the max here. We will look for it again later.  We save a lot of work this way.
		}
	}

	size_t localCount = (count & -4L) - 4 * index;
	if (localCount)
	{
		for (unsigned int i = 0; i < localCount / 4; i++, index++)
		{  // do four dot products at a time. Carefully avoid touching the w element.
			float4 v0 = vertices[0];
			float4 v1 = vertices[1];
			float4 v2 = vertices[2];
			float4 v3 = vertices[3];
			vertices += 4;

			float4 lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			float4 hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			float4 lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			float4 hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			float4 z = _mm_shuffle_ps(hi0, hi1, 0x88);
			float4 x = _mm_shuffle_ps(lo0, lo1, 0x88);
			float4 y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index] = x;
			max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan
		}
	}

	// process the last few points
	if (count & 3)
	{
		float4 v0, v1, v2, x, y, z;
		switch (count & 3)
		{
			case 3:
			{
				v0 = vertices[0];
				v1 = vertices[1];
				v2 = vertices[2];

				// Calculate 3 dot products, transpose, duplicate v2
				float4 lo0 = _mm_movelh_ps(v0, v1);  // xyxy.lo
				float4 hi0 = _mm_movehl_ps(v1, v0);  // z?z?.lo
				lo0 = lo0 * vLo;
				z = _mm_shuffle_ps(hi0, v2, 0xa8);  // z0z1z2z2
				z = z * vHi;
				float4 lo1 = _mm_movelh_ps(v2, v2);  // xyxy
				lo1 = lo1 * vLo;
				x = _mm_shuffle_ps(lo0, lo1, 0x88);
				y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			}
			break;
			case 2:
			{
				v0 = vertices[0];
				v1 = vertices[1];
				float4 xy = _mm_movelh_ps(v0, v1);
				z = _mm_movehl_ps(v1, v0);
				xy = xy * vLo;
				z = _mm_shuffle_ps(z, z, 0xa8);
				x = _mm_shuffle_ps(xy, xy, 0xa8);
				y = _mm_shuffle_ps(xy, xy, 0xfd);
				z = z * vHi;
			}
			break;
			case 1:
			{
				float4 xy = vertices[0];
				z = _mm_shuffle_ps(xy, xy, 0xaa);
				xy = xy * vLo;
				z = z * vHi;
				x = _mm_shuffle_ps(xy, xy, 0);
				y = _mm_shuffle_ps(xy, xy, 0x55);
			}
			break;
		}
		x = x + y;
		x = x + z;
		stack_array[index] = x;
		max = _mm_max_ps(x, max);  // control the order here so that max is never NaN even if x is nan
		index++;
	}

	// if we found a new max.
	if (0 == segment || 0xf != _mm_movemask_ps((float4)_mm_cmpeq_ps(max, dotMax)))
	{  // we found a new max. Search for it
		// find max across the max vector, place in all elements of max -- big latency hit here
		max = _mm_max_ps(max, (float4)_mm_shuffle_ps(max, max, 0x4e));
		max = _mm_max_ps(max, (float4)_mm_shuffle_ps(max, max, 0xb1));

		// It is slightly faster to do this part in scalar code when count < 8. However, the common case for
		// this where it actually makes a difference is handled in the early out at the top of the function,
		// so it is less than a 1% difference here. I opted for improved code size, fewer branches and reduced
		// complexity, and removed it.

		dotMax = max;

		// scan for the first occurence of max in the array
		size_t test;
		for (index = 0; 0 == (test = _mm_movemask_ps(_mm_cmpeq_ps(stack_array[index], max))); index++)  // local_count must be a multiple of 4
		{
		}
		maxIndex = 4 * index + segment + indexTable[test];
	}

	_mm_store_ss(dotResult, dotMax);
	return maxIndex;
}

long _mindot_large(const float *vv, const float *vec, unsigned long count, float *dotResult);
long _mindot_large(const float *vv, const float *vec, unsigned long count, float *dotResult)
{
	const float4 *vertices = (const float4 *)vv;
	static const unsigned char indexTable[16] = {(unsigned char)-1, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0};
	float4 dotmin = btAssign128(BT_INFINITY, BT_INFINITY, BT_INFINITY, BT_INFINITY);
	float4 vvec = _mm_loadu_ps(vec);
	float4 vHi = btCastiTo128f(_mm_shuffle_epi32(btCastfTo128i(vvec), 0xaa));  /// zzzz
	float4 vLo = _mm_movelh_ps(vvec, vvec);                                    /// xyxy

	long minIndex = -1L;

	size_t segment = 0;
	float4 stack_array[STACK_ARRAY_COUNT];

	size_t index;
	float4 min;
	// Faster loop without cleanup code for full tiles
	for (segment = 0; segment + STACK_ARRAY_COUNT * 4 <= count; segment += STACK_ARRAY_COUNT * 4)
	{
		min = dotmin;

		for (index = 0; index < STACK_ARRAY_COUNT; index += 4)
		{  // do four dot products at a time. Carefully avoid touching the w element.
			float4 v0 = vertices[0];
			float4 v1 = vertices[1];
			float4 v2 = vertices[2];
			float4 v3 = vertices[3];
			vertices += 4;

			float4 lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			float4 hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			float4 lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			float4 hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			float4 z = _mm_shuffle_ps(hi0, hi1, 0x88);
			float4 x = _mm_shuffle_ps(lo0, lo1, 0x88);
			float4 y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 1] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 2] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 3] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			// It is too costly to keep the index of the min here. We will look for it again later.  We save a lot of work this way.
		}

		// If we found a new min
		if (0xf != _mm_movemask_ps((float4)_mm_cmpeq_ps(min, dotmin)))
		{
			// copy the new min across all lanes of our min accumulator
			min = _mm_min_ps(min, (float4)_mm_shuffle_ps(min, min, 0x4e));
			min = _mm_min_ps(min, (float4)_mm_shuffle_ps(min, min, 0xb1));

			dotmin = min;

			// find first occurrence of that min
			size_t test;
			for (index = 0; 0 == (test = _mm_movemask_ps(_mm_cmpeq_ps(stack_array[index], min))); index++)  // local_count must be a multiple of 4
			{
			}
			// record where it is.
			minIndex = 4 * index + segment + indexTable[test];
		}
	}

	// account for work we've already done
	count -= segment;

	// Deal with the last < STACK_ARRAY_COUNT vectors
	min = dotmin;
	index = 0;

	if (btUnlikely(count > 16))
	{
		for (; index + 4 <= count / 4; index += 4)
		{  // do four dot products at a time. Carefully avoid touching the w element.
			float4 v0 = vertices[0];
			float4 v1 = vertices[1];
			float4 v2 = vertices[2];
			float4 v3 = vertices[3];
			vertices += 4;

			float4 lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			float4 hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			float4 lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			float4 hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			float4 z = _mm_shuffle_ps(hi0, hi1, 0x88);
			float4 x = _mm_shuffle_ps(lo0, lo1, 0x88);
			float4 y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 1] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 2] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			v0 = vertices[0];
			v1 = vertices[1];
			v2 = vertices[2];
			v3 = vertices[3];
			vertices += 4;

			lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
			hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
			lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
			hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

			lo0 = lo0 * vLo;
			lo1 = lo1 * vLo;
			z = _mm_shuffle_ps(hi0, hi1, 0x88);
			x = _mm_shuffle_ps(lo0, lo1, 0x88);
			y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			z = z * vHi;
			x = x + y;
			x = x + z;
			stack_array[index + 3] = x;
			min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan

			// It is too costly to keep the index of the min here. We will look for it again later.  We save a lot of work this way.
		}
	}

	size_t localCount = (count & -4L) - 4 * index;
	if (localCount)
	{
		{
			for (unsigned int i = 0; i < localCount / 4; i++, index++)
			{  // do four dot products at a time. Carefully avoid touching the w element.
				float4 v0 = vertices[0];
				float4 v1 = vertices[1];
				float4 v2 = vertices[2];
				float4 v3 = vertices[3];
				vertices += 4;

				float4 lo0 = _mm_movelh_ps(v0, v1);  // x0y0x1y1
				float4 hi0 = _mm_movehl_ps(v1, v0);  // z0?0z1?1
				float4 lo1 = _mm_movelh_ps(v2, v3);  // x2y2x3y3
				float4 hi1 = _mm_movehl_ps(v3, v2);  // z2?2z3?3

				lo0 = lo0 * vLo;
				lo1 = lo1 * vLo;
				float4 z = _mm_shuffle_ps(hi0, hi1, 0x88);
				float4 x = _mm_shuffle_ps(lo0, lo1, 0x88);
				float4 y = _mm_shuffle_ps(lo0, lo1, 0xdd);
				z = z * vHi;
				x = x + y;
				x = x + z;
				stack_array[index] = x;
				min = _mm_min_ps(x, min);  // control the order here so that max is never NaN even if x is nan
			}
		}
	}

	// process the last few points
	if (count & 3)
	{
		float4 v0, v1, v2, x, y, z;
		switch (count & 3)
		{
			case 3:
			{
				v0 = vertices[0];
				v1 = vertices[1];
				v2 = vertices[2];

				// Calculate 3 dot products, transpose, duplicate v2
				float4 lo0 = _mm_movelh_ps(v0, v1);  // xyxy.lo
				float4 hi0 = _mm_movehl_ps(v1, v0);  // z?z?.lo
				lo0 = lo0 * vLo;
				z = _mm_shuffle_ps(hi0, v2, 0xa8);  // z0z1z2z2
				z = z * vHi;
				float4 lo1 = _mm_movelh_ps(v2, v2);  // xyxy
				lo1 = lo1 * vLo;
				x = _mm_shuffle_ps(lo0, lo1, 0x88);
				y = _mm_shuffle_ps(lo0, lo1, 0xdd);
			}
			break;
			case 2:
			{
				v0 = vertices[0];
				v1 = vertices[1];
				float4 xy = _mm_movelh_ps(v0, v1);
				z = _mm_movehl_ps(v1, v0);
				xy = xy * vLo;
				z = _mm_shuffle_ps(z, z, 0xa8);
				x = _mm_shuffle_ps(xy, xy, 0xa8);
				y = _mm_shuffle_ps(xy, xy, 0xfd);
				z = z * vHi;
			}
			break;
			case 1:
			{
				float4 xy = vertices[0];
				z = _mm_shuffle_ps(xy, xy, 0xaa);
				xy = xy * vLo;
				z = z * vHi;
				x = _mm_shuffle_ps(xy, xy, 0);
				y = _mm_shuffle_ps(xy, xy, 0x55);
			}
			break;
		}
		x = x + y;
		x = x + z;
		stack_array[index] = x;
		min = _mm_min_ps(x, min);  // control the order here so that min is never NaN even if x is nan
		index++;
	}

	// if we found a new min.
	if (0 == segment || 0xf != _mm_movemask_ps((float4)_mm_cmpeq_ps(min, dotmin)))
	{  // we found a new min. Search for it
		// find min across the min vector, place in all elements of min -- big latency hit here
		min = _mm_min_ps(min, (float4)_mm_shuffle_ps(min, min, 0x4e));
		min = _mm_min_ps(min, (float4)_mm_shuffle_ps(min, min, 0xb1));

		// It is slightly faster to do this part in scalar code when count < 8. However, the common case for
		// this where it actually makes a difference is handled in the early out at the top of the function,
		// so it is less than a 1% difference here. I opted for improved code size, fewer branches and reduced
		// complexity, and removed it.

		dotmin = min;

		// scan for the first occurence of min in the array
		size_t test;
		for (index = 0; 0 == (test = _mm_movemask_ps(_mm_cmpeq_ps(stack_array[index], min))); index++)  // local_count must be a multiple of 4
		{
		}
		minIndex = 4 * index + segment + indexTable[test];
	}

	_mm_store_ss(dotResult, dotmin);
	return minIndex;
}
