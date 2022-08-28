#ifndef BT_SCALAR_H
#define BT_SCALAR_H

#include <math.h>
#include <stdlib.h>  //size_t for MSVC 6.0
#include <float.h>

/* SVN $Revision$ on $Date$ from http://bullet.googlecode.com*/
#define BT_BULLET_VERSION 320

inline int btGetVersion()
{
	return BT_BULLET_VERSION;
}

inline int btIsDoublePrecision()
{
	return false;
}


// The following macro "BT_NOT_EMPTY_FILE" can be put into a file
// in order suppress the MS Visual C++ Linker warning 4221
//
// warning LNK4221: no public symbols found; archive member will be inaccessible
//
// This warning occurs on PC and XBOX when a file compiles out completely
// has no externally visible symbols which may be dependant on configuration
// #defines and options.
//
// see more https://stackoverflow.com/questions/1822887/what-is-the-best-way-to-eliminate-ms-visual-c-linker-warning-warning-lnk422

#define BT_NOT_EMPTY_FILE_CAT_II(p, res) res
#define BT_NOT_EMPTY_FILE_CAT_I(a, b) BT_NOT_EMPTY_FILE_CAT_II(~, a##b)
#define BT_NOT_EMPTY_FILE_CAT(a, b) BT_NOT_EMPTY_FILE_CAT_I(a, b)
#define BT_NOT_EMPTY_FILE namespace { char BT_NOT_EMPTY_FILE_CAT(NoEmptyFileDummy, __COUNTER__); }

#define BT_DEBUG

#pragma warning(disable : 4324) // disable padding warning
//			#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
#pragma warning(disable:4996) //Turn off warnings about deprecated C routines
//			#pragma warning(disable:4786) // Disable the "debug name too long" warning

#define SIMD_FORCE_INLINE __forceinline
#define ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
#define ATTRIBUTE_ALIGNED64(a) __declspec(align(64)) a
#define ATTRIBUTE_ALIGNED128(a) __declspec (align(128)) a


#define BT_USE_SIMD_VECTOR3
#define BT_USE_SSE

#define BT_ALLOW_SSE4

//BT_USE_SSE_IN_API is disabled under Windows by default, because 
//it makes it harder to integrate Bullet into your application under Windows 
//(structured embedding Bullet structs/classes need to be 16-byte aligned)
//with relatively little performance gain
//If you are not embedded Bullet data in your classes, or make sure that you align those classes on 16-byte boundaries
//you can manually enable this line or set it in the build system for a bit of performance gain (a few percent, dependent on usage)
//#define BT_USE_SSE_IN_API

#include <emmintrin.h>

#include <stdio.h>
#define btAssert(x) { if(!(x)){printf("Assert " __FILE__ ":%u (%s)\n", __LINE__, #x);__debugbreak();	}}

//btFullAssert is optional, slows down a lot
#define btFullAssert(x)

#define btLikely(_c)  _c
#define btUnlikely(_c) _c


///The btScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
typedef float btScalar;
//keep BT_LARGE_FLOAT*BT_LARGE_FLOAT < FLT_MAX
#define BT_LARGE_FLOAT 1e18f

typedef __m128 btSimdFloat4;

static int btNanMask = 0x7F800001;	
#define BT_NAN (*(float *)&btNanMask)

static int btInfinityMask = 0x7F800000;
#define BT_INFINITY (*(float *)&btInfinityMask)
inline int btGetInfinityMask()  //suppress stupid compiler warning
{
	return btInfinityMask;
}

inline __m128 operator+(const __m128 A, const __m128 B)
{
	return _mm_add_ps(A, B);
}

inline __m128 operator-(const __m128 A, const __m128 B)
{
	return _mm_sub_ps(A, B);
}

inline __m128 operator*(const __m128 A, const __m128 B)
{
	return _mm_mul_ps(A, B);
}

#define btCastfTo128i(a) (_mm_castps_si128(a))
#define btCastfTo128d(a) (_mm_castps_pd(a))
#define btCastiTo128f(a) (_mm_castsi128_ps(a))
#define btCastdTo128f(a) (_mm_castpd_ps(a))
#define btCastdTo128i(a) (_mm_castpd_si128(a))
#define btAssign128(r0, r1, r2, r3) _mm_setr_ps(r0, r1, r2, r3)

#define BT_DECLARE_ALIGNED_ALLOCATOR()                                                                     \
	SIMD_FORCE_INLINE void *operator new(size_t sizeInBytes) { return btAlignedAlloc(sizeInBytes, 16); }   \
	SIMD_FORCE_INLINE void operator delete(void *ptr) { btAlignedFree(ptr); }                              \
	SIMD_FORCE_INLINE void *operator new(size_t, void *ptr) { return ptr; }                                \
	SIMD_FORCE_INLINE void operator delete(void *, void *) {}                                              \
	SIMD_FORCE_INLINE void *operator new[](size_t sizeInBytes) { return btAlignedAlloc(sizeInBytes, 16); } \
	SIMD_FORCE_INLINE void operator delete[](void *ptr) { btAlignedFree(ptr); }                            \
	SIMD_FORCE_INLINE void *operator new[](size_t, void *ptr) { return ptr; }                              \
	SIMD_FORCE_INLINE void operator delete[](void *, void *) {}

	SIMD_FORCE_INLINE btScalar btSqrt(btScalar y)
	{
		return sqrtf(y);
	}
	SIMD_FORCE_INLINE btScalar btFabs(btScalar x) { return fabsf(x); }
	SIMD_FORCE_INLINE btScalar btCos(btScalar x) { return cosf(x); }
	SIMD_FORCE_INLINE btScalar btSin(btScalar x) { return sinf(x); }
	SIMD_FORCE_INLINE btScalar btTan(btScalar x) { return tanf(x); }
	SIMD_FORCE_INLINE btScalar btAcos(btScalar x)
	{
		if (x < btScalar(-1))
			x = btScalar(-1);
		if (x > btScalar(1))
			x = btScalar(1);
		return acosf(x);
	}
	SIMD_FORCE_INLINE btScalar btAsin(btScalar x)
	{
		if (x < btScalar(-1))
			x = btScalar(-1);
		if (x > btScalar(1))
			x = btScalar(1);
		return asinf(x);
	}
	SIMD_FORCE_INLINE btScalar btAtan(btScalar x) { return atanf(x); }
	SIMD_FORCE_INLINE btScalar btAtan2(btScalar x, btScalar y) { return atan2f(x, y); }
	SIMD_FORCE_INLINE btScalar btExp(btScalar x) { return expf(x); }
	SIMD_FORCE_INLINE btScalar btLog(btScalar x) { return logf(x); }
	SIMD_FORCE_INLINE btScalar btPow(btScalar x, btScalar y) { return powf(x, y); }
	SIMD_FORCE_INLINE btScalar btFmod(btScalar x, btScalar y) { return fmodf(x, y); }

#define SIMD_PI btScalar(3.1415926535897932384626433832795029)
#define SIMD_2_PI (btScalar(2.0) * SIMD_PI)
#define SIMD_HALF_PI (SIMD_PI * btScalar(0.5))
#define SIMD_RADS_PER_DEG (SIMD_2_PI / btScalar(360.0))
#define SIMD_DEGS_PER_RAD (btScalar(360.0) / SIMD_2_PI)
#define SIMDSQRT12 btScalar(0.7071067811865475244008443621048490)
#define btRecipSqrt(x) ((btScalar)(btScalar(1.0) / btSqrt(btScalar(x)))) /* reciprocal square root */
#define btRecip(x) (btScalar(1.0) / btScalar(x))

#define SIMD_EPSILON FLT_EPSILON
#define SIMD_INFINITY FLT_MAX
#define BT_ONE 1.0f
#define BT_ZERO 0.0f
#define BT_TWO 2.0f
#define BT_HALF 0.5f

// clang-format on

SIMD_FORCE_INLINE btScalar btAtan2Fast(btScalar y, btScalar x)
{
	btScalar coeff_1 = SIMD_PI / 4.0f;
	btScalar coeff_2 = 3.0f * coeff_1;
	btScalar abs_y = btFabs(y);
	btScalar angle;
	if (x >= 0.0f)
	{
		btScalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	}
	else
	{
		btScalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

SIMD_FORCE_INLINE bool btFuzzyZero(btScalar x) { return btFabs(x) < SIMD_EPSILON; }

SIMD_FORCE_INLINE bool btEqual(btScalar a, btScalar eps)
{
	return (((a) <= eps) && !((a) < -eps));
}
SIMD_FORCE_INLINE bool btGreaterEqual(btScalar a, btScalar eps)
{
	return (!((a) <= eps));
}

SIMD_FORCE_INLINE int btIsNegative(btScalar x)
{
	return x < btScalar(0.0) ? 1 : 0;
}

SIMD_FORCE_INLINE btScalar btRadians(btScalar x) { return x * SIMD_RADS_PER_DEG; }
SIMD_FORCE_INLINE btScalar btDegrees(btScalar x) { return x * SIMD_DEGS_PER_RAD; }

#define BT_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } * name

SIMD_FORCE_INLINE btScalar btFsel(btScalar a, btScalar b, btScalar c)
{
	return a >= 0 ? b : c;
}

#define btFsels(a, b, c) (btScalar) btFsel(a, b, c)

SIMD_FORCE_INLINE bool btMachineIsLittleEndian()
{
	long int i = 1;
	const char *p = (const char *)&i;
	if (p[0] == 1)  // Lowest address contains the least significant byte
		return true;
	else
		return false;
}

///btSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
SIMD_FORCE_INLINE unsigned btSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero)
{
	// Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
	// Rely on positive value or'ed with its negative having sign bit on
	// and zero value or'ed with its negative (which is still zero) having sign bit off
	// Use arithmetic shift right, shifting the sign bit through all 32 bits
	unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
	unsigned testEqz = ~testNz;
	return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
SIMD_FORCE_INLINE int btSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
	unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
	unsigned testEqz = ~testNz;
	return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
SIMD_FORCE_INLINE float btSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef BT_HAVE_NATIVE_FSEL
	return (float)btFsel((btScalar)condition - btScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
	return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero;
#endif
}

template <typename T>
SIMD_FORCE_INLINE void btSwap(T &a, T &b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

//PCK: endian swapping functions
SIMD_FORCE_INLINE unsigned btSwapEndian(unsigned val)
{
	return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8) | ((val & 0x000000ff) << 24));
}

SIMD_FORCE_INLINE unsigned short btSwapEndian(unsigned short val)
{
	return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

SIMD_FORCE_INLINE unsigned btSwapEndian(int val)
{
	return btSwapEndian((unsigned)val);
}

SIMD_FORCE_INLINE unsigned short btSwapEndian(short val)
{
	return btSwapEndian((unsigned short)val);
}

///btSwapFloat uses using char pointers to swap the endianness
////btSwapFloat/btSwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
///Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754.
///When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception.
///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you.
///so instead of returning a float/double, we return integer/long long integer
SIMD_FORCE_INLINE unsigned int btSwapEndianFloat(float d)
{
	unsigned int a = 0;
	unsigned char *dst = (unsigned char *)&a;
	unsigned char *src = (unsigned char *)&d;

	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
	return a;
}

// unswap using char pointers
SIMD_FORCE_INLINE float btUnswapEndianFloat(unsigned int a)
{
	float d = 0.0f;
	unsigned char *src = (unsigned char *)&a;
	unsigned char *dst = (unsigned char *)&d;

	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];

	return d;
}

// swap using char pointers
SIMD_FORCE_INLINE void btSwapEndianDouble(double d, unsigned char *dst)
{
	unsigned char *src = (unsigned char *)&d;

	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
}

// unswap using char pointers
SIMD_FORCE_INLINE double btUnswapEndianDouble(const unsigned char *src)
{
	double d = 0.0;
	unsigned char *dst = (unsigned char *)&d;

	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];

	return d;
}

template <typename T>
SIMD_FORCE_INLINE void btSetZero(T *a, int n)
{
	T *acurr = a;
	size_t ncurr = n;
	while (ncurr > 0)
	{
		*(acurr++) = 0;
		--ncurr;
	}
}

SIMD_FORCE_INLINE btScalar btLargeDot(const btScalar *a, const btScalar *b, int n)
{
	btScalar p0, q0, m0, p1, q1, m1, sum;
	sum = 0;
	n -= 2;
	while (n >= 0)
	{
		p0 = a[0];
		q0 = b[0];
		m0 = p0 * q0;
		p1 = a[1];
		q1 = b[1];
		m1 = p1 * q1;
		sum += m0;
		sum += m1;
		a += 2;
		b += 2;
		n -= 2;
	}
	n += 2;
	while (n > 0)
	{
		sum += (*a) * (*b);
		a++;
		b++;
		n--;
	}
	return sum;
}

// returns normalized value in range [-SIMD_PI, SIMD_PI]
SIMD_FORCE_INLINE btScalar btNormalizeAngle(btScalar angleInRadians)
{
	angleInRadians = btFmod(angleInRadians, SIMD_2_PI);
	if (angleInRadians < -SIMD_PI)
	{
		return angleInRadians + SIMD_2_PI;
	}
	else if (angleInRadians > SIMD_PI)
	{
		return angleInRadians - SIMD_2_PI;
	}
	else
	{
		return angleInRadians;
	}
}

///rudimentary class to provide type info
struct btTypedObject
{
	btTypedObject(int objectType)
		: m_objectType(objectType)
	{
	}
	int m_objectType;
	inline int getObjectType() const
	{
		return m_objectType;
	}
};

///align a pointer to the provided alignment, upwards
template <typename T>
T *btAlignPointer(T *unalignedPtr, size_t alignment)
{
	struct btConvertPointerSizeT
	{
		union {
			T *ptr;
			size_t integer;
		};
	};
	btConvertPointerSizeT converter;

	const size_t bit_mask = ~(alignment - 1);
	converter.ptr = unalignedPtr;
	converter.integer += alignment - 1;
	converter.integer &= bit_mask;
	return converter.ptr;
}

#endif  //BT_SCALAR_H
