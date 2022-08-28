///@file Configuration for Inverse Dynamics Library, such as choice of linear algebra library and underlying scalar type
#ifndef IDCONFIG_HPP_
#define IDCONFIG_HPP_

// If true, enable jacobian calculations.
// This adds a 3xN matrix to every body, + 2 3-Vectors.
// so it is not advised for large systems if it is not absolutely necessary.
// Also, this is not required for standard inverse dynamics calculations.
// Will only work with vector math libraries that support 3xN matrices.
#define BT_ID_WITH_JACOBIANS

#define BT_ID_SQRT(x) btSqrt(x)
#define BT_ID_FABS(x) btFabs(x)
#define BT_ID_COS(x) btCos(x)
#define BT_ID_SIN(x) btSin(x)
#define BT_ID_ATAN2(x, y) btAtan2(x, y)
#define BT_ID_POW(x, y) btPow(x, y)
#define BT_ID_PI SIMD_PI
#define BT_ID_SNPRINTF _snprintf

// error messages
#include "IDErrorMessages.hpp"

#define btInverseDynamics btInverseDynamicsBullet3
// Use default configuration with bullet's types
// Use the same scalar type as rest of bullet library
#include "LinearMath/btScalar.h"
typedef btScalar idScalar;
#include "LinearMath/btMinMax.h"
#define BT_ID_MAX(a, b) btMax(a, b)
#define BT_ID_MIN(a, b) btMin(a, b)

// use bullet types for arrays and array indices
#include "Bullet3Common/b3AlignedObjectArray.h"
// this is to make it work with C++2003, otherwise we could do this:
// template <typename T>
// using idArray = b3AlignedObjectArray<T>;
template <typename T>
struct idArray
{
	typedef b3AlignedObjectArray<T> type;
};
typedef int idArrayIdx;
#define ID_DECLARE_ALIGNED_ALLOCATOR() B3_DECLARE_ALIGNED_ALLOCATOR()

// use bullet's allocator functions
#define idMalloc btAllocFunc
#define idFree btFreeFunc

#define ID_LINEAR_MATH_USE_BULLET
#include "details/IDLinearMathInterface.hpp"
#endif