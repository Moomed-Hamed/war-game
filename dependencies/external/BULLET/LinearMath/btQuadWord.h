#ifndef BT_SIMD_QUADWORD_H
#define BT_SIMD_QUADWORD_H

#include "btScalar.h"
#include "btMinMax.h"

/**@brief The btQuadWord class is base class for btVector3 and btQuaternion. 
 * Some issues under PS3 Linux with IBM 2.1 SDK, gcc compiler prevent from using aligned quadword.
 */
ATTRIBUTE_ALIGNED16(class)
btQuadWord
{
protected:
	union
	{
		btSimdFloat4 mVec128;
		btScalar m_floats[4];
	};

public:
	SIMD_FORCE_INLINE btSimdFloat4 get128() const
	{
		return mVec128;
	}
	SIMD_FORCE_INLINE void set128(btSimdFloat4 v128)
	{
		mVec128 = v128;
	}

public:

	/**@brief Return the x value */
	SIMD_FORCE_INLINE const btScalar& getX() const { return m_floats[0]; }
	/**@brief Return the y value */
	SIMD_FORCE_INLINE const btScalar& getY() const { return m_floats[1]; }
	/**@brief Return the z value */
	SIMD_FORCE_INLINE const btScalar& getZ() const { return m_floats[2]; }
	/**@brief Set the x value */
	SIMD_FORCE_INLINE void setX(btScalar _x) { m_floats[0] = _x; };
	/**@brief Set the y value */
	SIMD_FORCE_INLINE void setY(btScalar _y) { m_floats[1] = _y; };
	/**@brief Set the z value */
	SIMD_FORCE_INLINE void setZ(btScalar _z) { m_floats[2] = _z; };
	/**@brief Set the w value */
	SIMD_FORCE_INLINE void setW(btScalar _w) { m_floats[3] = _w; };
	/**@brief Return the x value */
	SIMD_FORCE_INLINE const btScalar& x() const { return m_floats[0]; }
	/**@brief Return the y value */
	SIMD_FORCE_INLINE const btScalar& y() const { return m_floats[1]; }
	/**@brief Return the z value */
	SIMD_FORCE_INLINE const btScalar& z() const { return m_floats[2]; }
	/**@brief Return the w value */
	SIMD_FORCE_INLINE const btScalar& w() const { return m_floats[3]; }

	//SIMD_FORCE_INLINE btScalar&       operator[](int i)       { return (&m_floats[0])[i];	}
	//SIMD_FORCE_INLINE const btScalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator btScalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
	SIMD_FORCE_INLINE operator btScalar*() { return &m_floats[0]; }
	SIMD_FORCE_INLINE operator const btScalar*() const { return &m_floats[0]; }

	SIMD_FORCE_INLINE bool operator==(const btQuadWord& other) const
	{
		return (0xf == _mm_movemask_ps((__m128)_mm_cmpeq_ps(mVec128, other.mVec128)));
	}

	SIMD_FORCE_INLINE bool operator!=(const btQuadWord& other) const
	{
		return !(*this == other);
	}

	/**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
	SIMD_FORCE_INLINE void setValue(const btScalar& _x, const btScalar& _y, const btScalar& _z)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = 0.f;
	}

	/*		void getValue(btScalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] = m_floats[2];
		}
*/
	/**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
	SIMD_FORCE_INLINE void setValue(const btScalar& _x, const btScalar& _y, const btScalar& _z, const btScalar& _w)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = _w;
	}
	/**@brief No initialization constructor */
	SIMD_FORCE_INLINE btQuadWord()
	//	:m_floats[0](btScalar(0.)),m_floats[1](btScalar(0.)),m_floats[2](btScalar(0.)),m_floats[3](btScalar(0.))
	{
	}

	/**@brief Three argument constructor (zeros w)
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
	SIMD_FORCE_INLINE btQuadWord(const btScalar& _x, const btScalar& _y, const btScalar& _z)
	{
		m_floats[0] = _x, m_floats[1] = _y, m_floats[2] = _z, m_floats[3] = 0.0f;
	}

	/**@brief Initializing constructor
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
	SIMD_FORCE_INLINE btQuadWord(const btScalar& _x, const btScalar& _y, const btScalar& _z, const btScalar& _w)
	{
		m_floats[0] = _x, m_floats[1] = _y, m_floats[2] = _z, m_floats[3] = _w;
	}

	/**@brief Set each element to the max of the current values and the values of another btQuadWord
   * @param other The other btQuadWord to compare with 
   */
	SIMD_FORCE_INLINE void setMax(const btQuadWord& other)
	{
		mVec128 = _mm_max_ps(mVec128, other.mVec128);
	}
	/**@brief Set each element to the min of the current values and the values of another btQuadWord
   * @param other The other btQuadWord to compare with 
   */
	SIMD_FORCE_INLINE void setMin(const btQuadWord& other)
	{
		mVec128 = _mm_min_ps(mVec128, other.mVec128);
	}
};

#endif  //BT_SIMD_QUADWORD_H
