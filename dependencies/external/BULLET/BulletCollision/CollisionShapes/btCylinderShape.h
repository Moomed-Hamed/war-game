#ifndef BT_CYLINDER_MINKOWSKI_H
#define BT_CYLINDER_MINKOWSKI_H

#include "btBoxShape.h"

SIMD_FORCE_INLINE btVector3 CylinderLocalSupportX(const btVector3& halfExtents, const btVector3& v)
{
	const int cylinderUpAxis = 0;
	const int XX = 1;
	const int YY = 0;
	const int ZZ = 2;

	//mapping depends on how cylinder local orientation is
	// extents of the cylinder is: X,Y is for radius, and Z for height

	btScalar radius = halfExtents[XX];
	btScalar halfHeight = halfExtents[cylinderUpAxis];

	btVector3 tmp;
	btScalar d;

	btScalar s = btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
	if (s != btScalar(0.0))
	{
		d = radius / s;
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
	else
	{
		tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = btScalar(0.0);
		return tmp;
	}
}

inline btVector3 CylinderLocalSupportY(const btVector3& halfExtents, const btVector3& v)
{
	const int cylinderUpAxis = 1;
	const int XX = 0;
	const int YY = 1;
	const int ZZ = 2;

	btScalar radius = halfExtents[XX];
	btScalar halfHeight = halfExtents[cylinderUpAxis];

	btVector3 tmp;
	btScalar d;

	btScalar s = btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
	if (s != btScalar(0.0))
	{
		d = radius / s;
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
	else
	{
		tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = btScalar(0.0);
		return tmp;
	}
}

inline btVector3 CylinderLocalSupportZ(const btVector3& halfExtents, const btVector3& v)
{
	const int cylinderUpAxis = 2;
	const int XX = 0;
	const int YY = 2;
	const int ZZ = 1;

	//mapping depends on how cylinder local orientation is
	// extents of the cylinder is: X,Y is for radius, and Z for height

	btScalar radius = halfExtents[XX];
	btScalar halfHeight = halfExtents[cylinderUpAxis];

	btVector3 tmp;
	btScalar d;

	btScalar s = btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
	if (s != btScalar(0.0))
	{
		d = radius / s;
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
	else
	{
		tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = btScalar(0.0);
		return tmp;
	}
}

/// The btCylinderShape class implements a cylinder shape primitive, centered around the origin. Its central axis aligned with the Y axis. btCylinderShapeX is aligned with the X axis and btCylinderShapeZ around the Z axis.
ATTRIBUTE_ALIGNED16(class)
btCylinderShape : public btConvexInternalShape

{
protected:
	int m_upAxis;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btVector3 getHalfExtentsWithMargin() const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();
		btVector3 margin(getMargin(), getMargin(), getMargin());
		halfExtents += margin;
		return halfExtents;
	}

	const btVector3& getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;  //changed in Bullet 2.63: assume the scaling and margin are included
	}

	btCylinderShape(const btVector3& halfExtents)
		: btConvexInternalShape(),
		  m_upAxis(1)
	{
		btVector3 margin(getMargin(), getMargin(), getMargin());
		m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;

		setSafeMargin(halfExtents);

		m_shapeType = CYLINDER_SHAPE_PROXYTYPE;
	}

	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		/*
	cylinder is defined as following:
	*
	* - principle axis aligned along y by default, radius in x, z-value not used
	* - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
	* - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
	*
	*/

		btScalar radius2;                                    // square of cylinder radius
		btScalar height2;                                    // square of cylinder height
		btVector3 halfExtents = getHalfExtentsWithMargin();  // get cylinder dimension
		btScalar div12 = mass / 12.f;
		btScalar div4 = mass / 4.f;
		btScalar div2 = mass / 2.f;
		int idxRadius, idxHeight;

		switch (m_upAxis)  // get indices of radius and height of cylinder
		{
			case 0:  // cylinder is aligned along x
				idxRadius = 1;
				idxHeight = 0;
				break;
			case 2:  // cylinder is aligned along z
				idxRadius = 0;
				idxHeight = 2;
				break;
			default:  // cylinder is aligned along y
				idxRadius = 0;
				idxHeight = 1;
		}

		// calculate squares
		radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
		height2 = btScalar(4.) * halfExtents[idxHeight] * halfExtents[idxHeight];

		// calculate tensor terms
		btScalar t1 = div12 * height2 + div4 * radius2;
		btScalar t2 = div2 * radius2;

		switch (m_upAxis)  // set diagonal elements of inertia tensor
		{
			case 0:  // cylinder is aligned along x
				inertia.setValue(t2, t1, t1);
				break;
			case 2:  // cylinder is aligned along z
				inertia.setValue(t1, t1, t2);
				break;
			default:  // cylinder is aligned along y
				inertia.setValue(t1, t2, t1);
		}
	}

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		return CylinderLocalSupportY(getHalfExtentsWithoutMargin(), vec);
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		for (int i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i] = CylinderLocalSupportY(getHalfExtentsWithoutMargin(), vectors[i]);
		}
	}

	virtual void setMargin(btScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		btVector3 oldMargin(getMargin(), getMargin(), getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

		btConvexInternalShape::setMargin(collisionMargin);
		btVector3 newMargin(getMargin(), getMargin(), getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
	}

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
	{
		btVector3 supVertex;
		supVertex = localGetSupportingVertexWithoutMargin(vec);

		if (getMargin() != btScalar(0.))
		{
			btVector3 vecnorm = vec;
			if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
			{
				vecnorm.setValue(btScalar(-1.), btScalar(-1.), btScalar(-1.));
			}
			vecnorm.normalize();
			supVertex += getMargin() * vecnorm;
		}
		return supVertex;
	}

	//use box inertia
	//	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;

	int getUpAxis() const
	{
		return m_upAxis;
	}

	virtual btVector3 getAnisotropicRollingFrictionDirection() const
	{
		btVector3 aniDir(0, 0, 0);
		aniDir[getUpAxis()] = 1;
		return aniDir;
	}

	virtual btScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		btVector3 oldMargin(getMargin(), getMargin(), getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
		btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		btConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
	}

	//debugging
	virtual const char* getName() const
	{
		return "CylinderY";
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

class btCylinderShapeX : public btCylinderShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btCylinderShapeX(const btVector3& halfExtents)
		: btCylinderShape(halfExtents)
	{
		m_upAxis = 0;
	}

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		return CylinderLocalSupportX(getHalfExtentsWithoutMargin(), vec);
	}
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		for (int i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i] = CylinderLocalSupportX(getHalfExtentsWithoutMargin(), vectors[i]);
		}
	}

	//debugging
	virtual const char* getName() const
	{
		return "CylinderX";
	}

	virtual btScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getY();
	}
};

class btCylinderShapeZ : public btCylinderShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btCylinderShapeZ(const btVector3& halfExtents)
		: btCylinderShape(halfExtents)
	{
		m_upAxis = 2;
	}

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		return CylinderLocalSupportZ(getHalfExtentsWithoutMargin(), vec);
	}
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		for (int i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i] = CylinderLocalSupportZ(getHalfExtentsWithoutMargin(), vectors[i]);
		}
	}

	//debugging
	virtual const char* getName() const
	{
		return "CylinderZ";
	}

	virtual btScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCylinderShapeData
{
	btConvexInternalShapeData m_convexInternalShapeData;

	int m_upAxis;

	char m_padding[4];
};

SIMD_FORCE_INLINE int btCylinderShape::calculateSerializeBufferSize() const
{
	return sizeof(btCylinderShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btCylinderShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btCylinderShapeData* shapeData = (btCylinderShapeData*)dataBuffer;

	btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	shapeData->m_upAxis = m_upAxis;

	// Fill padding with zeros to appease msan.
	shapeData->m_padding[0] = 0;
	shapeData->m_padding[1] = 0;
	shapeData->m_padding[2] = 0;
	shapeData->m_padding[3] = 0;

	return "btCylinderShapeData";
}

#endif  //BT_CYLINDER_MINKOWSKI_H
