#ifndef BT_SPHERE_MINKOWSKI_H
#define BT_SPHERE_MINKOWSKI_H

#include "btConvexInternalShape.h"

///The btSphereShape implements an implicit sphere, centered around a local origin with radius.
ATTRIBUTE_ALIGNED16(class)
btSphereShape : public btConvexInternalShape

{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btSphereShape(btScalar radius) : btConvexInternalShape()
	{
		m_shapeType = SPHERE_SHAPE_PROXYTYPE;
		m_localScaling.setValue(1.0, 1.0, 1.0);
		m_implicitShapeDimensions.setZero();
		m_implicitShapeDimensions.setX(radius);
		m_collisionMargin = radius;
		m_padding = 0;
	}

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
	{
		btVector3 supVertex;
		supVertex = localGetSupportingVertexWithoutMargin(vec);

		btVector3 vecnorm = vec;
		if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
		{
			vecnorm.setValue(btScalar(-1.), btScalar(-1.), btScalar(-1.));
		}
		vecnorm.normalize();
		supVertex += getMargin() * vecnorm;
		return supVertex;
	}
	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		(void)vec;
		return btVector3(btScalar(0.), btScalar(0.), btScalar(0.));
	}
	//notice that the vectors should be unit length
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		(void)vectors;

		for (int i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i].setValue(btScalar(0.), btScalar(0.), btScalar(0.));
		}
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		btScalar elem = btScalar(0.4) * mass * getMargin() * getMargin();
		inertia.setValue(elem, elem, elem);
	}


	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		// this function is broken due to scaling
		const btVector3& center = t.getOrigin();
		btVector3 extent(getMargin(), getMargin(), getMargin());
		aabbMin = center - extent;
		aabbMax = center + extent;
	}

	btScalar getRadius() const { return m_implicitShapeDimensions.getX() * m_localScaling.getX(); }

	void setUnscaledRadius(btScalar radius)
	{
		m_implicitShapeDimensions.setX(radius);
		btConvexInternalShape::setMargin(radius);
	}

	//debugging
	virtual const char* getName() const { return "SPHERE"; }

	virtual void setMargin(btScalar margin)
	{
		btConvexInternalShape::setMargin(margin);
	}
	virtual btScalar getMargin() const
	{
		//to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		//this means, non-uniform scaling is not supported anymore
		return getRadius();
	}
};

#endif  //BT_SPHERE_MINKOWSKI_H
