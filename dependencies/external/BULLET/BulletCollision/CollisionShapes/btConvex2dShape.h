#ifndef BT_CONVEX_2D_SHAPE_H
#define BT_CONVEX_2D_SHAPE_H

#include "BulletCollision/CollisionShapes/btConvexShape.h"

///The btConvex2dShape allows to use arbitrary convex shapes as 2d convex shapes, with the Z component assumed to be 0.
///For 2d boxes, the btBox2dShape is recommended.
ATTRIBUTE_ALIGNED16(class)
btConvex2dShape : public btConvexShape
{
	btConvexShape* m_childConvexShape;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btConvex2dShape(btConvexShape * convexChildShape) : btConvexShape(), m_childConvexShape(convexChildShape)
	{
		m_shapeType = CONVEX_2D_SHAPE_PROXYTYPE;
	}

	virtual ~btConvex2dShape(){};

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		return m_childConvexShape->localGetSupportingVertexWithoutMargin(vec);
	}

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
	{
		return m_childConvexShape->localGetSupportingVertex(vec);
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		m_childConvexShape->batchedUnitVectorGetSupportingVertexWithoutMargin(vectors, supportVerticesOut, numVectors);
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		///this linear upscaling is not realistic, but we don't deal with large mass ratios...
		m_childConvexShape->calculateLocalInertia(mass, inertia);
	}

	btConvexShape* getChildShape()
	{
		return m_childConvexShape;
	}

	const btConvexShape* getChildShape() const
	{
		return m_childConvexShape;
	}

	virtual const char* getName() const
	{
		return "Convex2dShape";
	}

	///////////////////////////

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		m_childConvexShape->getAabb(t, aabbMin, aabbMax);
	}

	virtual void getAabbSlow(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		m_childConvexShape->getAabbSlow(t, aabbMin, aabbMax);
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_childConvexShape->setLocalScaling(scaling);
	}
	virtual const btVector3& getLocalScaling() const
	{
		return m_childConvexShape->getLocalScaling();
	}

	virtual void setMargin(btScalar margin)
	{
		m_childConvexShape->setMargin(margin);
	}
	virtual btScalar getMargin() const
	{
		return m_childConvexShape->getMargin();
	}

	virtual int getNumPreferredPenetrationDirections() const
	{
		return m_childConvexShape->getNumPreferredPenetrationDirections();
	}

	virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const
	{
		m_childConvexShape->getPreferredPenetrationDirection(index, penetrationVector);
	}
};

#endif  //BT_CONVEX_2D_SHAPE_H
