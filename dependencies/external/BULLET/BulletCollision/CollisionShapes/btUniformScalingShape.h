#ifndef BT_UNIFORM_SCALING_SHAPE_H
#define BT_UNIFORM_SCALING_SHAPE_H

#include "btConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"  // for the types

///The btUniformScalingShape allows to re-use uniform scaled instances of btConvexShape in a memory efficient way.
///Istead of using btUniformScalingShape, it is better to use the non-uniform setLocalScaling method on convex shapes that implement it.
ATTRIBUTE_ALIGNED16(class)
btUniformScalingShape : public btConvexShape
{
	btConvexShape* m_childConvexShape;

	btScalar m_uniformScalingFactor;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btUniformScalingShape(btConvexShape * convexChildShape, btScalar uniformScalingFactor)
		: btConvexShape(), m_childConvexShape(convexChildShape), m_uniformScalingFactor(uniformScalingFactor)
	{
		m_shapeType = UNIFORM_SCALING_SHAPE_PROXYTYPE;
	}

	virtual ~btUniformScalingShape(){};

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		btVector3 tmpVertex;
		tmpVertex = m_childConvexShape->localGetSupportingVertexWithoutMargin(vec);
		return tmpVertex * m_uniformScalingFactor;
	}

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
	{
		btVector3 tmpVertex;
		tmpVertex = m_childConvexShape->localGetSupportingVertex(vec);
		return tmpVertex * m_uniformScalingFactor;
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		m_childConvexShape->batchedUnitVectorGetSupportingVertexWithoutMargin(vectors, supportVerticesOut, numVectors);
		int i;
		for (i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i] = supportVerticesOut[i] * m_uniformScalingFactor;
		}
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		///this linear upscaling is not realistic, but we don't deal with large mass ratios...
		btVector3 tmpInertia;
		m_childConvexShape->calculateLocalInertia(mass, tmpInertia);
		inertia = tmpInertia * m_uniformScalingFactor;
	}

	btScalar getUniformScalingFactor() const
	{
		return m_uniformScalingFactor;
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
		return "UniformScalingShape";
	}

	///////////////////////////

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btTransform& trans, btVector3& aabbMin, btVector3& aabbMax) const
	{
		getAabbSlow(trans, aabbMin, aabbMax);
	}

	virtual void getAabbSlow(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btVector3 _directions[] =
			{
				btVector3(1., 0., 0.),
				btVector3(0., 1., 0.),
				btVector3(0., 0., 1.),
				btVector3(-1., 0., 0.),
				btVector3(0., -1., 0.),
				btVector3(0., 0., -1.)};

		btVector3 _supporting[] =
			{
				btVector3(0., 0., 0.),
				btVector3(0., 0., 0.),
				btVector3(0., 0., 0.),
				btVector3(0., 0., 0.),
				btVector3(0., 0., 0.),
				btVector3(0., 0., 0.)};

		for (int i = 0; i < 6; i++)
		{
			_directions[i] = _directions[i] * t.getBasis();
		}

		batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

		btVector3 aabbMin1(0, 0, 0), aabbMax1(0, 0, 0);

		for (int i = 0; i < 3; ++i)
		{
			aabbMax1[i] = t(_supporting[i])[i];
			aabbMin1[i] = t(_supporting[i + 3])[i];
		}
		btVector3 marginVec(getMargin(), getMargin(), getMargin());
		aabbMin = aabbMin1 - marginVec;
		aabbMax = aabbMax1 + marginVec;
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
		return m_childConvexShape->getMargin() * m_uniformScalingFactor;
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

#endif  //BT_UNIFORM_SCALING_SHAPE_H
