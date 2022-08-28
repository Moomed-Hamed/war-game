#ifndef BT_MINKOWSKI_SUM_SHAPE_H
#define BT_MINKOWSKI_SUM_SHAPE_H

#include "btConvexInternalShape.h"

/// The btMinkowskiSumShape is only for advanced users. This shape represents implicit based minkowski sum of two convex implicit shapes.
ATTRIBUTE_ALIGNED16(class)
btMinkowskiSumShape : public btConvexInternalShape
{
	btTransform m_transA;
	btTransform m_transB;
	const btConvexShape* m_shapeA;
	const btConvexShape* m_shapeB;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btMinkowskiSumShape(const btConvexShape* shapeA, const btConvexShape* shapeB)
		: btConvexInternalShape(),
		  m_shapeA(shapeA),
		  m_shapeB(shapeB)
	{
		m_shapeType = MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
		m_transA.setIdentity();
		m_transB.setIdentity();
	}

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		btVector3 supVertexA = m_transA(m_shapeA->localGetSupportingVertexWithoutMargin(vec * m_transA.getBasis()));
		btVector3 supVertexB = m_transB(m_shapeB->localGetSupportingVertexWithoutMargin(-vec * m_transB.getBasis()));
		return supVertexA - supVertexB;
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		///@todo: could make recursive use of batching. probably this shape is not used frequently.
		for (int i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i] = localGetSupportingVertexWithoutMargin(vectors[i]);
		}
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		(void)mass;
		//inertia of the AABB of the Minkowski sum
		btTransform identity;
		identity.setIdentity();
		btVector3 aabbMin, aabbMax;
		getAabb(identity, aabbMin, aabbMax);

		btVector3 halfExtents = (aabbMax - aabbMin) * btScalar(0.5);

		btScalar margin = getMargin();

		btScalar lx = btScalar(2.) * (halfExtents.x() + margin);
		btScalar ly = btScalar(2.) * (halfExtents.y() + margin);
		btScalar lz = btScalar(2.) * (halfExtents.z() + margin);
		const btScalar x2 = lx * lx;
		const btScalar y2 = ly * ly;
		const btScalar z2 = lz * lz;
		const btScalar scaledmass = mass * btScalar(0.08333333);

		inertia = scaledmass * (btVector3(y2 + z2, x2 + z2, x2 + y2));
	}

	void setTransformA(const btTransform& transA) { m_transA = transA; }
	void setTransformB(const btTransform& transB) { m_transB = transB; }

	const btTransform& getTransformA() const { return m_transA; }
	const btTransform& GetTransformB() const { return m_transB; }

	virtual btScalar getMargin() const
	{
		return m_shapeA->getMargin() + m_shapeB->getMargin();
	}

	const btConvexShape* getShapeA() const { return m_shapeA; }
	const btConvexShape* getShapeB() const { return m_shapeB; }

	virtual const char* getName() const
	{
		return "MinkowskiSum";
	}
};

#endif  //BT_MINKOWSKI_SUM_SHAPE_H
