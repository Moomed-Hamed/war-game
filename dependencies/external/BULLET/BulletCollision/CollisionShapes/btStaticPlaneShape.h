#ifndef BT_STATIC_PLANE_SHAPE_H
#define BT_STATIC_PLANE_SHAPE_H

#include "btConcaveShape.h"

///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
ATTRIBUTE_ALIGNED16(class)
btStaticPlaneShape : public btConcaveShape
{
protected:
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;

	btVector3 m_planeNormal;
	btScalar m_planeConstant;
	btVector3 m_localScaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btStaticPlaneShape(const btVector3& planeNormal, btScalar planeConstant)
		: btConcaveShape(), m_planeNormal(planeNormal.normalized()), m_planeConstant(planeConstant), m_localScaling(btScalar(1.), btScalar(1.), btScalar(1.))
	{
		m_shapeType = STATIC_PLANE_PROXYTYPE;
	}

	virtual ~btStaticPlaneShape(){};

	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		(void)t;

		aabbMin.setValue(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT));
		aabbMax.setValue(btScalar( BT_LARGE_FLOAT), btScalar( BT_LARGE_FLOAT), btScalar( BT_LARGE_FLOAT));
	}

	virtual void processAllTriangles(btTriangleCallback * callback, const btVector3& aabbMin, const btVector3& aabbMax) const
	{
		btVector3 halfExtents = (aabbMax - aabbMin) * btScalar(0.5);
		btScalar radius = halfExtents.length();
		btVector3 center = (aabbMax + aabbMin) * btScalar(0.5);

		//this is where the triangles are generated, given AABB and plane equation (normal/constant)

		btVector3 tangentDir0, tangentDir1;

		//tangentDir0/tangentDir1 can be precalculated
		btPlaneSpace1(m_planeNormal, tangentDir0, tangentDir1);

		btVector3 projectedCenter = center - (m_planeNormal.dot(center) - m_planeConstant) * m_planeNormal;

		btVector3 triangle[3];
		triangle[0] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;
		triangle[1] = projectedCenter + tangentDir0 * radius - tangentDir1 * radius;
		triangle[2] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;

		callback->processTriangle(triangle, 0, 0);

		triangle[0] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;
		triangle[1] = projectedCenter - tangentDir0 * radius + tangentDir1 * radius;
		triangle[2] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;

		callback->processTriangle(triangle, 0, 1);
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		(void)mass;

		//moving concave objects not supported

		inertia.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
	}
	virtual const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	const btVector3& getPlaneNormal() const
	{
		return m_planeNormal;
	}

	const btScalar& getPlaneConstant() const
	{
		return m_planeConstant;
	}

	//debugging
	virtual const char* getName() const { return "STATICPLANE"; }

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btStaticPlaneShapeData
{
	btCollisionShapeData m_collisionShapeData;

	btVector3FloatData m_localScaling;
	btVector3FloatData m_planeNormal;
	float m_planeConstant;
	char m_pad[4];
};

SIMD_FORCE_INLINE int btStaticPlaneShape::calculateSerializeBufferSize() const
{
	return sizeof(btStaticPlaneShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btStaticPlaneShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btStaticPlaneShapeData* planeData = (btStaticPlaneShapeData*)dataBuffer;
	btCollisionShape::serialize(&planeData->m_collisionShapeData, serializer);

	m_localScaling.serializeFloat(planeData->m_localScaling);
	m_planeNormal.serializeFloat(planeData->m_planeNormal);
	planeData->m_planeConstant = float(m_planeConstant);

	// Fill padding with zeros to appease msan.
	planeData->m_pad[0] = 0;
	planeData->m_pad[1] = 0;
	planeData->m_pad[2] = 0;
	planeData->m_pad[3] = 0;

	return "btStaticPlaneShapeData";
}

#endif  //BT_STATIC_PLANE_SHAPE_H
