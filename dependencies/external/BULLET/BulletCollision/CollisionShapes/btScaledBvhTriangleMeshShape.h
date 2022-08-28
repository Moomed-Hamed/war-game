#ifndef BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H

#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"

class btScaledTriangleCallback : public btTriangleCallback
{
	btTriangleCallback* m_originalCallback;

	btVector3 m_localScaling;

public:
	btScaledTriangleCallback(btTriangleCallback* originalCallback, const btVector3& localScaling)
		: m_originalCallback(originalCallback),
		  m_localScaling(localScaling)
	{
	}

	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
	{
		btVector3 newTriangle[3];
		newTriangle[0] = triangle[0] * m_localScaling;
		newTriangle[1] = triangle[1] * m_localScaling;
		newTriangle[2] = triangle[2] * m_localScaling;
		m_originalCallback->processTriangle(&newTriangle[0], partId, triangleIndex);
	}
};

///The btScaledBvhTriangleMeshShape allows to instance a scaled version of an existing btBvhTriangleMeshShape.
///Note that each btBvhTriangleMeshShape still can have its own local scaling, independent from this btScaledBvhTriangleMeshShape 'localScaling'
ATTRIBUTE_ALIGNED16(class)
btScaledBvhTriangleMeshShape : public btConcaveShape
{
	btVector3 m_localScaling;

	btBvhTriangleMeshShape* m_bvhTriMeshShape;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btScaledBvhTriangleMeshShape(btBvhTriangleMeshShape * childShape, const btVector3& localScaling)
		: m_localScaling(localScaling), m_bvhTriMeshShape(childShape)
	{
		m_shapeType = SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	}

	virtual ~btScaledBvhTriangleMeshShape(){};

	virtual void getAabb(const btTransform& trans, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btVector3 localAabbMin = m_bvhTriMeshShape->getLocalAabbMin();
		btVector3 localAabbMax = m_bvhTriMeshShape->getLocalAabbMax();

		btVector3 tmpLocalAabbMin = localAabbMin * m_localScaling;
		btVector3 tmpLocalAabbMax = localAabbMax * m_localScaling;

		localAabbMin[0] = (m_localScaling.getX() >= 0.) ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
		localAabbMin[1] = (m_localScaling.getY() >= 0.) ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
		localAabbMin[2] = (m_localScaling.getZ() >= 0.) ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];
		localAabbMax[0] = (m_localScaling.getX() <= 0.) ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
		localAabbMax[1] = (m_localScaling.getY() <= 0.) ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
		localAabbMax[2] = (m_localScaling.getZ() <= 0.) ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];

		btVector3 localHalfExtents = btScalar(0.5) * (localAabbMax - localAabbMin);
		btScalar margin = m_bvhTriMeshShape->getMargin();
		localHalfExtents += btVector3(margin, margin, margin);
		btVector3 localCenter = btScalar(0.5) * (localAabbMax + localAabbMin);

		btMatrix3x3 abs_b = trans.getBasis().absolute();

		btVector3 center = trans(localCenter);

		btVector3 extent = localHalfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
		aabbMin = center - extent;
		aabbMax = center + extent;
	}
	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
	}
	virtual const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}
	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		///don't make this a movable object!
		//	btAssert(0);
	}

	virtual void processAllTriangles(btTriangleCallback * callback, const btVector3& aabbMin, const btVector3& aabbMax) const
	{
		btScaledTriangleCallback scaledCallback(callback, m_localScaling);

		btVector3 invLocalScaling(1.f / m_localScaling.getX(), 1.f / m_localScaling.getY(), 1.f / m_localScaling.getZ());
		btVector3 scaledAabbMin, scaledAabbMax;

		///support negative scaling
		scaledAabbMin[0] = m_localScaling.getX() >= 0. ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
		scaledAabbMin[1] = m_localScaling.getY() >= 0. ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
		scaledAabbMin[2] = m_localScaling.getZ() >= 0. ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];
		scaledAabbMin[3] = 0.f;

		scaledAabbMax[0] = m_localScaling.getX() <= 0. ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
		scaledAabbMax[1] = m_localScaling.getY() <= 0. ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
		scaledAabbMax[2] = m_localScaling.getZ() <= 0. ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];
		scaledAabbMax[3] = 0.f;

		m_bvhTriMeshShape->processAllTriangles(&scaledCallback, scaledAabbMin, scaledAabbMax);
	}

	btBvhTriangleMeshShape* getChildShape()
	{
		return m_bvhTriMeshShape;
	}

	const btBvhTriangleMeshShape* getChildShape() const
	{
		return m_bvhTriMeshShape;
	}

	//debugging
	virtual const char* getName() const { return "SCALEDBVHTRIANGLEMESH"; }

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btScaledTriangleMeshShapeData
{
	btTriangleMeshShapeData m_trimeshShapeData;

	btVector3FloatData m_localScaling;
};

SIMD_FORCE_INLINE int btScaledBvhTriangleMeshShape::calculateSerializeBufferSize() const
{
	return sizeof(btScaledTriangleMeshShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btScaledBvhTriangleMeshShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btScaledTriangleMeshShapeData* scaledMeshData = (btScaledTriangleMeshShapeData*)dataBuffer;
	m_bvhTriMeshShape->serialize(&scaledMeshData->m_trimeshShapeData, serializer);
	scaledMeshData->m_trimeshShapeData.m_collisionShapeData.m_shapeType = SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	m_localScaling.serializeFloat(scaledMeshData->m_localScaling);
	return "btScaledTriangleMeshShapeData";
}

#endif  //BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
