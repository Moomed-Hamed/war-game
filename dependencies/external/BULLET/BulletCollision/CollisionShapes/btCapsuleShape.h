#ifndef BT_CAPSULE_SHAPE_H
#define BT_CAPSULE_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"  // for the types

///The btCapsuleShape represents a capsule around the Y axis, there is also the btCapsuleShapeX aligned around the X axis and btCapsuleShapeZ around the Z axis.
///The total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
///The btCapsuleShape is a convex hull of two spheres. The btMultiSphereShape is a more general collision shape that takes the convex hull of multiple sphere, so it can also represent a capsule when just using two spheres.
ATTRIBUTE_ALIGNED16(class)
btCapsuleShape : public btConvexInternalShape
{
protected:
	int m_upAxis;

protected:
	///only used for btCapsuleShapeZ and btCapsuleShapeX subclasses.
	btCapsuleShape() : btConvexInternalShape() { m_shapeType = CAPSULE_SHAPE_PROXYTYPE; };

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btCapsuleShape(btScalar radius, btScalar height) : btConvexInternalShape()
	{
		m_collisionMargin = radius;
		m_shapeType = CAPSULE_SHAPE_PROXYTYPE;
		m_upAxis = 1;
		m_implicitShapeDimensions.setValue(radius, 0.5f * height, radius);
	}

	///CollisionShape Interface
	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		//as an approximation, take the inertia of the box that bounds the spheres

		btTransform ident;
		ident.setIdentity();

		btScalar radius = getRadius();

		btVector3 halfExtents(radius, radius, radius);
		halfExtents[getUpAxis()] += getHalfHeight();

		btScalar lx = btScalar(2.) * (halfExtents[0]);
		btScalar ly = btScalar(2.) * (halfExtents[1]);
		btScalar lz = btScalar(2.) * (halfExtents[2]);
		const btScalar x2 = lx * lx;
		const btScalar y2 = ly * ly;
		const btScalar z2 = lz * lz;
		const btScalar scaledmass = mass * btScalar(.08333333);

		inertia[0] = scaledmass * (y2 + z2);
		inertia[1] = scaledmass * (x2 + z2);
		inertia[2] = scaledmass * (x2 + y2);
	}

	/// btConvexShape Interface
	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec0) const
	{
		btVector3 supVec(0, 0, 0);

		btScalar maxDot(btScalar(-BT_LARGE_FLOAT));

		btVector3 vec = vec0;
		btScalar lenSqr = vec.length2();
		if (lenSqr < btScalar(0.0001))
		{
			vec.setValue(1, 0, 0);
		}
		else
		{
			btScalar rlen = btScalar(1.) / btSqrt(lenSqr);
			vec *= rlen;
		}

		btVector3 vtx;
		btScalar newDot;

		{
			btVector3 pos(0, 0, 0);
			pos[getUpAxis()] = getHalfHeight();

			vtx = pos;
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}
		{
			btVector3 pos(0, 0, 0);
			pos[getUpAxis()] = -getHalfHeight();

			vtx = pos;
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}

		return supVec;
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		for (int j = 0; j < numVectors; j++)
		{
			btScalar maxDot(btScalar(-BT_LARGE_FLOAT));
			const btVector3& vec = vectors[j];

			btVector3 vtx;
			btScalar newDot;
			{
				btVector3 pos(0, 0, 0);
				pos[getUpAxis()] = getHalfHeight();
				vtx = pos;
				newDot = vec.dot(vtx);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					supportVerticesOut[j] = vtx;
				}
			}
			{
				btVector3 pos(0, 0, 0);
				pos[getUpAxis()] = -getHalfHeight();
				vtx = pos;
				newDot = vec.dot(vtx);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					supportVerticesOut[j] = vtx;
				}
			}
		}
	}

	virtual void setMargin(btScalar collisionMargin)
	{
		//don't override the margin for capsules, their entire radius == margin
		(void)collisionMargin;
	}

	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btVector3 halfExtents(getRadius(), getRadius(), getRadius());
		halfExtents[m_upAxis] = getRadius() + getHalfHeight();
		btMatrix3x3 abs_b = t.getBasis().absolute();
		btVector3 center = t.getOrigin();
		btVector3 extent = halfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);

		aabbMin = center - extent;
		aabbMax = center + extent;
	}

	virtual const char* getName() const
	{
		return "CapsuleShape";
	}

	int getUpAxis() const
	{
		return m_upAxis;
	}

	btScalar getRadius() const
	{
		int radiusAxis = (m_upAxis + 2) % 3;
		return m_implicitShapeDimensions[radiusAxis];
	}

	btScalar getHalfHeight() const
	{
		return m_implicitShapeDimensions[m_upAxis];
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		btVector3 unScaledImplicitShapeDimensions = m_implicitShapeDimensions / m_localScaling;
		btConvexInternalShape::setLocalScaling(scaling);
		m_implicitShapeDimensions = (unScaledImplicitShapeDimensions * scaling);
		//update m_collisionMargin, since entire radius==margin
		int radiusAxis = (m_upAxis + 2) % 3;
		m_collisionMargin = m_implicitShapeDimensions[radiusAxis];
	}

	virtual btVector3 getAnisotropicRollingFrictionDirection() const
	{
		btVector3 aniDir(0, 0, 0);
		aniDir[getUpAxis()] = 1;
		return aniDir;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

	SIMD_FORCE_INLINE void deSerializeFloat(struct btCapsuleShapeData * dataBuffer);
};

///btCapsuleShapeX represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btCapsuleShapeX : public btCapsuleShape
{
public:
	btCapsuleShapeX(btScalar radius, btScalar height)
	{
		m_collisionMargin = radius;
		m_upAxis = 0;
		m_implicitShapeDimensions.setValue(0.5f * height, radius, radius);
	}

	//debugging
	virtual const char* getName() const
	{
		return "CapsuleX";
	}
};

///btCapsuleShapeZ represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btCapsuleShapeZ : public btCapsuleShape
{
public:
	btCapsuleShapeZ(btScalar radius, btScalar height)
	{
		m_collisionMargin = radius;
		m_upAxis = 2;
		m_implicitShapeDimensions.setValue(radius, radius, 0.5f * height);
	}

	//debugging
	virtual const char* getName() const
	{
		return "CapsuleZ";
	}
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCapsuleShapeData
{
	btConvexInternalShapeData m_convexInternalShapeData;

	int m_upAxis;

	char m_padding[4];
};

SIMD_FORCE_INLINE int btCapsuleShape::calculateSerializeBufferSize() const
{
	return sizeof(btCapsuleShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btCapsuleShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btCapsuleShapeData* shapeData = (btCapsuleShapeData*)dataBuffer;

	btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	shapeData->m_upAxis = m_upAxis;

	// Fill padding with zeros to appease msan.
	shapeData->m_padding[0] = 0;
	shapeData->m_padding[1] = 0;
	shapeData->m_padding[2] = 0;
	shapeData->m_padding[3] = 0;

	return "btCapsuleShapeData";
}

SIMD_FORCE_INLINE void btCapsuleShape::deSerializeFloat(btCapsuleShapeData* dataBuffer)
{
	m_implicitShapeDimensions.deSerializeFloat(dataBuffer->m_convexInternalShapeData.m_implicitShapeDimensions);
	m_collisionMargin = dataBuffer->m_convexInternalShapeData.m_collisionMargin;
	m_localScaling.deSerializeFloat(dataBuffer->m_convexInternalShapeData.m_localScaling);
	//it is best to already pre-allocate the matching btCapsuleShape*(X/Z) version to match m_upAxis
	m_upAxis = dataBuffer->m_upAxis;
}

#endif  //BT_CAPSULE_SHAPE_H
