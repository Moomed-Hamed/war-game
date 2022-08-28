#ifndef BT_COLLISION_SHAPE_H
#define BT_COLLISION_SHAPE_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btSerializer.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"  //for the shape types
class btSerializer;

/*
  Make sure this dummy function never changes so that it
  can be used by probes that are checking whether the
  library is actually installed.
*/
//extern "C"
//{
//	void btBulletCollisionProbe();
//
//	void btBulletCollisionProbe() {}
//}

// parser needs * with the name
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCollisionShapeData
{
	char* m_name;
	int m_shapeType;
	char m_padding[4];
};

///The btCollisionShape class provides an interface for collision shapes that can be shared among btCollisionObjects.
ATTRIBUTE_ALIGNED16(class)
btCollisionShape
{
protected:
	int m_shapeType;
	void* m_userPointer;
	int m_userIndex;
	int m_userIndex2;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btCollisionShape() : m_shapeType(INVALID_SHAPE_PROXYTYPE), m_userPointer(0), m_userIndex(-1), m_userIndex2(-1)
	{
	}

	virtual ~btCollisionShape()
	{
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const = 0;

	virtual void getBoundingSphere(btVector3 & center, btScalar & radius) const
	{
		btTransform tr;
		tr.setIdentity();
		btVector3 aabbMin, aabbMax;

		getAabb(tr, aabbMin, aabbMax);

		radius = (aabbMax - aabbMin).length() * btScalar(0.5);
		center = (aabbMin + aabbMax) * btScalar(0.5);
	}

	///getAngularMotionDisc returns the maximum radius needed for Conservative Advancement to handle time-of-impact with rotations.
	virtual btScalar getAngularMotionDisc() const
	{
		///@todo cache this value, to improve performance
		btVector3 center;
		btScalar disc;
		getBoundingSphere(center, disc);
		disc += (center).length();
		return disc;
	}

	virtual btScalar getContactBreakingThreshold(btScalar defaultContactThreshold) const
	{
		return getAngularMotionDisc() * defaultContactThreshold;
	}

	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	void calculateTemporalAabb(const btTransform& curTrans, const btVector3& linvel, const btVector3& angvel, btScalar timeStep, btVector3& temporalAabbMin, btVector3& temporalAabbMax) const
	{
		//start with static aabb
		getAabb(curTrans, temporalAabbMin, temporalAabbMax);

		btScalar temporalAabbMaxx = temporalAabbMax.getX();
		btScalar temporalAabbMaxy = temporalAabbMax.getY();
		btScalar temporalAabbMaxz = temporalAabbMax.getZ();
		btScalar temporalAabbMinx = temporalAabbMin.getX();
		btScalar temporalAabbMiny = temporalAabbMin.getY();
		btScalar temporalAabbMinz = temporalAabbMin.getZ();

		// add linear motion
		btVector3 linMotion = linvel * timeStep;
		///@todo: simd would have a vector max/min operation, instead of per-element access
		if (linMotion.x() > btScalar(0.))
			temporalAabbMaxx += linMotion.x();
		else
			temporalAabbMinx += linMotion.x();
		if (linMotion.y() > btScalar(0.))
			temporalAabbMaxy += linMotion.y();
		else
			temporalAabbMiny += linMotion.y();
		if (linMotion.z() > btScalar(0.))
			temporalAabbMaxz += linMotion.z();
		else
			temporalAabbMinz += linMotion.z();

		//add conservative angular motion
		btScalar angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
		btVector3 angularMotion3d(angularMotion, angularMotion, angularMotion);
		temporalAabbMin = btVector3(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
		temporalAabbMax = btVector3(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);

		temporalAabbMin -= angularMotion3d;
		temporalAabbMax += angularMotion3d;
	}

	SIMD_FORCE_INLINE bool isPolyhedral() const
	{
		return btBroadphaseProxy::isPolyhedral(getShapeType());
	}

	SIMD_FORCE_INLINE bool isConvex2d() const
	{
		return btBroadphaseProxy::isConvex2d(getShapeType());
	}

	SIMD_FORCE_INLINE bool isConvex() const
	{
		return btBroadphaseProxy::isConvex(getShapeType());
	}
	SIMD_FORCE_INLINE bool isNonMoving() const
	{
		return btBroadphaseProxy::isNonMoving(getShapeType());
	}
	SIMD_FORCE_INLINE bool isConcave() const
	{
		return btBroadphaseProxy::isConcave(getShapeType());
	}
	SIMD_FORCE_INLINE bool isCompound() const
	{
		return btBroadphaseProxy::isCompound(getShapeType());
	}

	SIMD_FORCE_INLINE bool isSoftBody() const
	{
		return btBroadphaseProxy::isSoftBody(getShapeType());
	}

	///isInfinite is used to catch simulation error (aabb check)
	SIMD_FORCE_INLINE bool isInfinite() const
	{
		return btBroadphaseProxy::isInfinite(getShapeType());
	}

#ifndef __SPU__
	virtual void setLocalScaling(const btVector3& scaling) = 0;
	virtual const btVector3& getLocalScaling() const = 0;
	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const = 0;

	//debugging support
	virtual const char* getName() const = 0;
#endif  //__SPU__

	int getShapeType() const
	{
		return m_shapeType;
	}

	///the getAnisotropicRollingFrictionDirection can be used in combination with setAnisotropicFriction
	///See Bullet/Demos/RollingFrictionDemo for an example
	virtual btVector3 getAnisotropicRollingFrictionDirection() const
	{
		return btVector3(1, 1, 1);
	}
	virtual void setMargin(btScalar margin) = 0;
	virtual btScalar getMargin() const = 0;

	///optional user data pointer
	void setUserPointer(void* userPtr)
	{
		m_userPointer = userPtr;
	}

	void* getUserPointer() const
	{
		return m_userPointer;
	}
	void setUserIndex(int index)
	{
		m_userIndex = index;
	}

	int getUserIndex() const
	{
		return m_userIndex;
	}

	void setUserIndex2(int index)
	{
		m_userIndex2 = index;
	}

	int getUserIndex2() const
	{
		return m_userIndex2;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const
	{
		btCollisionShapeData* shapeData = (btCollisionShapeData*)dataBuffer;
		char* name = (char*)serializer->findNameForPointer(this);
		shapeData->m_name = (char*)serializer->getUniquePointer(name);
		if (shapeData->m_name)
		{
			serializer->serializeName(name);
		}
		shapeData->m_shapeType = m_shapeType;

		// Fill padding with zeros to appease msan.
		memset(shapeData->m_padding, 0, sizeof(shapeData->m_padding));

		return "btCollisionShapeData";
	}

	virtual void serializeSingleShape(btSerializer * serializer) const
	{
		int len = calculateSerializeBufferSize();
		btChunk* chunk = serializer->allocate(len, 1);
		const char* structType = serialize(chunk->m_oldPtr, serializer);
		serializer->finalizeChunk(chunk, structType, BT_SHAPE_CODE, (void*)this);
	}
};

SIMD_FORCE_INLINE int btCollisionShape::calculateSerializeBufferSize() const
{
	return sizeof(btCollisionShapeData);
}

#endif  //BT_COLLISION_SHAPE_H
