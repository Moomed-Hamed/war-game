#ifndef BT_CONVEX_INTERNAL_SHAPE_H
#define BT_CONVEX_INTERNAL_SHAPE_H

#include "btConvexShape.h"
#include "LinearMath/btAabbUtil2.h"

///The btConvexInternalShape is an internal base class, shared by most convex shape implementations.
///The btConvexInternalShape uses a default collision margin set to CONVEX_DISTANCE_MARGIN.
///This collision margin used by Gjk and some other algorithms, see also btCollisionMargin.h
///Note that when creating small shapes (derived from btConvexInternalShape),
///you need to make sure to set a smaller collision margin, using the 'setMargin' API
///There is a automatic mechanism 'setSafeMargin' used by btBoxShape and btCylinderShape
ATTRIBUTE_ALIGNED16(class)
btConvexInternalShape : public btConvexShape
{
protected:
	//local scaling. collisionMargin is not scaled !
	btVector3 m_localScaling;

	btVector3 m_implicitShapeDimensions;

	btScalar m_collisionMargin;

	btScalar m_padding;

	btConvexInternalShape()
		: m_localScaling(btScalar(1.), btScalar(1.), btScalar(1.)),
		  m_collisionMargin(CONVEX_DISTANCE_MARGIN)
	{
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	virtual ~btConvexInternalShape()
	{
	}

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
	{
		btVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

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

	const btVector3& getImplicitShapeDimensions() const
	{
		return m_implicitShapeDimensions;
	}

	///warning: use setImplicitShapeDimensions with care
	///changing a collision shape while the body is in the world is not recommended,
	///it is best to remove the body from the world, then make the change, and re-add it
	///alternatively flush the contact points, see documentation for 'cleanProxyFromPairs'
	void setImplicitShapeDimensions(const btVector3& dimensions)
	{
		m_implicitShapeDimensions = dimensions;
	}

	void setSafeMargin(btScalar minDimension, btScalar defaultMarginMultiplier = 0.1f)
	{
		btScalar safeMargin = defaultMarginMultiplier * minDimension;
		if (safeMargin < getMargin())
		{
			setMargin(safeMargin);
		}
	}
	void setSafeMargin(const btVector3& halfExtents, btScalar defaultMarginMultiplier = 0.1f)
	{
		//see http://code.google.com/p/bullet/issues/detail?id=349
		//this margin check could could be added to other collision shapes too,
		//or add some assert/warning somewhere
		btScalar minDimension = halfExtents[halfExtents.minAxis()];
		setSafeMargin(minDimension, defaultMarginMultiplier);
	}

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		getAabbSlow(t, aabbMin, aabbMax);
	}

	virtual void getAabbSlow(const btTransform& trans, btVector3& minAabb, btVector3& maxAabb) const
	{
		//use localGetSupportingVertexWithoutMargin?
		btScalar margin = getMargin();
		for (int i = 0; i < 3; i++)
		{
			btVector3 vec(btScalar(0.), btScalar(0.), btScalar(0.));
			vec[i] = btScalar(1.);

			btVector3 sv = localGetSupportingVertex(vec * trans.getBasis());

			btVector3 tmp = trans(sv);
			maxAabb[i] = tmp[i] + margin;
			vec[i] = btScalar(-1.);
			tmp = trans(localGetSupportingVertex(vec * trans.getBasis()));
			minAabb[i] = tmp[i] - margin;
		}
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling.absolute();
	}
	virtual const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	const btVector3& getLocalScalingNV() const
	{
		return m_localScaling;
	}

	virtual void setMargin(btScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual btScalar getMargin() const
	{
		return m_collisionMargin;
	}

	btScalar getMarginNV() const
	{
		return m_collisionMargin;
	}

	virtual int getNumPreferredPenetrationDirections() const
	{
		return 0;
	}

	virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const
	{
		(void)penetrationVector;
		(void)index;
		btAssert(0);
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btConvexInternalShapeData
{
	btCollisionShapeData m_collisionShapeData;

	btVector3FloatData m_localScaling;

	btVector3FloatData m_implicitShapeDimensions;

	float m_collisionMargin;

	int m_padding;
};

SIMD_FORCE_INLINE int btConvexInternalShape::calculateSerializeBufferSize() const
{
	return sizeof(btConvexInternalShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btConvexInternalShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btConvexInternalShapeData* shapeData = (btConvexInternalShapeData*)dataBuffer;
	btCollisionShape::serialize(&shapeData->m_collisionShapeData, serializer);

	m_implicitShapeDimensions.serializeFloat(shapeData->m_implicitShapeDimensions);
	m_localScaling.serializeFloat(shapeData->m_localScaling);
	shapeData->m_collisionMargin = float(m_collisionMargin);

	// Fill padding with zeros to appease msan.
	shapeData->m_padding = 0;

	return "btConvexInternalShapeData";
}

///btConvexInternalAabbCachingShape adds local aabb caching for convex shapes, to avoid expensive bounding box calculations
class btConvexInternalAabbCachingShape : public btConvexInternalShape
{
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;
	bool m_isLocalAabbValid;

protected:
	btConvexInternalAabbCachingShape()
		: btConvexInternalShape(),
		  m_localAabbMin(1, 1, 1),
		  m_localAabbMax(-1, -1, -1),
		  m_isLocalAabbValid(false)
	{
	}

	void setCachedLocalAabb(const btVector3& aabbMin, const btVector3& aabbMax)
	{
		m_isLocalAabbValid = true;
		m_localAabbMin = aabbMin;
		m_localAabbMax = aabbMax;
	}

	inline void getCachedLocalAabb(btVector3& aabbMin, btVector3& aabbMax) const
	{
		btAssert(m_isLocalAabbValid);
		aabbMin = m_localAabbMin;
		aabbMax = m_localAabbMax;
	}

	inline void getNonvirtualAabb(const btTransform& trans, btVector3& aabbMin, btVector3& aabbMax, btScalar margin) const
	{
		//lazy evaluation of local aabb
		btAssert(m_isLocalAabbValid);
		btTransformAabb(m_localAabbMin, m_localAabbMax, margin, trans, aabbMin, aabbMax);
	}

public:
	virtual void setLocalScaling(const btVector3& scaling)
	{
		btConvexInternalShape::setLocalScaling(scaling);
		recalcLocalAabb();
	}

	virtual void getAabb(const btTransform& trans, btVector3& aabbMin, btVector3& aabbMax) const
	{
		getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
	}

	void recalcLocalAabb()
	{
		m_isLocalAabbValid = true;

		static const btVector3 _directions[] =
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

		batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

		for (int i = 0; i < 3; ++i)
		{
			m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
			m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
		}
	}
};

#endif  //BT_CONVEX_INTERNAL_SHAPE_H
