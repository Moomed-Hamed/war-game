#ifndef BT_SDF_COLLISION_SHAPE_H
#define BT_SDF_COLLISION_SHAPE_H

#include "btConcaveShape.h"
#include "btMiniSDF.h"

ATTRIBUTE_ALIGNED16(struct)
btSdfCollisionShapeInternalData
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btVector3 m_localScaling;
	btScalar m_margin;
	btMiniSDF m_sdf;

	btSdfCollisionShapeInternalData()
		: m_localScaling(1, 1, 1),
		  m_margin(0)
	{
	}
};

class btSdfCollisionShape : public btConcaveShape
{
	struct btSdfCollisionShapeInternalData* m_data;

public:
	btSdfCollisionShape()
	{
		m_shapeType = SDF_SHAPE_PROXYTYPE;
		m_data = new btSdfCollisionShapeInternalData();
	}
	virtual ~btSdfCollisionShape()
	{
		delete m_data;
	}

	bool initializeSDF(const char* sdfData, int sizeInBytes)
	{
		bool valid = m_data->m_sdf.load(sdfData, sizeInBytes);
		return valid;
	}

	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btAssert(m_data->m_sdf.isValid());
		btVector3 localAabbMin = m_data->m_sdf.m_domain.m_min;
		btVector3 localAabbMax = m_data->m_sdf.m_domain.m_max;
		btScalar margin(0);
		btTransformAabb(localAabbMin, localAabbMax, margin, t, aabbMin, aabbMax);
	}
	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_data->m_localScaling = scaling;
	}
	virtual const btVector3& getLocalScaling() const
	{
		return m_data->m_localScaling;
	}
	virtual void calculateLocalInertia(btScalar mass, btVector3& inertia) const
	{
		inertia.setValue(0, 0, 0);
	}
	virtual const char* getName() const
	{
		return "btSdfCollisionShape";
	}
	virtual void setMargin(btScalar margin)
	{
		m_data->m_margin = margin;
	}
	virtual btScalar getMargin() const
	{
		return m_data->m_margin;
	}

	// not yet
	virtual void processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const {};

	bool queryPoint(const btVector3& ptInSDF, btScalar& distOut, btVector3& normal)
	{
		int field = 0;
		btVector3 grad;
		double dist;
		bool hasResult = m_data->m_sdf.interpolate(field, dist, ptInSDF, &grad);
		if (hasResult)
		{
			normal.setValue(grad[0], grad[1], grad[2]);
			distOut = dist;
		}
		return hasResult;
	}
};

#endif  //BT_SDF_COLLISION_SHAPE_H
