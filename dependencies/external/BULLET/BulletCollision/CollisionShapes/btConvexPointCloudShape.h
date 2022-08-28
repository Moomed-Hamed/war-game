#ifndef BT_CONVEX_POINT_CLOUD_SHAPE_H
#define BT_CONVEX_POINT_CLOUD_SHAPE_H

#include "btPolyhedralConvexShape.h"

///The btConvexPointCloudShape implements an implicit convex hull of an array of vertices.
ATTRIBUTE_ALIGNED16(class)
btConvexPointCloudShape : public btPolyhedralConvexAabbCachingShape
{
	btVector3* m_unscaledPoints;
	int m_numPoints;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btConvexPointCloudShape()
	{
		m_localScaling.setValue(1.f, 1.f, 1.f);
		m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = 0;
		m_numPoints = 0;
	}

	btConvexPointCloudShape(btVector3 * points, int numPoints, const btVector3& localScaling, bool computeAabb = true)
	{
		m_localScaling = localScaling;
		m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = points;
		m_numPoints = numPoints;

		if (computeAabb)
			recalcLocalAabb();
	}

	void setPoints(btVector3 * points, int numPoints, bool computeAabb = true, const btVector3& localScaling = btVector3(1.f, 1.f, 1.f))
	{
		m_unscaledPoints = points;
		m_numPoints = numPoints;
		m_localScaling = localScaling;

		if (computeAabb)
			recalcLocalAabb();
	}

	SIMD_FORCE_INLINE btVector3* getUnscaledPoints()
	{
		return m_unscaledPoints;
	}

	SIMD_FORCE_INLINE const btVector3* getUnscaledPoints() const
	{
		return m_unscaledPoints;
	}

	SIMD_FORCE_INLINE int getNumPoints() const
	{
		return m_numPoints;
	}

	SIMD_FORCE_INLINE btVector3 getScaledPoint(int index) const
	{
		return m_unscaledPoints[index] * m_localScaling;
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
	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec0) const
	{
		btVector3 supVec(btScalar(0.), btScalar(0.), btScalar(0.));
		btScalar maxDot = btScalar(-BT_LARGE_FLOAT);

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

		if (m_numPoints > 0)
		{
			// Here we take advantage of dot(a*b, c) = dot( a, b*c) to do less work. Note this transformation is true mathematically, not numerically.
			//    btVector3 scaled = vec * m_localScaling;
			int index = (int)vec.maxDot(&m_unscaledPoints[0], m_numPoints, maxDot);  //FIXME: may violate encapsulation of m_unscaledPoints
			return getScaledPoint(index);
		}

		return supVec;
	}
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		for (int j = 0; j < numVectors; j++)
		{
			const btVector3& vec = vectors[j] * m_localScaling;  // dot( a*c, b) = dot(a, b*c)
			btScalar maxDot;
			int index = (int)vec.maxDot(&m_unscaledPoints[0], m_numPoints, maxDot);
			supportVerticesOut[j][3] = btScalar(-BT_LARGE_FLOAT);
			if (0 <= index)
			{
				//WARNING: don't swap next lines, the w component would get overwritten!
				supportVerticesOut[j] = getScaledPoint(index);
				supportVerticesOut[j][3] = maxDot;
			}
		}
	}

	//debugging
	virtual const char* getName() const { return "ConvexPointCloud"; }

	virtual int getNumVertices() const
	{
		return m_numPoints;
	}

	//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
	//Please note that you can debug-draw btConvexHullShape with the Raytracer Demo
	virtual int getNumEdges() const
	{
		return 0;
	}
	virtual void getEdge(int i, btVector3& pa, btVector3& pb) const
	{
		btAssert(0);
	}
	virtual void getVertex(int i, btVector3& vtx) const
	{
		vtx = m_unscaledPoints[i] * m_localScaling;
	}
	virtual int getNumPlanes() const
	{
		return 0;
	}
	virtual void getPlane(btVector3 & planeNormal, btVector3 & planeSupport, int i) const
	{
		btAssert(0);
	}
	virtual bool isInside(const btVector3&, btScalar) const // not yet
	{
		btAssert(0);
		return false;
	}

	///in case we receive negative scaling
	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
		recalcLocalAabb();
	}
};

#endif  //BT_CONVEX_POINT_CLOUD_SHAPE_H
