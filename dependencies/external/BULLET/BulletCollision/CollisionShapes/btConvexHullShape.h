#ifndef BT_CONVEX_HULL_SHAPE_H
#define BT_CONVEX_HULL_SHAPE_H

#include "btPolyhedralConvexShape.h"

#define BT_USE_SSE_IN_API

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btConvexHullShapeData
{
	btConvexInternalShapeData m_convexInternalShapeData;

	btVector3FloatData* m_unscaledPointsFloatPtr;
	btVector3DoubleData* m_unscaledPointsDoublePtr;

	int m_numUnscaledPoints;
	char m_padding3[4];
};

///The btConvexHullShape implements an implicit convex hull of an array of vertices.
///Bullet provides a general and fast collision detector for convex shapes based on GJK and EPA using localGetSupportingVertex.
ATTRIBUTE_ALIGNED16(class)
btConvexHullShape : public btPolyhedralConvexAabbCachingShape
{
	btAlignedObjectArray<btVector3> m_unscaledPoints;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	///this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive btScalar (x,y,z), the striding defines the number of bytes between each point, in memory.
	///It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
	///btConvexHullShape make an internal copy of the points.
	btConvexHullShape(const btScalar* points = 0, int numPoints = 0, int stride = sizeof(btVector3))
		: btPolyhedralConvexAabbCachingShape()
	{
		m_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
		m_unscaledPoints.resize(numPoints);

		unsigned char* pointsAddress = (unsigned char*)points;

		for (int i = 0; i < numPoints; i++)
		{
			btScalar* point = (btScalar*)pointsAddress;
			m_unscaledPoints[i] = btVector3(point[0], point[1], point[2]);
			pointsAddress += stride;
		}

		recalcLocalAabb();
	}

	void addPoint(const btVector3& point, bool recalculateLocalAabb = true)
	{
		m_unscaledPoints.push_back(point);
		if (recalculateLocalAabb)
			recalcLocalAabb();
	}

	btVector3* getUnscaledPoints()
	{
		return &m_unscaledPoints[0];
	}

	const btVector3* getUnscaledPoints() const
	{
		return &m_unscaledPoints[0];
	}

	///getPoints is obsolete, please use getUnscaledPoints
	const btVector3* getPoints() const
	{
		return getUnscaledPoints();
	}

	void optimizeConvexHull()
	{
		btConvexHullComputer conv;
		conv.compute(&m_unscaledPoints[0].getX(), sizeof(btVector3), m_unscaledPoints.size(), 0.f, 0.f);
		int numVerts = conv.vertices.size();
		m_unscaledPoints.resize(0);
		for (int i = 0; i < numVerts; i++)
		{
			m_unscaledPoints.push_back(conv.vertices[i]);
		}
	}

	SIMD_FORCE_INLINE btVector3 getScaledPoint(int i) const
	{
		return m_unscaledPoints[i] * m_localScaling;
	}

	SIMD_FORCE_INLINE int getNumPoints() const
	{
		return m_unscaledPoints.size();
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
	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		btVector3 supVec(btScalar(0.), btScalar(0.), btScalar(0.));
		btScalar maxDot = btScalar(-BT_LARGE_FLOAT);

		// Here we take advantage of dot(a, b*c) = dot(a*b, c).  Note: This is true mathematically, but not numerically.
		if (0 < m_unscaledPoints.size())
		{
			btVector3 scaled = vec * m_localScaling;
			int index = (int)scaled.maxDot(&m_unscaledPoints[0], m_unscaledPoints.size(), maxDot);  // FIXME: may violate encapsulation of m_unscaledPoints
			return m_unscaledPoints[index] * m_localScaling;
		}

		return supVec;
	}
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		btScalar newDot;
		//use 'w' component of supportVerticesOut?
		{
			for (int i = 0; i < numVectors; i++)
			{
				supportVerticesOut[i][3] = btScalar(-BT_LARGE_FLOAT);
			}
		}

		for (int j = 0; j < numVectors; j++)
		{
			btVector3 vec = vectors[j] * m_localScaling;  // dot(a*b,c) = dot(a,b*c)
			if (0 < m_unscaledPoints.size())
			{
				int i = (int)vec.maxDot(&m_unscaledPoints[0], m_unscaledPoints.size(), newDot);
				supportVerticesOut[j] = getScaledPoint(i);
				supportVerticesOut[j][3] = newDot;
			}
			else
				supportVerticesOut[j][3] = -BT_LARGE_FLOAT;
		}
	}

	virtual void project(const btTransform& trans, const btVector3& dir, btScalar& minProj, btScalar& maxProj, btVector3& witnesPtMin, btVector3& witnesPtMax) const
	{
		minProj = FLT_MAX;
		maxProj = -FLT_MAX;

		int numVerts = m_unscaledPoints.size();
		for (int i = 0; i < numVerts; i++)
		{
			btVector3 vtx = m_unscaledPoints[i] * m_localScaling;
			btVector3 pt = trans * vtx;
			btScalar dp = pt.dot(dir);
			if (dp < minProj)
			{
				minProj = dp;
				witnesPtMin = pt;
			}
			if (dp > maxProj)
			{
				maxProj = dp;
				witnesPtMax = pt;
			}
		}

		if (minProj > maxProj)
		{
			btSwap(minProj, maxProj);
			btSwap(witnesPtMin, witnesPtMax);
		}
	}

	//debugging
	virtual const char* getName() const { return "Convex"; }

	//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
	//Please note that you can debug-draw btConvexHullShape with the Raytracer Demo
	virtual int getNumVertices() const
	{
		return m_unscaledPoints.size();
	}
	virtual int getNumEdges() const
	{
		return m_unscaledPoints.size();
	}
	virtual void getEdge(int i, btVector3& pa, btVector3& pb) const
	{
		int index0 = i % m_unscaledPoints.size();
		int index1 = (i + 1) % m_unscaledPoints.size();
		pa = getScaledPoint(index0);
		pb = getScaledPoint(index1);
	}
	virtual void getVertex(int i, btVector3& vtx) const
	{
		vtx = getScaledPoint(i);
	}
	virtual int getNumPlanes() const { return 0; }
	virtual void getPlane(btVector3 & planeNormal, btVector3 & planeSupport, int i) const
	{
		btAssert(0);
	}
	
	//not yet
	virtual bool isInside(const btVector3&, btScalar) const
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

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const
	{
		//int szc = sizeof(btConvexHullShapeData);
		btConvexHullShapeData* shapeData = (btConvexHullShapeData*)dataBuffer;
		btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

		int numElem = m_unscaledPoints.size();
		shapeData->m_numUnscaledPoints = numElem;
		shapeData->m_unscaledPointsFloatPtr = numElem ? (btVector3Data*)serializer->getUniquePointer((void*)&m_unscaledPoints[0]) : 0;
		shapeData->m_unscaledPointsDoublePtr = 0;

		if (numElem)
		{
			int sz = sizeof(btVector3Data);
			//	int sz2 = sizeof(btVector3DoubleData);
			//	int sz3 = sizeof(btVector3FloatData);
			btChunk* chunk = serializer->allocate(sz, numElem);
			btVector3Data* memPtr = (btVector3Data*)chunk->m_oldPtr;
			for (int i = 0; i < numElem; i++, memPtr++)
			{
				m_unscaledPoints[i].serialize(*memPtr);
			}
			serializer->finalizeChunk(chunk, btVector3DataName, BT_ARRAY_CODE, (void*)&m_unscaledPoints[0]);
		}

		// Fill padding with zeros to appease msan.
		memset(shapeData->m_padding3, 0, sizeof(shapeData->m_padding3));

		return "btConvexHullShapeData";
	}
};

SIMD_FORCE_INLINE int btConvexHullShape::calculateSerializeBufferSize() const
{
	return sizeof(btConvexHullShapeData);
}

#endif  //BT_CONVEX_HULL_SHAPE_H
