#ifndef BT_POLYHEDRAL_CONVEX_SHAPE_H
#define BT_POLYHEDRAL_CONVEX_SHAPE_H

#define BT_USE_SSE_IN_API

#include "btConvexInternalShape.h"
#include "btConvexPolyhedron.h"
#include "LinearMath/btConvexHullComputer.h"
#include "LinearMath/btGeometryUtil.h"

#ifndef MIN
#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#endif

///The btPolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
ATTRIBUTE_ALIGNED16(class)
btPolyhedralConvexShape : public btConvexInternalShape
{
protected:
	btConvexPolyhedron* m_polyhedron;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btPolyhedralConvexShape() : btConvexInternalShape(), m_polyhedron(0)
	{
	}

	virtual ~btPolyhedralConvexShape()
	{
		if (m_polyhedron)
		{
			m_polyhedron->~btConvexPolyhedron();
			btAlignedFree(m_polyhedron);
		}
	}

	///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
	///experimental/work-in-progress
	virtual bool initializePolyhedralFeatures(int shiftVerticesByMargin = 0)
	{
		if (m_polyhedron)
		{
			m_polyhedron->~btConvexPolyhedron();
			btAlignedFree(m_polyhedron);
		}

		void* mem = btAlignedAlloc(sizeof(btConvexPolyhedron), 16);
		m_polyhedron = new (mem) btConvexPolyhedron;

		btAlignedObjectArray<btVector3> orgVertices;

		for (int i = 0; i < getNumVertices(); i++)
		{
			btVector3& newVertex = orgVertices.expand();
			getVertex(i, newVertex);
		}

		btConvexHullComputer conv;

		if (shiftVerticesByMargin)
		{
			btAlignedObjectArray<btVector3> planeEquations;
			btGeometryUtil::getPlaneEquationsFromVertices(orgVertices, planeEquations);

			btAlignedObjectArray<btVector3> shiftedPlaneEquations;
			for (int p = 0; p < planeEquations.size(); p++)
			{
				btVector3 plane = planeEquations[p];
				//	   btScalar margin = getMargin();
				plane[3] -= getMargin();
				shiftedPlaneEquations.push_back(plane);
			}

			btAlignedObjectArray<btVector3> tmpVertices;

			btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations, tmpVertices);

			conv.compute(&tmpVertices[0].getX(), sizeof(btVector3), tmpVertices.size(), 0.f, 0.f);
		}
		else
		{
			conv.compute(&orgVertices[0].getX(), sizeof(btVector3), orgVertices.size(), 0.f, 0.f);
		}

		int numVertices = conv.vertices.size();
		m_polyhedron->m_vertices.resize(numVertices);
		for (int p = 0; p < numVertices; p++)
		{
			m_polyhedron->m_vertices[p] = conv.vertices[p];
		}

		int v0, v1;
		for (int j = 0; j < conv.faces.size(); j++)
		{
			btVector3 edges[3];
			int numEdges = 0;
			btFace combinedFace;
			const btConvexHullComputer::Edge* edge = &conv.edges[conv.faces[j]];
			v0 = edge->getSourceVertex();
			int prevVertex = v0;
			combinedFace.m_indices.push_back(v0);
			v1 = edge->getTargetVertex();
			while (v1 != v0)
			{
				btVector3 wa = conv.vertices[prevVertex];
				btVector3 wb = conv.vertices[v1];
				btVector3 newEdge = wb - wa;
				newEdge.normalize();
				if (numEdges < 2)
					edges[numEdges++] = newEdge;

				//face->addIndex(v1);
				combinedFace.m_indices.push_back(v1);
				edge = edge->getNextEdgeOfFace();
				prevVertex = v1;
				int v01 = edge->getSourceVertex();
				v1 = edge->getTargetVertex();
			}

			btAssert(combinedFace.m_indices.size() > 2);

			btVector3 faceNormal = edges[0].cross(edges[1]);
			faceNormal.normalize();

			btScalar planeEq = 1e30f;

			for (int v = 0; v < combinedFace.m_indices.size(); v++)
			{
				btScalar eq = m_polyhedron->m_vertices[combinedFace.m_indices[v]].dot(faceNormal);
				if (planeEq > eq)
				{
					planeEq = eq;
				}
			}
			combinedFace.m_plane[0] = faceNormal.getX();
			combinedFace.m_plane[1] = faceNormal.getY();
			combinedFace.m_plane[2] = faceNormal.getZ();
			combinedFace.m_plane[3] = -planeEq;

			m_polyhedron->m_faces.push_back(combinedFace);
		}

		m_polyhedron->initialize();

		return true;
	}

	virtual void setPolyhedralFeatures(btConvexPolyhedron & polyhedron)
	{
		if (m_polyhedron)
		{
			*m_polyhedron = polyhedron;
		}
		else
		{
			void* mem = btAlignedAlloc(sizeof(btConvexPolyhedron), 16);
			m_polyhedron = new (mem) btConvexPolyhedron(polyhedron);
		}
	}

	const btConvexPolyhedron* getConvexPolyhedron() const
	{
		return m_polyhedron;
	}

	//brute force implementations

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec0) const
	{
		btVector3 supVec(0, 0, 0);
		int i;
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

		for (int k = 0; k < getNumVertices(); k += 128)
		{
			btVector3 temp[128];
			int inner_count = MIN(getNumVertices() - k, 128);
			for (i = 0; i < inner_count; i++)
				getVertex(i, temp[i]);
			i = (int)vec.maxDot(temp, inner_count, newDot);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = temp[i];
			}
		}

		return supVec;
	}
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		int i;

		btVector3 vtx;
		btScalar newDot;

		for (i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i][3] = btScalar(-BT_LARGE_FLOAT);
		}

		for (int j = 0; j < numVectors; j++)
		{
			const btVector3& vec = vectors[j];

			for (int k = 0; k < getNumVertices(); k += 128)
			{
				btVector3 temp[128];
				int inner_count = MIN(getNumVertices() - k, 128);
				for (i = 0; i < inner_count; i++)
					getVertex(i, temp[i]);
				i = (int)vec.maxDot(temp, inner_count, newDot);
				if (newDot > supportVerticesOut[j][3])
				{
					supportVerticesOut[j] = temp[i];
					supportVerticesOut[j][3] = newDot;
				}
			}
		}
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		//not yet, return box inertia

		btScalar margin = getMargin();

		btTransform ident;
		ident.setIdentity();
		btVector3 aabbMin, aabbMax;
		getAabb(ident, aabbMin, aabbMax);
		btVector3 halfExtents = (aabbMax - aabbMin) * btScalar(0.5);

		btScalar lx = btScalar(2.) * (halfExtents.x() + margin);
		btScalar ly = btScalar(2.) * (halfExtents.y() + margin);
		btScalar lz = btScalar(2.) * (halfExtents.z() + margin);
		const btScalar x2 = lx * lx;
		const btScalar y2 = ly * ly;
		const btScalar z2 = lz * lz;
		const btScalar scaledmass = mass * btScalar(0.08333333);

		inertia = scaledmass * (btVector3(y2 + z2, x2 + z2, x2 + y2));
	}

	virtual int getNumVertices() const = 0;
	virtual int getNumEdges() const = 0;
	virtual void getEdge(int i, btVector3& pa, btVector3& pb) const = 0;
	virtual void getVertex(int i, btVector3& vtx) const = 0;
	virtual int getNumPlanes() const = 0;
	virtual void getPlane(btVector3 & planeNormal, btVector3 & planeSupport, int i) const = 0;
	//	virtual int getIndex(int i) const = 0 ;

	virtual bool isInside(const btVector3& pt, btScalar tolerance) const = 0;
};

///The btPolyhedralConvexAabbCachingShape adds aabb caching to the btPolyhedralConvexShape
class btPolyhedralConvexAabbCachingShape : public btPolyhedralConvexShape
{
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;
	bool m_isLocalAabbValid;

protected:
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

protected:
	btPolyhedralConvexAabbCachingShape()
		: btPolyhedralConvexShape(),
		  m_localAabbMin(1, 1, 1),
		  m_localAabbMax(-1, -1, -1),
		  m_isLocalAabbValid(false)
	{
	}

public:
	inline void getNonvirtualAabb(const btTransform& trans, btVector3& aabbMin, btVector3& aabbMax, btScalar margin) const
	{
		//lazy evaluation of local aabb
		btAssert(m_isLocalAabbValid);
		btTransformAabb(m_localAabbMin, m_localAabbMax, margin, trans, aabbMin, aabbMax);
	}

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

#endif  //BT_POLYHEDRAL_CONVEX_SHAPE_H
