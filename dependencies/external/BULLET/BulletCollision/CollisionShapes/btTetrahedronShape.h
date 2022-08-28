#ifndef BT_SIMPLEX_1TO4_SHAPE
#define BT_SIMPLEX_1TO4_SHAPE

#include "btPolyhedralConvexShape.h"

///The btBU_Simplex1to4 implements tetrahedron, triangle, line, vertex collision shapes. In most cases it is better to use btConvexHullShape instead.
ATTRIBUTE_ALIGNED16(class)
btBU_Simplex1to4 : public btPolyhedralConvexAabbCachingShape
{
protected:
	int m_numVertices;
	btVector3 m_vertices[4];

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btBU_Simplex1to4() : btPolyhedralConvexAabbCachingShape(), m_numVertices(0)
	{
		m_shapeType = TETRAHEDRAL_SHAPE_PROXYTYPE;
	}

	btBU_Simplex1to4(const btVector3& pt0) : btPolyhedralConvexAabbCachingShape(), m_numVertices(0)
	{
		m_shapeType = TETRAHEDRAL_SHAPE_PROXYTYPE;
		addVertex(pt0);
	}
	btBU_Simplex1to4(const btVector3& pt0, const btVector3& pt1) : btPolyhedralConvexAabbCachingShape(), m_numVertices(0)
	{
		m_shapeType = TETRAHEDRAL_SHAPE_PROXYTYPE;
		addVertex(pt0);
		addVertex(pt1);
	}
	btBU_Simplex1to4(const btVector3& pt0, const btVector3& pt1, const btVector3& pt2) : btPolyhedralConvexAabbCachingShape(), m_numVertices(0)
	{
		m_shapeType = TETRAHEDRAL_SHAPE_PROXYTYPE;
		addVertex(pt0);
		addVertex(pt1);
		addVertex(pt2);
	}
	btBU_Simplex1to4(const btVector3& pt0, const btVector3& pt1, const btVector3& pt2, const btVector3& pt3) : btPolyhedralConvexAabbCachingShape(), m_numVertices(0)
	{
		m_shapeType = TETRAHEDRAL_SHAPE_PROXYTYPE;
		addVertex(pt0);
		addVertex(pt1);
		addVertex(pt2);
		addVertex(pt3);
	}

	void reset()
	{
		m_numVertices = 0;
	}

	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btPolyhedralConvexAabbCachingShape::getAabb(t, aabbMin, aabbMax);
	}

	void addVertex(const btVector3& pt)
	{
		m_vertices[m_numVertices++] = pt;
		recalcLocalAabb();
	}

	//PolyhedralConvexShape interface

	virtual int getNumVertices() const
	{
		return m_numVertices;
	}

	virtual int getNumEdges() const
	{
		//euler formula, F-E+V = 2, so E = F+V-2

		switch (m_numVertices)
		{
			case 0:
				return 0;
			case 1:
				return 0;
			case 2:
				return 1;
			case 3:
				return 3;
			case 4:
				return 6;
		}

		return 0;
	}

	virtual void getEdge(int i, btVector3& pa, btVector3& pb) const
	{
		switch (m_numVertices)
		{
			case 2:
				pa = m_vertices[0];
				pb = m_vertices[1];
				break;
			case 3:
				switch (i)
				{
					case 0:
						pa = m_vertices[0];
						pb = m_vertices[1];
						break;
					case 1:
						pa = m_vertices[1];
						pb = m_vertices[2];
						break;
					case 2:
						pa = m_vertices[2];
						pb = m_vertices[0];
						break;
				}
				break;
			case 4:
				switch (i)
				{
					case 0:
						pa = m_vertices[0];
						pb = m_vertices[1];
						break;
					case 1:
						pa = m_vertices[1];
						pb = m_vertices[2];
						break;
					case 2:
						pa = m_vertices[2];
						pb = m_vertices[0];
						break;
					case 3:
						pa = m_vertices[0];
						pb = m_vertices[3];
						break;
					case 4:
						pa = m_vertices[1];
						pb = m_vertices[3];
						break;
					case 5:
						pa = m_vertices[2];
						pb = m_vertices[3];
						break;
				}
		}
	}

	virtual void getVertex(int i, btVector3& vtx) const
	{
		vtx = m_vertices[i];
	}

	virtual int getNumPlanes() const
	{
		switch (m_numVertices)
		{
			case 0:
				return 0;
			case 1:
				return 0;
			case 2:
				return 0;
			case 3:
				return 2;
			case 4:
				return 4;
			default:
			{
			}
		}
		return 0;
	}

	virtual void getPlane(btVector3&, btVector3&, int) const
	{
	}


	virtual int getIndex(int) const
	{
		return 0;
	}

	virtual bool isInside(const btVector3& pt, btScalar tolerance) const
	{
		return false;
	}

	///getName is for debugging
	virtual const char* getName() const { return "btBU_Simplex1to4"; }
};

#endif  //BT_SIMPLEX_1TO4_SHAPE
