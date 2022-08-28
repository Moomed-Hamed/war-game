#ifndef BT_TRIANGLE_BUFFER_H
#define BT_TRIANGLE_BUFFER_H

#include "btTriangleCallback.h"
#include "LinearMath/btAlignedObjectArray.h"

struct btTriangle
{
	btVector3 m_vertex0;
	btVector3 m_vertex1;
	btVector3 m_vertex2;
	int m_partId;
	int m_triangleIndex;
};

///The btTriangleBuffer callback can be useful to collect and store overlapping triangles between AABB and concave objects that support 'processAllTriangles'
///Example usage of this class:
///			btTriangleBuffer	triBuf;
///			concaveShape->processAllTriangles(&triBuf,aabbMin, aabbMax);
///			for (int i=0;i<triBuf.getNumTriangles();i++)
///			{
///				const btTriangle& tri = triBuf.getTriangle(i);
///				//do something useful here with the triangle
///			}
class btTriangleBuffer : public btTriangleCallback
{
	btAlignedObjectArray<btTriangle> m_triangleBuffer;

public:
	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
	{
		btTriangle tri;
		tri.m_vertex0 = triangle[0];
		tri.m_vertex1 = triangle[1];
		tri.m_vertex2 = triangle[2];
		tri.m_partId = partId;
		tri.m_triangleIndex = triangleIndex;

		m_triangleBuffer.push_back(tri);
	}

	int getNumTriangles() const
	{
		return int(m_triangleBuffer.size());
	}

	const btTriangle& getTriangle(int index) const
	{
		return m_triangleBuffer[index];
	}

	void clearBuffer()
	{
		m_triangleBuffer.clear();
	}
};

#endif  //BT_TRIANGLE_BUFFER_H
