#ifndef BT_TRIANGLE_MESH_H
#define BT_TRIANGLE_MESH_H

#include "btTriangleIndexVertexArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

///The btTriangleMesh class is a convenience class derived from btTriangleIndexVertexArray, that provides storage for a concave triangle mesh. It can be used as data for the btBvhTriangleMeshShape.
///It allows either 32bit or 16bit indices, and 4 (x-y-z-w) or 3 (x-y-z) component vertices.
///If you want to share triangle/index data between graphics mesh and collision mesh (btBvhTriangleMeshShape), you can directly use btTriangleIndexVertexArray or derive your own class from btStridingMeshInterface.
///Performance of btTriangleMesh and btTriangleIndexVertexArray used in a btBvhTriangleMeshShape is the same.
class btTriangleMesh : public btTriangleIndexVertexArray
{
	btAlignedObjectArray<btVector3> m_4componentVertices;
	btAlignedObjectArray<btScalar> m_3componentVertices;

	btAlignedObjectArray<unsigned int> m_32bitIndices;
	btAlignedObjectArray<unsigned short int> m_16bitIndices;
	bool m_use32bitIndices;
	bool m_use4componentVertices;

public:
	btScalar m_weldingThreshold;

	btTriangleMesh(bool use32bitIndices = true, bool use4componentVertices = true)
		: m_use32bitIndices(use32bitIndices),
		  m_use4componentVertices(use4componentVertices),
		  m_weldingThreshold(0.0)
	{
		btIndexedMesh meshIndex;
		meshIndex.m_numTriangles = 0;
		meshIndex.m_numVertices = 0;
		meshIndex.m_indexType = PHY_INTEGER;
		meshIndex.m_triangleIndexBase = 0;
		meshIndex.m_triangleIndexStride = 3 * sizeof(int);
		meshIndex.m_vertexBase = 0;
		meshIndex.m_vertexStride = sizeof(btVector3);
		m_indexedMeshes.push_back(meshIndex);

		if (m_use32bitIndices)
		{
			m_indexedMeshes[0].m_numTriangles = m_32bitIndices.size() / 3;
			m_indexedMeshes[0].m_triangleIndexBase = 0;
			m_indexedMeshes[0].m_indexType = PHY_INTEGER;
			m_indexedMeshes[0].m_triangleIndexStride = 3 * sizeof(int);
		}
		else
		{
			m_indexedMeshes[0].m_numTriangles = m_16bitIndices.size() / 3;
			m_indexedMeshes[0].m_triangleIndexBase = 0;
			m_indexedMeshes[0].m_indexType = PHY_SHORT;
			m_indexedMeshes[0].m_triangleIndexStride = 3 * sizeof(short int);
		}

		if (m_use4componentVertices)
		{
			m_indexedMeshes[0].m_numVertices = m_4componentVertices.size();
			m_indexedMeshes[0].m_vertexBase = 0;
			m_indexedMeshes[0].m_vertexStride = sizeof(btVector3);
		}
		else
		{
			m_indexedMeshes[0].m_numVertices = m_3componentVertices.size() / 3;
			m_indexedMeshes[0].m_vertexBase = 0;
			m_indexedMeshes[0].m_vertexStride = 3 * sizeof(btScalar);
		}
	}

	bool getUse32bitIndices() const
	{
		return m_use32bitIndices;
	}

	bool getUse4componentVertices() const
	{
		return m_use4componentVertices;
	}
	///By default addTriangle won't search for duplicate vertices, because the search is very slow for large triangle meshes.
	///In general it is better to directly use btTriangleIndexVertexArray instead.
	void addTriangle(const btVector3& vertex0, const btVector3& vertex1, const btVector3& vertex2, bool removeDuplicateVertices = false)
	{
		m_indexedMeshes[0].m_numTriangles++;
		addIndex(findOrAddVertex(vertex0, removeDuplicateVertices));
		addIndex(findOrAddVertex(vertex1, removeDuplicateVertices));
		addIndex(findOrAddVertex(vertex2, removeDuplicateVertices));
	}

	///Add a triangle using its indices. Make sure the indices are pointing within the vertices array, so add the vertices first (and to be sure, avoid removal of duplicate vertices)
	void addTriangleIndices(int index1, int index2, int index3)
	{
		m_indexedMeshes[0].m_numTriangles++;
		addIndex(index1);
		addIndex(index2);
		addIndex(index3);
	}

	int getNumTriangles() const
	{
		if (m_use32bitIndices)
		{
			return m_32bitIndices.size() / 3;
		}
		return m_16bitIndices.size() / 3;
	}

	virtual void preallocateVertices(int numverts)
	{
		if (m_use4componentVertices)
		{
			m_4componentVertices.reserve(numverts);
		}
		else
		{
			m_3componentVertices.reserve(numverts);
		}
	}
	virtual void preallocateIndices(int numindices)
	{
		if (m_use32bitIndices)
		{
			m_32bitIndices.reserve(numindices);
		}
		else
		{
			m_16bitIndices.reserve(numindices);
		}
	}

	///findOrAddVertex is an internal method, use addTriangle instead
	int findOrAddVertex(const btVector3& vertex, bool removeDuplicateVertices)
	{
		//return index of new/existing vertex
		///@todo: could use acceleration structure for this
		if (m_use4componentVertices)
		{
			if (removeDuplicateVertices)
			{
				for (int i = 0; i < m_4componentVertices.size(); i++)
				{
					if ((m_4componentVertices[i] - vertex).length2() <= m_weldingThreshold)
					{
						return i;
					}
				}
			}
			m_indexedMeshes[0].m_numVertices++;
			m_4componentVertices.push_back(vertex);
			m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_4componentVertices[0];

			return m_4componentVertices.size() - 1;
		}
		else
		{
			if (removeDuplicateVertices)
			{
				for (int i = 0; i < m_3componentVertices.size(); i += 3)
				{
					btVector3 vtx(m_3componentVertices[i], m_3componentVertices[i + 1], m_3componentVertices[i + 2]);
					if ((vtx - vertex).length2() <= m_weldingThreshold)
					{
						return i / 3;
					}
				}
			}
			m_3componentVertices.push_back(vertex.getX());
			m_3componentVertices.push_back(vertex.getY());
			m_3componentVertices.push_back(vertex.getZ());
			m_indexedMeshes[0].m_numVertices++;
			m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_3componentVertices[0];
			return (m_3componentVertices.size() / 3) - 1;
		}
	}
	///addIndex is an internal method, use addTriangle instead
	void addIndex(int index)
	{
		if (m_use32bitIndices)
		{
			m_32bitIndices.push_back(index);
			m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*)&m_32bitIndices[0];
		}
		else
		{
			m_16bitIndices.push_back(index);
			m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*)&m_16bitIndices[0];
		}
	}
};

#endif  //BT_TRIANGLE_MESH_H
