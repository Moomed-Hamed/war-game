///This file was written by Erwin Coumans

#ifndef _BT_POLYHEDRAL_FEATURES_H
#define _BT_POLYHEDRAL_FEATURES_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"

#define TEST_INTERNAL_OBJECTS 1

inline bool IsAlmostZero1(const btVector3& v)
{
	if (btFabs(v.x()) > 1e-6 || btFabs(v.y()) > 1e-6 || btFabs(v.z()) > 1e-6) return false;
	return true;
}

struct btInternalVertexPair
{
	btInternalVertexPair(short int v0, short int v1)
		: m_v0(v0),
		  m_v1(v1)
	{
		if (m_v1 > m_v0)
			btSwap(m_v0, m_v1);
	}
	short int m_v0;
	short int m_v1;
	int getHash() const
	{
		return m_v0 + (m_v1 << 16);
	}
	bool equals(const btInternalVertexPair& other) const
	{
		return m_v0 == other.m_v0 && m_v1 == other.m_v1;
	}
};

struct btInternalEdge
{
	btInternalEdge()
		: m_face0(-1),
		  m_face1(-1)
	{
	}
	short int m_face0;
	short int m_face1;
};

struct btFace
{
	btAlignedObjectArray<int> m_indices;
	//	btAlignedObjectArray<int>	m_connectedFaces;
	btScalar m_plane[4];
};

ATTRIBUTE_ALIGNED16(class)
btConvexPolyhedron
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btConvexPolyhedron(){};
	virtual ~btConvexPolyhedron(){};

	btAlignedObjectArray<btVector3> m_vertices;
	btAlignedObjectArray<btFace> m_faces;
	btAlignedObjectArray<btVector3> m_uniqueEdges;

	btVector3 m_localCenter;
	btVector3 m_extents;
	btScalar m_radius;
	btVector3 mC;
	btVector3 mE;

	void initialize()
	{
		btHashMap<btInternalVertexPair, btInternalEdge> edges;

		for (int i = 0; i < m_faces.size(); i++)
		{
			int numVertices = m_faces[i].m_indices.size();
			int NbTris = numVertices;
			for (int j = 0; j < NbTris; j++)
			{
				int k = (j + 1) % numVertices;
				btInternalVertexPair vp(m_faces[i].m_indices[j], m_faces[i].m_indices[k]);
				btInternalEdge* edptr = edges.find(vp);
				btVector3 edge = m_vertices[vp.m_v1] - m_vertices[vp.m_v0];
				edge.normalize();

				bool found = false;

				for (int p = 0; p < m_uniqueEdges.size(); p++)
				{
					if (IsAlmostZero1(m_uniqueEdges[p] - edge) ||
						IsAlmostZero1(m_uniqueEdges[p] + edge))
					{
						found = true;
						break;
					}
				}

				if (!found)
				{
					m_uniqueEdges.push_back(edge);
				}

				if (edptr)
				{
					btAssert(edptr->m_face0 >= 0);
					btAssert(edptr->m_face1 < 0);
					edptr->m_face1 = i;
				}
				else
				{
					btInternalEdge ed;
					ed.m_face0 = i;
					edges.insert(vp, ed);
				}
			}
		}

		initialize2();
	}
	void initialize2()
	{
		m_localCenter.setValue(0, 0, 0);
		btScalar TotalArea = 0.0f;
		for (int i = 0; i < m_faces.size(); i++)
		{
			int numVertices = m_faces[i].m_indices.size();
			int NbTris = numVertices - 2;

			const btVector3& p0 = m_vertices[m_faces[i].m_indices[0]];
			for (int j = 1; j <= NbTris; j++)
			{
				int k = (j + 1) % numVertices;
				const btVector3& p1 = m_vertices[m_faces[i].m_indices[j]];
				const btVector3& p2 = m_vertices[m_faces[i].m_indices[k]];
				btScalar Area = ((p0 - p1).cross(p0 - p2)).length() * 0.5f;
				btVector3 Center = (p0 + p1 + p2) / 3.0f;
				m_localCenter += Area * Center;
				TotalArea += Area;
			}
		}
		m_localCenter /= TotalArea;

		if (1)
		{
			m_radius = FLT_MAX;
			for (int i = 0; i < m_faces.size(); i++)
			{
				const btVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
				const btScalar dist = btFabs(m_localCenter.dot(Normal) + m_faces[i].m_plane[3]);
				if (dist < m_radius)
					m_radius = dist;
			}

			btScalar MinX = FLT_MAX;
			btScalar MinY = FLT_MAX;
			btScalar MinZ = FLT_MAX;
			btScalar MaxX = -FLT_MAX;
			btScalar MaxY = -FLT_MAX;
			btScalar MaxZ = -FLT_MAX;
			for (int i = 0; i < m_vertices.size(); i++)
			{
				const btVector3& pt = m_vertices[i];
				if (pt.x() < MinX) MinX = pt.x();
				if (pt.x() > MaxX) MaxX = pt.x();
				if (pt.y() < MinY) MinY = pt.y();
				if (pt.y() > MaxY) MaxY = pt.y();
				if (pt.z() < MinZ) MinZ = pt.z();
				if (pt.z() > MaxZ) MaxZ = pt.z();
			}
			mC.setValue(MaxX + MinX, MaxY + MinY, MaxZ + MinZ);
			mE.setValue(MaxX - MinX, MaxY - MinY, MaxZ - MinZ);

			//		const btScalar r = m_radius / sqrtf(2.0f);
			const btScalar r = m_radius / sqrtf(3.0f);
			const int LargestExtent = mE.maxAxis();
			const btScalar Step = (mE[LargestExtent] * 0.5f - r) / 1024.0f;
			m_extents[0] = m_extents[1] = m_extents[2] = r;
			m_extents[LargestExtent] = mE[LargestExtent] * 0.5f;
			bool FoundBox = false;
			for (int j = 0; j < 1024; j++)
			{
				if (testContainment())
				{
					FoundBox = true;
					break;
				}

				m_extents[LargestExtent] -= Step;
			}
			if (!FoundBox)
			{
				m_extents[0] = m_extents[1] = m_extents[2] = r;
			}
			else
			{
				// Refine the box
				const btScalar Step = (m_radius - r) / 1024.0f;
				const int e0 = (1 << LargestExtent) & 3;
				const int e1 = (1 << e0) & 3;

				for (int j = 0; j < 1024; j++)
				{
					const btScalar Saved0 = m_extents[e0];
					const btScalar Saved1 = m_extents[e1];
					m_extents[e0] += Step;
					m_extents[e1] += Step;

					if (!testContainment())
					{
						m_extents[e0] = Saved0;
						m_extents[e1] = Saved1;
						break;
					}
				}
			}
		}
	}
	bool testContainment() const
	{
		for (int p = 0; p < 8; p++)
		{
			btVector3 LocalPt;
			if (p == 0)
				LocalPt = m_localCenter + btVector3(m_extents[0], m_extents[1], m_extents[2]);
			else if (p == 1)
				LocalPt = m_localCenter + btVector3(m_extents[0], m_extents[1], -m_extents[2]);
			else if (p == 2)
				LocalPt = m_localCenter + btVector3(m_extents[0], -m_extents[1], m_extents[2]);
			else if (p == 3)
				LocalPt = m_localCenter + btVector3(m_extents[0], -m_extents[1], -m_extents[2]);
			else if (p == 4)
				LocalPt = m_localCenter + btVector3(-m_extents[0], m_extents[1], m_extents[2]);
			else if (p == 5)
				LocalPt = m_localCenter + btVector3(-m_extents[0], m_extents[1], -m_extents[2]);
			else if (p == 6)
				LocalPt = m_localCenter + btVector3(-m_extents[0], -m_extents[1], m_extents[2]);
			else if (p == 7)
				LocalPt = m_localCenter + btVector3(-m_extents[0], -m_extents[1], -m_extents[2]);

			for (int i = 0; i < m_faces.size(); i++)
			{
				const btVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
				const btScalar d = LocalPt.dot(Normal) + m_faces[i].m_plane[3];
				if (d > 0.0f)
					return false;
			}
		}
		return true;
	}

	void project(const btTransform& trans, const btVector3& dir, btScalar& minProj, btScalar& maxProj, btVector3& witnesPtMin, btVector3& witnesPtMax) const
	{
		minProj = FLT_MAX;
		maxProj = -FLT_MAX;
		int numVerts = m_vertices.size();
		for (int i = 0; i < numVerts; i++)
		{
			btVector3 pt = trans * m_vertices[i];
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
};

#endif  //_BT_POLYHEDRAL_FEATURES_H
