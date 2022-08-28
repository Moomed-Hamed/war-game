#ifndef BT_CONVEX_TRIANGLEMESH_SHAPE_H
#define BT_CONVEX_TRIANGLEMESH_SHAPE_H

#include "btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"

/// It's not nice to have all this virtual function overhead, so perhaps we can also gather the points once
/// but then we are duplicating
class LocalSupportVertexCallback : public btInternalTriangleIndexCallback
{
	btVector3 m_supportVertexLocal;

public:
	btScalar m_maxDot;
	btVector3 m_supportVecLocal;

	LocalSupportVertexCallback(const btVector3& supportVecLocal)
		: m_supportVertexLocal(btScalar(0.), btScalar(0.), btScalar(0.)),
		  m_maxDot(btScalar(-BT_LARGE_FLOAT)),
		  m_supportVecLocal(supportVecLocal)
	{
	}

	virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
	{
		(void)triangleIndex;
		(void)partId;

		for (int i = 0; i < 3; i++)
		{
			btScalar dot = m_supportVecLocal.dot(triangle[i]);
			if (dot > m_maxDot)
			{
				m_maxDot = dot;
				m_supportVertexLocal = triangle[i];
			}
		}
	}

	btVector3 GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}
};

/// The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as btConvexHullShape.
/// A small benefit of this class is that it uses the btStridingMeshInterface, so you can avoid the duplication of the triangle mesh data. Nevertheless, most users should use the much better performing btConvexHullShape instead.
ATTRIBUTE_ALIGNED16(class)
btConvexTriangleMeshShape : public btPolyhedralConvexAabbCachingShape
{
	class btStridingMeshInterface* m_stridingMesh;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btConvexTriangleMeshShape(btStridingMeshInterface * meshInterface, bool calcAabb = true)
		: btPolyhedralConvexAabbCachingShape(), m_stridingMesh(meshInterface)
	{
		m_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
		if (calcAabb)
			recalcLocalAabb();
	}

	class btStridingMeshInterface* getMeshInterface()
	{
		return m_stridingMesh;
	}
	const class btStridingMeshInterface* getMeshInterface() const
	{
		return m_stridingMesh;
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

		LocalSupportVertexCallback supportCallback(vec);
		btVector3 aabbMax(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
		m_stridingMesh->InternalProcessAllTriangles(&supportCallback, -aabbMax, aabbMax);
		supVec = supportCallback.GetSupportVertexLocal();

		return supVec;
	}
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		//use 'w' component of supportVerticesOut?
		{
			for (int i = 0; i < numVectors; i++)
			{
				supportVerticesOut[i][3] = btScalar(-BT_LARGE_FLOAT);
			}
		}

		///@todo: could do the batch inside the callback!

		for (int j = 0; j < numVectors; j++)
		{
			const btVector3& vec = vectors[j];
			LocalSupportVertexCallback supportCallback(vec);
			btVector3 aabbMax(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
			m_stridingMesh->InternalProcessAllTriangles(&supportCallback, -aabbMax, aabbMax);
			supportVerticesOut[j] = supportCallback.GetSupportVertexLocal();
		}
	}

	//debugging
	virtual const char* getName() const { return "ConvexTrimesh"; }

	//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
	//Please note that you can debug-draw btConvexTriangleMeshShape with the Raytracer Demo
	virtual int getNumVertices() const
	{
		//cache this?
		return 0;
	}
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
		btAssert(0);
	}
	virtual int getNumPlanes() const
	{
		return 0;
	}
	virtual void getPlane(btVector3 & planeNormal, btVector3 & planeSupport, int i) const
	{
		btAssert(0);
	}
	virtual bool isInside(const btVector3& pt, btScalar tolerance) const
	{
		btAssert(0);
		return false;
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_stridingMesh->setScaling(scaling);

		recalcLocalAabb();
	}
	virtual const btVector3& getLocalScaling() const
	{
		return m_stridingMesh->getScaling();
	}

	///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
	///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
	///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
	void calculatePrincipalAxisTransform(btTransform & principal, btVector3 & inertia, btScalar & volume) const
	{
		class CenterCallback : public btInternalTriangleIndexCallback
		{
			bool first;
			btVector3 ref;
			btVector3 sum;
			btScalar volume;

		public:
			CenterCallback() : first(true), ref(0, 0, 0), sum(0, 0, 0), volume(0)
			{
			}

			virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
			{
				(void)triangleIndex;
				(void)partId;
				if (first)
				{
					ref = triangle[0];
					first = false;
				}
				else
				{
					btScalar vol = btFabs((triangle[0] - ref).triple(triangle[1] - ref, triangle[2] - ref));
					sum += (btScalar(0.25) * vol) * ((triangle[0] + triangle[1] + triangle[2] + ref));
					volume += vol;
				}
			}

			btVector3 getCenter()
			{
				return (volume > 0) ? sum / volume : ref;
			}

			btScalar getVolume()
			{
				return volume * btScalar(1. / 6);
			}
		};

		class InertiaCallback : public btInternalTriangleIndexCallback
		{
			btMatrix3x3 sum;
			btVector3 center;

		public:
			InertiaCallback(btVector3& center) : sum(0, 0, 0, 0, 0, 0, 0, 0, 0), center(center)
			{
			}

			virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
			{
				(void)triangleIndex;
				(void)partId;
				btMatrix3x3 i;
				btVector3 a = triangle[0] - center;
				btVector3 b = triangle[1] - center;
				btVector3 c = triangle[2] - center;
				btScalar volNeg = -btFabs(a.triple(b, c)) * btScalar(1. / 6);
				for (int j = 0; j < 3; j++)
				{
					for (int k = 0; k <= j; k++)
					{
						i[j][k] = i[k][j] = volNeg * (btScalar(0.1) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k]) + btScalar(0.05) * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
					}
				}
				btScalar i00 = -i[0][0];
				btScalar i11 = -i[1][1];
				btScalar i22 = -i[2][2];
				i[0][0] = i11 + i22;
				i[1][1] = i22 + i00;
				i[2][2] = i00 + i11;
				sum[0] += i[0];
				sum[1] += i[1];
				sum[2] += i[2];
			}

			btMatrix3x3& getInertia()
			{
				return sum;
			}
		};

		CenterCallback centerCallback;
		btVector3 aabbMax(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
		m_stridingMesh->InternalProcessAllTriangles(&centerCallback, -aabbMax, aabbMax);
		btVector3 center = centerCallback.getCenter();
		principal.setOrigin(center);
		volume = centerCallback.getVolume();

		InertiaCallback inertiaCallback(center);
		m_stridingMesh->InternalProcessAllTriangles(&inertiaCallback, -aabbMax, aabbMax);

		btMatrix3x3& i = inertiaCallback.getInertia();
		i.diagonalize(principal.getBasis(), btScalar(0.00001), 20);
		inertia.setValue(i[0][0], i[1][1], i[2][2]);
		inertia /= volume;
	}
};

#endif  //BT_CONVEX_TRIANGLEMESH_SHAPE_H
