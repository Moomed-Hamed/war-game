#ifndef BT_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_BVH_TRIANGLE_MESH_SHAPE_H

#include "btTriangleMeshShape.h"
#include "btOptimizedBvh.h"
#include "btTriangleInfoMap.h"

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btTriangleMeshShapeData
{
	btCollisionShapeData m_collisionShapeData;

	btStridingMeshInterfaceData m_meshInterface;

	btQuantizedBvhFloatData* m_quantizedFloatBvh;
	btQuantizedBvhDoubleData* m_quantizedDoubleBvh;

	btTriangleInfoMapData* m_triangleInfoMap;

	float m_collisionMargin;

	char m_pad3[4];
};

///The btBvhTriangleMeshShape is a static-triangle mesh shape, it can only be used for fixed/non-moving objects.
///If you required moving concave triangle meshes, it is recommended to perform convex decomposition
///using HACD, see Bullet/Demos/ConvexDecompositionDemo.
///Alternatively, you can use btGimpactMeshShape for moving concave triangle meshes.
///btBvhTriangleMeshShape has several optimizations, such as bounding volume hierarchy and
///cache friendly traversal for PlayStation 3 Cell SPU.
///It is recommended to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
ATTRIBUTE_ALIGNED16(class)
btBvhTriangleMeshShape : public btTriangleMeshShape
{
	btOptimizedBvh* m_bvh;
	btTriangleInfoMap* m_triangleInfoMap;

	bool m_useQuantizedAabbCompression;
	bool m_ownsBvh;
	bool m_pad[11];  ////need padding due to alignment

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	///Bvh Concave triangle mesh is a static-triangle mesh shape with Bounding Volume Hierarchy optimization.
	///Uses an interface to access the triangles to allow for sharing graphics/physics triangles.
	btBvhTriangleMeshShape(btStridingMeshInterface * meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true)
		: btTriangleMeshShape(meshInterface),
		  m_bvh(0),
		  m_triangleInfoMap(0),
		  m_useQuantizedAabbCompression(useQuantizedAabbCompression),
		  m_ownsBvh(false)
	{
		m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
		//construct bvh from meshInterface
		if (buildBvh)
		{
			buildOptimizedBvh();
		}
	}

	///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
	btBvhTriangleMeshShape(btStridingMeshInterface * meshInterface, bool useQuantizedAabbCompression, const btVector3& bvhAabbMin, const btVector3& bvhAabbMax, bool buildBvh = true)
		: btTriangleMeshShape(meshInterface),
		  m_bvh(0),
		  m_triangleInfoMap(0),
		  m_useQuantizedAabbCompression(useQuantizedAabbCompression),
		  m_ownsBvh(false)
	{
		m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
		//construct bvh from meshInterface

		if (buildBvh)
		{
			void* mem = btAlignedAlloc(sizeof(btOptimizedBvh), 16);
			m_bvh = new (mem) btOptimizedBvh();

			m_bvh->build(meshInterface, m_useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax);
			m_ownsBvh = true;
		}
	}

	virtual ~btBvhTriangleMeshShape()
	{
		if (m_ownsBvh)
		{
			m_bvh->~btOptimizedBvh();
			btAlignedFree(m_bvh);
		}
	}

	bool getOwnsBvh() const
	{
		return m_ownsBvh;
	}

	void performRaycast(btTriangleCallback * callback, const btVector3& raySource, const btVector3& rayTarget)
	{
		struct MyNodeOverlapCallback : public btNodeOverlapCallback
		{
			btStridingMeshInterface* m_meshInterface;
			btTriangleCallback* m_callback;

			MyNodeOverlapCallback(btTriangleCallback* callback, btStridingMeshInterface* meshInterface)
				: m_meshInterface(meshInterface),
				  m_callback(callback)
			{
			}

			virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
			{
				btVector3 m_triangle[3];
				const unsigned char* vertexbase;
				int numverts;
				PHY_ScalarType type;
				int stride;
				const unsigned char* indexbase;
				int indexstride;
				int numfaces;
				PHY_ScalarType indicestype;

				m_meshInterface->getLockedReadOnlyVertexIndexBase(
					&vertexbase,
					numverts,
					type,
					stride,
					&indexbase,
					indexstride,
					numfaces,
					indicestype,
					nodeSubPart);

				unsigned int* gfxbase = (unsigned int*)(indexbase + nodeTriangleIndex * indexstride);

				const btVector3& meshScaling = m_meshInterface->getScaling();
				for (int j = 2; j >= 0; j--)
				{
					int graphicsindex;
					switch (indicestype)
					{
						case PHY_INTEGER:
							graphicsindex = gfxbase[j];
							break;
						case PHY_SHORT:
							graphicsindex = ((unsigned short*)gfxbase)[j];
							break;
						case PHY_UCHAR:
							graphicsindex = ((unsigned char*)gfxbase)[j];
							break;
						default:
							btAssert(0);
					}

					if (type == PHY_FLOAT)
					{
						float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);

						m_triangle[j] = btVector3(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
					}
					else
					{
						double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);

						m_triangle[j] = btVector3(btScalar(graphicsbase[0]) * meshScaling.getX(), btScalar(graphicsbase[1]) * meshScaling.getY(), btScalar(graphicsbase[2]) * meshScaling.getZ());
					}
				}

				/* Perform ray vs. triangle collision here */
				m_callback->processTriangle(m_triangle, nodeSubPart, nodeTriangleIndex);
				m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
			}
		};

		MyNodeOverlapCallback myNodeCallback(callback, m_meshInterface);

		m_bvh->reportRayOverlappingNodex(&myNodeCallback, raySource, rayTarget);
	}
	void performConvexcast(btTriangleCallback * callback, const btVector3& raySource, const btVector3& rayTarget, const btVector3& aabbMin, const btVector3& aabbMax)
	{
		struct MyNodeOverlapCallback : public btNodeOverlapCallback
		{
			btStridingMeshInterface* m_meshInterface;
			btTriangleCallback* m_callback;

			MyNodeOverlapCallback(btTriangleCallback* callback, btStridingMeshInterface* meshInterface)
				: m_meshInterface(meshInterface),
				  m_callback(callback)
			{
			}

			virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
			{
				btVector3 m_triangle[3];
				const unsigned char* vertexbase;
				int numverts;
				PHY_ScalarType type;
				int stride;
				const unsigned char* indexbase;
				int indexstride;
				int numfaces;
				PHY_ScalarType indicestype;

				m_meshInterface->getLockedReadOnlyVertexIndexBase(
					&vertexbase,
					numverts,
					type,
					stride,
					&indexbase,
					indexstride,
					numfaces,
					indicestype,
					nodeSubPart);

				unsigned int* gfxbase = (unsigned int*)(indexbase + nodeTriangleIndex * indexstride);

				const btVector3& meshScaling = m_meshInterface->getScaling();
				for (int j = 2; j >= 0; j--)
				{
					int graphicsindex;
					switch (indicestype)
					{
						case PHY_INTEGER:
							graphicsindex = gfxbase[j];
							break;
						case PHY_SHORT:
							graphicsindex = ((unsigned short*)gfxbase)[j];
							break;
						case PHY_UCHAR:
							graphicsindex = ((unsigned char*)gfxbase)[j];
							break;
						default:
							btAssert(0);
					}

					if (type == PHY_FLOAT)
					{
						float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);

						m_triangle[j] = btVector3(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
					}
					else
					{
						double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);

						m_triangle[j] = btVector3(btScalar(graphicsbase[0]) * meshScaling.getX(), btScalar(graphicsbase[1]) * meshScaling.getY(), btScalar(graphicsbase[2]) * meshScaling.getZ());
					}
				}

				/* Perform ray vs. triangle collision here */
				m_callback->processTriangle(m_triangle, nodeSubPart, nodeTriangleIndex);
				m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
			}
		};

		MyNodeOverlapCallback myNodeCallback(callback, m_meshInterface);

		m_bvh->reportBoxCastOverlappingNodex(&myNodeCallback, raySource, rayTarget, aabbMin, aabbMax);
	}

	virtual void processAllTriangles(btTriangleCallback * callback, const btVector3& aabbMin, const btVector3& aabbMax) const
	{
		//first get all the nodes

		struct MyNodeOverlapCallback : public btNodeOverlapCallback
		{
			btStridingMeshInterface* m_meshInterface;
			btTriangleCallback* m_callback;
			btVector3 m_triangle[3];
			int m_numOverlap;

			MyNodeOverlapCallback(btTriangleCallback* callback, btStridingMeshInterface* meshInterface)
				: m_meshInterface(meshInterface),
				  m_callback(callback),
				  m_numOverlap(0)
			{
			}

			virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
			{
				m_numOverlap++;
				const unsigned char* vertexbase;
				int numverts;
				PHY_ScalarType type;
				int stride;
				const unsigned char* indexbase;
				int indexstride;
				int numfaces;
				PHY_ScalarType indicestype;

				m_meshInterface->getLockedReadOnlyVertexIndexBase(
					&vertexbase,
					numverts,
					type,
					stride,
					&indexbase,
					indexstride,
					numfaces,
					indicestype,
					nodeSubPart);

				unsigned int* gfxbase = (unsigned int*)(indexbase + nodeTriangleIndex * indexstride);
				btAssert(indicestype == PHY_INTEGER || indicestype == PHY_SHORT || indicestype == PHY_UCHAR);

				const btVector3& meshScaling = m_meshInterface->getScaling();
				for (int j = 2; j >= 0; j--)
				{
					int graphicsindex = indicestype == PHY_SHORT ? ((unsigned short*)gfxbase)[j] : indicestype == PHY_INTEGER ? gfxbase[j]
																															  : ((unsigned char*)gfxbase)[j];

					if (type == PHY_FLOAT)
					{
						float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);

						m_triangle[j] = btVector3(
							graphicsbase[0] * meshScaling.getX(),
							graphicsbase[1] * meshScaling.getY(),
							graphicsbase[2] * meshScaling.getZ());
					}
					else
					{
						double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);

						m_triangle[j] = btVector3(
							btScalar(graphicsbase[0]) * meshScaling.getX(),
							btScalar(graphicsbase[1]) * meshScaling.getY(),
							btScalar(graphicsbase[2]) * meshScaling.getZ());
					}
				}

				m_callback->processTriangle(m_triangle, nodeSubPart, nodeTriangleIndex);
				m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
			}
		};

		MyNodeOverlapCallback myNodeCallback(callback, m_meshInterface);

		m_bvh->reportAabbOverlappingNodex(&myNodeCallback, aabbMin, aabbMax);
	}

	void refitTree(const btVector3& aabbMin, const btVector3& aabbMax)
	{
		m_bvh->refit(m_meshInterface, aabbMin, aabbMax);

		recalcLocalAabb();
	}

	///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
	void partialRefitTree(const btVector3& aabbMin, const btVector3& aabbMax)
	{
		m_bvh->refitPartial(m_meshInterface, aabbMin, aabbMax);

		m_localAabbMin.setMin(aabbMin);
		m_localAabbMax.setMax(aabbMax);
	}

	//debugging
	virtual const char* getName() const { return "BVHTRIANGLEMESH"; }

	virtual void setLocalScaling(const btVector3& scaling)
	{
		if ((getLocalScaling() - scaling).length2() > SIMD_EPSILON)
		{
			btTriangleMeshShape::setLocalScaling(scaling);
			buildOptimizedBvh();
		}
	}

	btOptimizedBvh* getOptimizedBvh()
	{
		return m_bvh;
	}

	void setOptimizedBvh(btOptimizedBvh * bvh, const btVector3& localScaling = btVector3(1, 1, 1))
	{
		btAssert(!m_bvh);
		btAssert(!m_ownsBvh);

		m_bvh = bvh;
		m_ownsBvh = false;
		// update the scaling without rebuilding the bvh
		if ((getLocalScaling() - localScaling).length2() > SIMD_EPSILON)
		{
			btTriangleMeshShape::setLocalScaling(localScaling);
		}
	}

	void buildOptimizedBvh()
	{
		if (m_ownsBvh)
		{
			m_bvh->~btOptimizedBvh();
			btAlignedFree(m_bvh);
		}
		///m_localAabbMin/m_localAabbMax is already re-calculated in btTriangleMeshShape. We could just scale aabb, but this needs some more work
		void* mem = btAlignedAlloc(sizeof(btOptimizedBvh), 16);
		m_bvh = new (mem) btOptimizedBvh();
		//rebuild the bvh...
		m_bvh->build(m_meshInterface, m_useQuantizedAabbCompression, m_localAabbMin, m_localAabbMax);
		m_ownsBvh = true;
	}

	bool usesQuantizedAabbCompression() const
	{
		return m_useQuantizedAabbCompression;
	}

	void setTriangleInfoMap(btTriangleInfoMap * triangleInfoMap)
	{
		m_triangleInfoMap = triangleInfoMap;
	}

	const btTriangleInfoMap* getTriangleInfoMap() const
	{
		return m_triangleInfoMap;
	}

	btTriangleInfoMap* getTriangleInfoMap()
	{
		return m_triangleInfoMap;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const
	{
		btTriangleMeshShapeData* trimeshData = (btTriangleMeshShapeData*)dataBuffer;

		btCollisionShape::serialize(&trimeshData->m_collisionShapeData, serializer);

		m_meshInterface->serialize(&trimeshData->m_meshInterface, serializer);

		trimeshData->m_collisionMargin = float(m_collisionMargin);

		if (m_bvh && !(serializer->getSerializationFlags() & BT_SERIALIZE_NO_BVH))
		{
			void* chunk = serializer->findPointer(m_bvh);
			if (chunk)
			{
				trimeshData->m_quantizedFloatBvh = (btQuantizedBvhData*)chunk;
				trimeshData->m_quantizedDoubleBvh = 0;
			}
			else
			{
				trimeshData->m_quantizedFloatBvh = (btQuantizedBvhData*)serializer->getUniquePointer(m_bvh);
				trimeshData->m_quantizedDoubleBvh = 0;

				int sz = m_bvh->calculateSerializeBufferSizeNew();
				btChunk* chunk = serializer->allocate(sz, 1);
				const char* structType = m_bvh->serialize(chunk->m_oldPtr, serializer);
				serializer->finalizeChunk(chunk, structType, BT_QUANTIZED_BVH_CODE, m_bvh);
			}
		}
		else
		{
			trimeshData->m_quantizedFloatBvh = 0;
			trimeshData->m_quantizedDoubleBvh = 0;
		}

		if (m_triangleInfoMap && !(serializer->getSerializationFlags() & BT_SERIALIZE_NO_TRIANGLEINFOMAP))
		{
			void* chunk = serializer->findPointer(m_triangleInfoMap);
			if (chunk)
			{
				trimeshData->m_triangleInfoMap = (btTriangleInfoMapData*)chunk;
			}
			else
			{
				trimeshData->m_triangleInfoMap = (btTriangleInfoMapData*)serializer->getUniquePointer(m_triangleInfoMap);
				int sz = m_triangleInfoMap->calculateSerializeBufferSize();
				btChunk* chunk = serializer->allocate(sz, 1);
				const char* structType = m_triangleInfoMap->serialize(chunk->m_oldPtr, serializer);
				serializer->finalizeChunk(chunk, structType, BT_TRIANLGE_INFO_MAP, m_triangleInfoMap);
			}
		}
		else
		{
			trimeshData->m_triangleInfoMap = 0;
		}

		// Fill padding with zeros to appease msan.
		memset(trimeshData->m_pad3, 0, sizeof(trimeshData->m_pad3));

		return "btTriangleMeshShapeData";
	}

	virtual void serializeSingleBvh(btSerializer * serializer) const
	{
		if (m_bvh)
		{
			int len = m_bvh->calculateSerializeBufferSizeNew();  //make sure not to use calculateSerializeBufferSize because it is used for in-place
			btChunk* chunk = serializer->allocate(len, 1);
			const char* structType = m_bvh->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_QUANTIZED_BVH_CODE, (void*)m_bvh);
		}
	}

	virtual void serializeSingleTriangleInfoMap(btSerializer * serializer) const
	{
		if (m_triangleInfoMap)
		{
			int len = m_triangleInfoMap->calculateSerializeBufferSize();
			btChunk* chunk = serializer->allocate(len, 1);
			const char* structType = m_triangleInfoMap->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_TRIANLGE_INFO_MAP, (void*)m_triangleInfoMap);
		}
	}
};

SIMD_FORCE_INLINE int btBvhTriangleMeshShape::calculateSerializeBufferSize() const
{
	return sizeof(btTriangleMeshShapeData);
}

#endif  //BT_BVH_TRIANGLE_MESH_SHAPE_H
