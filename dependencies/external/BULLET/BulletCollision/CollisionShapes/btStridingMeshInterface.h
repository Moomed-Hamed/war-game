#ifndef BT_STRIDING_MESHINTERFACE_H
#define BT_STRIDING_MESHINTERFACE_H

#include "btConcaveShape.h"

struct btIntIndexData
{
	int m_value;
};

struct btShortIntIndexData
{
	short m_value;
	char m_pad[2];
};

struct btShortIntIndexTripletData
{
	short m_values[3];
	char m_pad[2];
};

struct btCharIndexTripletData
{
	unsigned char m_values[3];
	char m_pad;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btMeshPartData
{
	btVector3FloatData* m_vertices3f;
	btVector3DoubleData* m_vertices3d;

	btIntIndexData* m_indices32;
	btShortIntIndexTripletData* m_3indices16;
	btCharIndexTripletData* m_3indices8;

	btShortIntIndexData* m_indices16;  //backwards compatibility

	int m_numTriangles;  //length of m_indices = m_numTriangles
	int m_numVertices;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btStridingMeshInterfaceData
{
	btMeshPartData* m_meshPartsPtr;
	btVector3FloatData m_scaling;
	int m_numMeshParts;
	char m_padding[4];
};

///	The btStridingMeshInterface is the interface class for high performance generic access to triangle meshes, used in combination with btBvhTriangleMeshShape and some other collision shapes.
/// Using index striding of 3*sizeof(integer) it can use triangle arrays, using index striding of 1*sizeof(integer) it can handle triangle strips.
/// It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that are in gpu memory.
ATTRIBUTE_ALIGNED16(class)
btStridingMeshInterface
{
protected:
	btVector3 m_scaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btStridingMeshInterface() : m_scaling(btScalar(1.), btScalar(1.), btScalar(1.))
	{
	}

	virtual ~btStridingMeshInterface(){};

	virtual void InternalProcessAllTriangles(btInternalTriangleIndexCallback * callback, const btVector3& aabbMin, const btVector3& aabbMax) const
	{
		(void)aabbMin;
		(void)aabbMax;
		int numtotalphysicsverts = 0;
		int part, graphicssubparts = getNumSubParts();
		const unsigned char* vertexbase;
		const unsigned char* indexbase;
		int indexstride;
		PHY_ScalarType type;
		PHY_ScalarType gfxindextype;
		int stride, numverts, numtriangles;
		int gfxindex;
		btVector3 triangle[3];

		btVector3 meshScaling = getScaling();

		///if the number of parts is big, the performance might drop due to the innerloop switch on indextype
		for (part = 0; part < graphicssubparts; part++)
		{
			getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numtriangles, gfxindextype, part);
			numtotalphysicsverts += numtriangles * 3;  //upper bound

			///unlike that developers want to pass in double-precision meshes in single-precision Bullet build
			///so disable this feature by default
			///see patch http://code.google.com/p/bullet/issues/detail?id=213

			switch (type)
			{
				case PHY_FLOAT:
				{
					float* graphicsbase;

					switch (gfxindextype)
					{
						case PHY_INTEGER:
						{
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned int* tri_indices = (unsigned int*)(indexbase + gfxindex * indexstride);
								graphicsbase = (float*)(vertexbase + tri_indices[0] * stride);
								triangle[0].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (float*)(vertexbase + tri_indices[1] * stride);
								triangle[1].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (float*)(vertexbase + tri_indices[2] * stride);
								triangle[2].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								callback->internalProcessTriangleIndex(triangle, part, gfxindex);
							}
							break;
						}
						case PHY_SHORT:
						{
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned short int* tri_indices = (unsigned short int*)(indexbase + gfxindex * indexstride);
								graphicsbase = (float*)(vertexbase + tri_indices[0] * stride);
								triangle[0].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (float*)(vertexbase + tri_indices[1] * stride);
								triangle[1].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (float*)(vertexbase + tri_indices[2] * stride);
								triangle[2].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								callback->internalProcessTriangleIndex(triangle, part, gfxindex);
							}
							break;
						}
						case PHY_UCHAR:
						{
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned char* tri_indices = (unsigned char*)(indexbase + gfxindex * indexstride);
								graphicsbase = (float*)(vertexbase + tri_indices[0] * stride);
								triangle[0].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (float*)(vertexbase + tri_indices[1] * stride);
								triangle[1].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (float*)(vertexbase + tri_indices[2] * stride);
								triangle[2].setValue(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
								callback->internalProcessTriangleIndex(triangle, part, gfxindex);
							}
							break;
						}
						default:
							btAssert((gfxindextype == PHY_INTEGER) || (gfxindextype == PHY_SHORT));
					}
					break;
				}

				case PHY_DOUBLE:
				{
					double* graphicsbase;

					switch (gfxindextype)
					{
						case PHY_INTEGER:
						{
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned int* tri_indices = (unsigned int*)(indexbase + gfxindex * indexstride);
								graphicsbase = (double*)(vertexbase + tri_indices[0] * stride);
								triangle[0].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (double*)(vertexbase + tri_indices[1] * stride);
								triangle[1].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (double*)(vertexbase + tri_indices[2] * stride);
								triangle[2].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								callback->internalProcessTriangleIndex(triangle, part, gfxindex);
							}
							break;
						}
						case PHY_SHORT:
						{
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned short int* tri_indices = (unsigned short int*)(indexbase + gfxindex * indexstride);
								graphicsbase = (double*)(vertexbase + tri_indices[0] * stride);
								triangle[0].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (double*)(vertexbase + tri_indices[1] * stride);
								triangle[1].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (double*)(vertexbase + tri_indices[2] * stride);
								triangle[2].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								callback->internalProcessTriangleIndex(triangle, part, gfxindex);
							}
							break;
						}
						case PHY_UCHAR:
						{
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned char* tri_indices = (unsigned char*)(indexbase + gfxindex * indexstride);
								graphicsbase = (double*)(vertexbase + tri_indices[0] * stride);
								triangle[0].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (double*)(vertexbase + tri_indices[1] * stride);
								triangle[1].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								graphicsbase = (double*)(vertexbase + tri_indices[2] * stride);
								triangle[2].setValue((btScalar)graphicsbase[0] * meshScaling.getX(), (btScalar)graphicsbase[1] * meshScaling.getY(), (btScalar)graphicsbase[2] * meshScaling.getZ());
								callback->internalProcessTriangleIndex(triangle, part, gfxindex);
							}
							break;
						}
						default:
							btAssert((gfxindextype == PHY_INTEGER) || (gfxindextype == PHY_SHORT));
					}
					break;
				}
				default:
					btAssert((type == PHY_FLOAT) || (type == PHY_DOUBLE));
			}

			unLockReadOnlyVertexBase(part);
		}
	}

	///brute force method to calculate aabb
	void calculateAabbBruteForce(btVector3 & aabbMin, btVector3 & aabbMax)
	{
		struct AabbCalculationCallback : public btInternalTriangleIndexCallback
		{
			btVector3 m_aabbMin;
			btVector3 m_aabbMax;

			AabbCalculationCallback()
			{
				m_aabbMin.setValue(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
				m_aabbMax.setValue(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT));
			}

			virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex)
			{
				(void)partId;
				(void)triangleIndex;

				m_aabbMin.setMin(triangle[0]);
				m_aabbMax.setMax(triangle[0]);
				m_aabbMin.setMin(triangle[1]);
				m_aabbMax.setMax(triangle[1]);
				m_aabbMin.setMin(triangle[2]);
				m_aabbMax.setMax(triangle[2]);
			}
		};

		//first calculate the total aabb for all triangles
		AabbCalculationCallback aabbCallback;
		aabbMin.setValue(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT));
		aabbMax.setValue(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
		InternalProcessAllTriangles(&aabbCallback, aabbMin, aabbMax);

		aabbMin = aabbCallback.m_aabbMin;
		aabbMax = aabbCallback.m_aabbMax;
	}

	/// get read and write access to a subpart of a triangle mesh
	/// this subpart has a continuous array of vertices and indices
	/// in this way the mesh can be handled as chunks of memory with striding
	/// very similar to OpenGL vertexarray support
	/// make a call to unLockVertexBase when the read and write access is finished
	virtual void getLockedVertexIndexBase(unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& stride, unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart = 0) = 0;

	virtual void getLockedReadOnlyVertexIndexBase(const unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& stride, const unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart = 0) const = 0;

	/// unLockVertexBase finishes the access to a subpart of the triangle mesh
	/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
	virtual void unLockVertexBase(int subpart) = 0;

	virtual void unLockReadOnlyVertexBase(int subpart) const = 0;

	/// getNumSubParts returns the number of separate subparts
	/// each subpart has a continuous array of vertices and indices
	virtual int getNumSubParts() const = 0;

	virtual void preallocateVertices(int numverts) = 0;
	virtual void preallocateIndices(int numindices) = 0;

	virtual bool hasPremadeAabb() const { return false; }
	virtual void setPremadeAabb(const btVector3& aabbMin, const btVector3& aabbMax) const
	{
		(void)aabbMin;
		(void)aabbMax;
	}
	virtual void getPremadeAabb(btVector3 * aabbMin, btVector3 * aabbMax) const
	{
		(void)aabbMin;
		(void)aabbMax;
	}

	const btVector3& getScaling() const
	{
		return m_scaling;
	}
	void setScaling(const btVector3& scaling)
	{
		m_scaling = scaling;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const
	{
		btStridingMeshInterfaceData* trimeshData = (btStridingMeshInterfaceData*)dataBuffer;

		trimeshData->m_numMeshParts = getNumSubParts();

		//void* uniquePtr = 0;

		trimeshData->m_meshPartsPtr = 0;

		if (trimeshData->m_numMeshParts)
		{
			btChunk* chunk = serializer->allocate(sizeof(btMeshPartData), trimeshData->m_numMeshParts);
			btMeshPartData* memPtr = (btMeshPartData*)chunk->m_oldPtr;
			trimeshData->m_meshPartsPtr = (btMeshPartData*)serializer->getUniquePointer(memPtr);

			//	int numtotalphysicsverts = 0;
			int part, graphicssubparts = getNumSubParts();
			const unsigned char* vertexbase;
			const unsigned char* indexbase;
			int indexstride;
			PHY_ScalarType type;
			PHY_ScalarType gfxindextype;
			int stride, numverts, numtriangles;
			int gfxindex;
			//	btVector3 triangle[3];

			//	btVector3 meshScaling = getScaling();

			///if the number of parts is big, the performance might drop due to the innerloop switch on indextype
			for (part = 0; part < graphicssubparts; part++, memPtr++)
			{
				getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numtriangles, gfxindextype, part);
				memPtr->m_numTriangles = numtriangles;  //indices = 3*numtriangles
				memPtr->m_numVertices = numverts;
				memPtr->m_indices16 = 0;
				memPtr->m_indices32 = 0;
				memPtr->m_3indices16 = 0;
				memPtr->m_3indices8 = 0;
				memPtr->m_vertices3f = 0;
				memPtr->m_vertices3d = 0;

				switch (gfxindextype)
				{
					case PHY_INTEGER:
					{
						int numindices = numtriangles * 3;

						if (numindices)
						{
							btChunk* chunk = serializer->allocate(sizeof(btIntIndexData), numindices);
							btIntIndexData* tmpIndices = (btIntIndexData*)chunk->m_oldPtr;
							memPtr->m_indices32 = (btIntIndexData*)serializer->getUniquePointer(tmpIndices);
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned int* tri_indices = (unsigned int*)(indexbase + gfxindex * indexstride);
								tmpIndices[gfxindex * 3].m_value = tri_indices[0];
								tmpIndices[gfxindex * 3 + 1].m_value = tri_indices[1];
								tmpIndices[gfxindex * 3 + 2].m_value = tri_indices[2];
							}
							serializer->finalizeChunk(chunk, "btIntIndexData", BT_ARRAY_CODE, (void*)chunk->m_oldPtr);
						}
						break;
					}
					case PHY_SHORT:
					{
						if (numtriangles)
						{
							btChunk* chunk = serializer->allocate(sizeof(btShortIntIndexTripletData), numtriangles);
							btShortIntIndexTripletData* tmpIndices = (btShortIntIndexTripletData*)chunk->m_oldPtr;
							memPtr->m_3indices16 = (btShortIntIndexTripletData*)serializer->getUniquePointer(tmpIndices);
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned short int* tri_indices = (unsigned short int*)(indexbase + gfxindex * indexstride);
								tmpIndices[gfxindex].m_values[0] = tri_indices[0];
								tmpIndices[gfxindex].m_values[1] = tri_indices[1];
								tmpIndices[gfxindex].m_values[2] = tri_indices[2];
								// Fill padding with zeros to appease msan.
								tmpIndices[gfxindex].m_pad[0] = 0;
								tmpIndices[gfxindex].m_pad[1] = 0;
							}
							serializer->finalizeChunk(chunk, "btShortIntIndexTripletData", BT_ARRAY_CODE, (void*)chunk->m_oldPtr);
						}
						break;
					}
					case PHY_UCHAR:
					{
						if (numtriangles)
						{
							btChunk* chunk = serializer->allocate(sizeof(btCharIndexTripletData), numtriangles);
							btCharIndexTripletData* tmpIndices = (btCharIndexTripletData*)chunk->m_oldPtr;
							memPtr->m_3indices8 = (btCharIndexTripletData*)serializer->getUniquePointer(tmpIndices);
							for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
							{
								unsigned char* tri_indices = (unsigned char*)(indexbase + gfxindex * indexstride);
								tmpIndices[gfxindex].m_values[0] = tri_indices[0];
								tmpIndices[gfxindex].m_values[1] = tri_indices[1];
								tmpIndices[gfxindex].m_values[2] = tri_indices[2];
								// Fill padding with zeros to appease msan.
								tmpIndices[gfxindex].m_pad = 0;
							}
							serializer->finalizeChunk(chunk, "btCharIndexTripletData", BT_ARRAY_CODE, (void*)chunk->m_oldPtr);
						}
						break;
					}
					default:
					{
						btAssert(0);
						//unknown index type
					}
				}

				switch (type)
				{
					case PHY_FLOAT:
					{
						float* graphicsbase;

						if (numverts)
						{
							btChunk* chunk = serializer->allocate(sizeof(btVector3FloatData), numverts);
							btVector3FloatData* tmpVertices = (btVector3FloatData*)chunk->m_oldPtr;
							memPtr->m_vertices3f = (btVector3FloatData*)serializer->getUniquePointer(tmpVertices);
							for (int i = 0; i < numverts; i++)
							{
								graphicsbase = (float*)(vertexbase + i * stride);
								tmpVertices[i].m_floats[0] = graphicsbase[0];
								tmpVertices[i].m_floats[1] = graphicsbase[1];
								tmpVertices[i].m_floats[2] = graphicsbase[2];
							}
							serializer->finalizeChunk(chunk, "btVector3FloatData", BT_ARRAY_CODE, (void*)chunk->m_oldPtr);
						}
						break;
					}

					case PHY_DOUBLE:
					{
						if (numverts)
						{
							btChunk* chunk = serializer->allocate(sizeof(btVector3DoubleData), numverts);
							btVector3DoubleData* tmpVertices = (btVector3DoubleData*)chunk->m_oldPtr;
							memPtr->m_vertices3d = (btVector3DoubleData*)serializer->getUniquePointer(tmpVertices);
							for (int i = 0; i < numverts; i++)
							{
								double* graphicsbase = (double*)(vertexbase + i * stride);  //for now convert to float, might leave it at double
								tmpVertices[i].m_floats[0] = graphicsbase[0];
								tmpVertices[i].m_floats[1] = graphicsbase[1];
								tmpVertices[i].m_floats[2] = graphicsbase[2];
							}
							serializer->finalizeChunk(chunk, "btVector3DoubleData", BT_ARRAY_CODE, (void*)chunk->m_oldPtr);
						}
						break;
					}

					default:
						btAssert((type == PHY_FLOAT) || (type == PHY_DOUBLE));
				}

				unLockReadOnlyVertexBase(part);
			}

			serializer->finalizeChunk(chunk, "btMeshPartData", BT_ARRAY_CODE, chunk->m_oldPtr);
		}

		// Fill padding with zeros to appease msan.
		memset(trimeshData->m_padding, 0, sizeof(trimeshData->m_padding));

		m_scaling.serializeFloat(trimeshData->m_scaling);
		return "btStridingMeshInterfaceData";
	}
};

SIMD_FORCE_INLINE int btStridingMeshInterface::calculateSerializeBufferSize() const
{
	return sizeof(btStridingMeshInterfaceData);
}

#endif  //BT_STRIDING_MESHINTERFACE_H
