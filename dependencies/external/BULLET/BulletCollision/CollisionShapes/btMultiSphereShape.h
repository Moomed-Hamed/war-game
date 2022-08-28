#ifndef BT_MULTI_SPHERE_MINKOWSKI_H
#define BT_MULTI_SPHERE_MINKOWSKI_H

#define BT_USE_SSE_IN_API

#include "btConvexInternalShape.h"

#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))

struct btPositionAndRadius
{
	btVector3FloatData m_pos;
	float m_radius;
};

struct btMultiSphereShapeData
{
	btConvexInternalShapeData m_convexInternalShapeData;

	btPositionAndRadius* m_localPositionArrayPtr;
	int m_localPositionArraySize;
	char m_padding[4];
};

///The btMultiSphereShape represents the convex hull of a collection of spheres. You can create special capsules or other smooth volumes.
///It is possible to animate the spheres for deformation, but call 'recalcLocalAabb' after changing any sphere position/radius
ATTRIBUTE_ALIGNED16(class)
btMultiSphereShape : public btConvexInternalAabbCachingShape
{
	btAlignedObjectArray<btVector3> m_localPositionArray;
	btAlignedObjectArray<btScalar> m_radiArray;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btMultiSphereShape(const btVector3* positions, const btScalar* radi, int numSpheres)
		: btConvexInternalAabbCachingShape()
	{
		m_shapeType = MULTI_SPHERE_SHAPE_PROXYTYPE;
		//btScalar startMargin = btScalar(BT_LARGE_FLOAT);

		m_localPositionArray.resize(numSpheres);
		m_radiArray.resize(numSpheres);
		for (int i = 0; i < numSpheres; i++)
		{
			m_localPositionArray[i] = positions[i];
			m_radiArray[i] = radi[i];
		}

		recalcLocalAabb();
	}

	///CollisionShape Interface
	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		//as an approximation, take the inertia of the box that bounds the spheres

		btVector3 localAabbMin, localAabbMax;
		getCachedLocalAabb(localAabbMin, localAabbMax);
		btVector3 halfExtents = (localAabbMax - localAabbMin) * btScalar(0.5);

		btScalar lx = btScalar(2.) * (halfExtents.x());
		btScalar ly = btScalar(2.) * (halfExtents.y());
		btScalar lz = btScalar(2.) * (halfExtents.z());

		inertia.setValue(mass / (btScalar(12.0)) * (ly * ly + lz * lz),
						 mass / (btScalar(12.0)) * (lx * lx + lz * lz),
						 mass / (btScalar(12.0)) * (lx * lx + ly * ly));
	}

	/// btConvexShape Interface
	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec0) const
	{
		btVector3 supVec(0, 0, 0);

		btScalar maxDot(btScalar(-BT_LARGE_FLOAT));

		btVector3 vec = vec0;
		btScalar lenSqr = vec.length2();
		if (lenSqr < (SIMD_EPSILON * SIMD_EPSILON))
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

		const btVector3* pos = &m_localPositionArray[0];
		const btScalar* rad = &m_radiArray[0];
		int numSpheres = m_localPositionArray.size();

		for (int k = 0; k < numSpheres; k += 128)
		{
			btVector3 temp[128];
			int inner_count = MIN(numSpheres - k, 128);
			for (long i = 0; i < inner_count; i++)
			{
				temp[i] = (*pos) * m_localScaling + vec * m_localScaling * (*rad) - vec * getMargin();
				pos++;
				rad++;
			}
			long i = vec.maxDot(temp, inner_count, newDot);
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
		for (int j = 0; j < numVectors; j++)
		{
			btScalar maxDot(btScalar(-BT_LARGE_FLOAT));

			const btVector3& vec = vectors[j];

			btVector3 vtx;
			btScalar newDot;

			const btVector3* pos = &m_localPositionArray[0];
			const btScalar* rad = &m_radiArray[0];
			int numSpheres = m_localPositionArray.size();

			for (int k = 0; k < numSpheres; k += 128)
			{
				btVector3 temp[128];
				int inner_count = MIN(numSpheres - k, 128);
				for (long i = 0; i < inner_count; i++)
				{
					temp[i] = (*pos) * m_localScaling + vec * m_localScaling * (*rad) - vec * getMargin();
					pos++;
					rad++;
				}
				long i = vec.maxDot(temp, inner_count, newDot);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					supportVerticesOut[j] = temp[i];
				}
			}
		}
	}

	int getSphereCount() const
	{
		return m_localPositionArray.size();
	}

	const btVector3& getSpherePosition(int index) const
	{
		return m_localPositionArray[index];
	}

	btScalar getSphereRadius(int index) const
	{
		return m_radiArray[index];
	}

	virtual const char* getName() const
	{
		return "MultiSphere";
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const
	{
		btMultiSphereShapeData* shapeData = (btMultiSphereShapeData*)dataBuffer;
		btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

		int numElem = m_localPositionArray.size();
		shapeData->m_localPositionArrayPtr = numElem ? (btPositionAndRadius*)serializer->getUniquePointer((void*)&m_localPositionArray[0]) : 0;

		shapeData->m_localPositionArraySize = numElem;
		if (numElem)
		{
			btChunk* chunk = serializer->allocate(sizeof(btPositionAndRadius), numElem);
			btPositionAndRadius* memPtr = (btPositionAndRadius*)chunk->m_oldPtr;
			for (int i = 0; i < numElem; i++, memPtr++)
			{
				m_localPositionArray[i].serializeFloat(memPtr->m_pos);
				memPtr->m_radius = float(m_radiArray[i]);
			}
			serializer->finalizeChunk(chunk, "btPositionAndRadius", BT_ARRAY_CODE, (void*)&m_localPositionArray[0]);
		}

		// Fill padding with zeros to appease msan.
		memset(shapeData->m_padding, 0, sizeof(shapeData->m_padding));

		return "btMultiSphereShapeData";
	}
};

SIMD_FORCE_INLINE int btMultiSphereShape::calculateSerializeBufferSize() const
{
	return sizeof(btMultiSphereShapeData);
}

#endif  //BT_MULTI_SPHERE_MINKOWSKI_H
