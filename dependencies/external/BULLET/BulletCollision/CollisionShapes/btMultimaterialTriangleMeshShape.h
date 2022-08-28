/// This file was created by Alex Silverman

#ifndef BT_BVH_TRIANGLE_MATERIAL_MESH_SHAPE_H
#define BT_BVH_TRIANGLE_MATERIAL_MESH_SHAPE_H

#include "btBvhTriangleMeshShape.h"
#include "btMaterial.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h"

///The BvhTriangleMaterialMeshShape extends the btBvhTriangleMeshShape. Its main contribution is the interface into a material array, which allows per-triangle friction and restitution.
ATTRIBUTE_ALIGNED16(class)
btMultimaterialTriangleMeshShape : public btBvhTriangleMeshShape
{
	btAlignedObjectArray<btMaterial *> m_materialList;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btMultimaterialTriangleMeshShape(btStridingMeshInterface * meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true) : btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, buildBvh)
	{
		m_shapeType = MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE;

		const unsigned char *vertexbase;
		int numverts;
		PHY_ScalarType type;
		int stride;
		const unsigned char *indexbase;
		int indexstride;
		int numfaces;
		PHY_ScalarType indicestype;

		//m_materialLookup = (int**)(btAlignedAlloc(sizeof(int*) * meshInterface->getNumSubParts(), 16));

		for (int i = 0; i < meshInterface->getNumSubParts(); i++)
		{
			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				i);
			//m_materialLookup[i] = (int*)(btAlignedAlloc(sizeof(int) * numfaces, 16));
		}
	}

	///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
	btMultimaterialTriangleMeshShape(btStridingMeshInterface * meshInterface, bool useQuantizedAabbCompression, const btVector3 &bvhAabbMin, const btVector3 &bvhAabbMax, bool buildBvh = true) : btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax, buildBvh)
	{
		m_shapeType = MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE;

		const unsigned char *vertexbase;
		int numverts;
		PHY_ScalarType type;
		int stride;
		const unsigned char *indexbase;
		int indexstride;
		int numfaces;
		PHY_ScalarType indicestype;

		//m_materialLookup = (int**)(btAlignedAlloc(sizeof(int*) * meshInterface->getNumSubParts(), 16));

		for (int i = 0; i < meshInterface->getNumSubParts(); i++)
		{
			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				i);
			//m_materialLookup[i] = (int*)(btAlignedAlloc(sizeof(int) * numfaces * 2, 16));
		}
	}

	virtual ~btMultimaterialTriangleMeshShape()
	{
		/*
        for(int i = 0; i < m_meshInterface->getNumSubParts(); i++)
        {
            btAlignedFree(m_materialValues[i]);
            m_materialLookup[i] = NULL;
        }
        btAlignedFree(m_materialValues);
        m_materialLookup = NULL;
*/
	}
	//debugging
	virtual const char *getName() const { return "MULTIMATERIALTRIANGLEMESH"; }

	///Obtains the material for a specific triangle
	const btMaterial *getMaterialProperties(int partID, int triIndex)
	{
		const unsigned char *materialBase = 0;
		int numMaterials;
		PHY_ScalarType materialType;
		int materialStride;
		const unsigned char *triangleMaterialBase = 0;
		int numTriangles;
		int triangleMaterialStride;
		PHY_ScalarType triangleType;

		((btTriangleIndexVertexMaterialArray *)m_meshInterface)->getLockedReadOnlyMaterialBase(&materialBase, numMaterials, materialType, materialStride, &triangleMaterialBase, numTriangles, triangleMaterialStride, triangleType, partID);

		// return the pointer to the place with the friction for the triangle
		// TODO: This depends on whether it's a moving mesh or not
		// BUG IN GIMPACT
		//return (btScalar*)(&materialBase[triangleMaterialBase[(triIndex-1) * triangleMaterialStride] * materialStride]);
		int *matInd = (int *)(&(triangleMaterialBase[(triIndex * triangleMaterialStride)]));
		btMaterial *matVal = (btMaterial *)(&(materialBase[*matInd * materialStride]));
		return (matVal);
	}
};

#endif  //BT_BVH_TRIANGLE_MATERIAL_MESH_SHAPE_H
