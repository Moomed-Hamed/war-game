#ifndef BT_GEOMETRY_UTIL_H
#define BT_GEOMETRY_UTIL_H

#include "btVector3.h"
#include "btAlignedObjectArray.h"

///The btGeometryUtil helper class provides a few methods to convert between plane equations and vertices.
class btGeometryUtil
{
public:
	static void getPlaneEquationsFromVertices(btAlignedObjectArray<btVector3>& vertices, btAlignedObjectArray<btVector3>& planeEquationsOut);

	static void getVerticesFromPlaneEquations(const btAlignedObjectArray<btVector3>& planeEquations, btAlignedObjectArray<btVector3>& verticesOut);

	static bool isInside(const btAlignedObjectArray<btVector3>& vertices, const btVector3& planeNormal, btScalar margin);

	static bool isPointInsidePlanes(const btAlignedObjectArray<btVector3>& planeEquations, const btVector3& point, btScalar margin);

	static bool areVerticesBehindPlane(const btVector3& planeNormal, const btAlignedObjectArray<btVector3>& vertices, btScalar margin);
};

#endif  //BT_GEOMETRY_UTIL_H
