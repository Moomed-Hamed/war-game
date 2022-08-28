#ifndef BT_TRIANGLE_CALLBACK_H
#define BT_TRIANGLE_CALLBACK_H

#include "LinearMath/btVector3.h"

///The btTriangleCallback provides a callback for each overlapping triangle when calling processAllTriangles.
///This callback is called by processAllTriangles for all btConcaveShape derived class, such as  btBvhTriangleMeshShape, btStaticPlaneShape and btHeightfieldTerrainShape.
class btTriangleCallback
{
public:
	virtual ~btTriangleCallback(){};
	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex) = 0;
};

class btInternalTriangleIndexCallback
{
public:
	virtual ~btInternalTriangleIndexCallback(){};
	virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex) = 0;
};

#endif  //BT_TRIANGLE_CALLBACK_H
