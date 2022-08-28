#ifndef BT_CONVEX_SHAPE_INTERFACE1
#define BT_CONVEX_SHAPE_INTERFACE1

#include "btCollisionShape.h"
#include "btCollisionMargin.h"

static btVector3 convexHullSupport(const btVector3& localDirOrg, const btVector3* points, int numPoints, const btVector3& localScaling)
{
	btVector3 vec = localDirOrg * localScaling;

	btScalar maxDot;
	long ptIndex = vec.maxDot(points, numPoints, maxDot);
	btAssert(ptIndex >= 0);
	if (ptIndex < 0)
	{
		ptIndex = 0;
	}
	btVector3 supVec = points[ptIndex] * localScaling;
	return supVec;
}

#define MAX_PREFERRED_PENETRATION_DIRECTIONS 10

/// The btConvexShape is an abstract shape interface, implemented by all convex shapes such as btBoxShape, btConvexHullShape etc.
/// It describes general convex shapes using the localGetSupportingVertex interface, used by collision detectors such as btGjkPairDetector.
ATTRIBUTE_ALIGNED16(class)
btConvexShape : public btCollisionShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btConvexShape(){};
	virtual ~btConvexShape(){};

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const = 0;

////////
	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const = 0;

	btVector3 localGetSupportVertexWithoutMarginNonVirtual(const btVector3& vec) const;
	btVector3 localGetSupportVertexNonVirtual(const btVector3& localDir) const
	{
		btVector3 localDirNorm = localDir;
		if (localDirNorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
		{
			localDirNorm.setValue(btScalar(-1.), btScalar(-1.), btScalar(-1.));
		}
		localDirNorm.normalize();

		return localGetSupportVertexWithoutMarginNonVirtual(localDirNorm) + getMarginNonVirtual() * localDirNorm;
	}
	btScalar getMarginNonVirtual() const;
	void getAabbNonVirtual(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;

	virtual void project(const btTransform& trans, const btVector3& dir, btScalar& min, btScalar& max, btVector3& witnesPtMin, btVector3& witnesPtMax) const
	{
		btVector3 localAxis = dir * trans.getBasis();
		btVector3 vtx1 = trans(localGetSupportingVertex(localAxis));
		btVector3 vtx2 = trans(localGetSupportingVertex(-localAxis));

		min = vtx1.dot(dir);
		max = vtx2.dot(dir);
		witnesPtMax = vtx2;
		witnesPtMin = vtx1;

		if (min > max)
		{
			btScalar tmp = min;
			min = max;
			max = tmp;
			witnesPtMax = vtx1;
			witnesPtMin = vtx2;
		}
	}

	//notice that the vectors should be unit length
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const = 0;

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const = 0;

	virtual void getAabbSlow(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const = 0;

	virtual void setLocalScaling(const btVector3& scaling) = 0;
	virtual const btVector3& getLocalScaling() const = 0;

	virtual void setMargin(btScalar margin) = 0;

	virtual btScalar getMargin() const = 0;

	virtual int getNumPreferredPenetrationDirections() const = 0;

	virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const = 0;
};

#endif  //BT_CONVEX_SHAPE_INTERFACE1
