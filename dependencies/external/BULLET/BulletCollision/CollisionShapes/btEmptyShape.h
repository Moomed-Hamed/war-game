#ifndef BT_EMPTY_SHAPE_H
#define BT_EMPTY_SHAPE_H

#include "btConcaveShape.h"

/// The btEmptyShape is a collision shape without actual collision detection shape, so most users should ignore this class.
/// It can be replaced by another shape during runtime, but the inertia tensor should be recomputed.
ATTRIBUTE_ALIGNED16(class)
btEmptyShape : public btConcaveShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btEmptyShape() : btConcaveShape()
	{
		m_shapeType = EMPTY_SHAPE_PROXYTYPE;
	}

	virtual ~btEmptyShape(){};

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btVector3 margin(getMargin(), getMargin(), getMargin());

		aabbMin = t.getOrigin() - margin;

		aabbMax = t.getOrigin() + margin;
	}


	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
	}
	virtual const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		btAssert(0);
	}

	virtual const char* getName() const
	{
		return "Empty";
	}

	virtual void processAllTriangles(btTriangleCallback*, const btVector3&, const btVector3&) const
	{
	}

protected:
	btVector3 m_localScaling;
};

#endif  //BT_EMPTY_SHAPE_H
