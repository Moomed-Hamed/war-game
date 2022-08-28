#ifndef BT_BOX_2D_BOX_2D__COLLISION_ALGORITHM_H
#define BT_BOX_2D_BOX_2D__COLLISION_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBox2dShape.h"

// btBox2dBox2dCollisionAlgorithm, with modified b2CollidePolygons routines from the Box2D library.
// The modifications include: switching from b2Vec to btVector3, redefinition of b2Dot, b2Cross

class btPersistentManifold;

static struct ClipVertex
{
	btVector3 v;
	int id;
};

#define b2Dot(a, b) (a).dot(b)
#define b2Mul(a, b) (a) * (b)
#define b2MulT(a, b) (a).transpose() * (b)
#define b2Cross(a, b) (a).cross(b)
#define btCrossS(a, s) btVector3(s* a.getY(), -s* a.getX(), 0.f)

static int b2_maxManifoldPoints = 2;

static int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2], const btVector3& normal, btScalar offset)
{
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	btScalar distance0 = b2Dot(normal, vIn[0].v) - offset;
	btScalar distance1 = b2Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		btScalar interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > 0.0f)
		{
			vOut[numOut].id = vIn[0].id;
		}
		else
		{
			vOut[numOut].id = vIn[1].id;
		}
		++numOut;
	}

	return numOut;
}

// Find the separation between poly1 and poly2 for a give edge normal on poly1.
static btScalar EdgeSeparation(const btBox2dShape* poly1, const btTransform& xf1, int edge1, const btBox2dShape* poly2, const btTransform& xf2)
{
	const btVector3* vertices1 = poly1->getVertices();
	const btVector3* normals1 = poly1->getNormals();

	int count2 = poly2->getVertexCount();
	const btVector3* vertices2 = poly2->getVertices();

	btAssert(0 <= edge1 && edge1 < poly1->getVertexCount());

	// Convert normal from poly1's frame into poly2's frame.
	btVector3 normal1World = b2Mul(xf1.getBasis(), normals1[edge1]);
	btVector3 normal1 = b2MulT(xf2.getBasis(), normal1World);

	// Find support vertex on poly2 for -normal.
	int index = 0;
	btScalar minDot = BT_LARGE_FLOAT;

	if (count2 > 0)
		index = (int)normal1.minDot(vertices2, count2, minDot);

	btVector3 v1 = b2Mul(xf1, vertices1[edge1]);
	btVector3 v2 = b2Mul(xf2, vertices2[index]);
	btScalar separation = b2Dot(v2 - v1, normal1World);
	return separation;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static btScalar FindMaxSeparation(int* edgeIndex, const btBox2dShape* poly1, const btTransform& xf1, const btBox2dShape* poly2, const btTransform& xf2)
{
	int count1 = poly1->getVertexCount();
	const btVector3* normals1 = poly1->getNormals();

	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	btVector3 d = b2Mul(xf2, poly2->getCentroid()) - b2Mul(xf1, poly1->getCentroid());
	btVector3 dLocal1 = b2MulT(xf1.getBasis(), d);

	// Find edge normal on poly1 that has the largest projection onto d.
	int edge = 0;
	btScalar maxDot;
	if (count1 > 0)
		edge = (int)dLocal1.maxDot(normals1, count1, maxDot);

	// Get the separation for the edge normal.
	btScalar s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
	if (s > 0.0f)
	{
		return s;
	}

	// Check the separation for the previous edge normal.
	int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
	btScalar sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
	if (sPrev > 0.0f)
	{
		return sPrev;
	}

	// Check the separation for the next edge normal.
	int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
	btScalar sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
	if (sNext > 0.0f)
	{
		return sNext;
	}

	// Find the best edge and the search direction.
	int bestEdge;
	btScalar bestSeparation;
	int increment;
	if (sPrev > s && sPrev > sNext)
	{
		increment = -1;
		bestEdge = prevEdge;
		bestSeparation = sPrev;
	}
	else if (sNext > s)
	{
		increment = 1;
		bestEdge = nextEdge;
		bestSeparation = sNext;
	}
	else
	{
		*edgeIndex = edge;
		return s;
	}

	// Perform a local search for the best edge normal.
	for (;;)
	{
		if (increment == -1)
			edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
		else
			edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

		s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
		if (s > 0.0f)
		{
			return s;
		}

		if (s > bestSeparation)
		{
			bestEdge = edge;
			bestSeparation = s;
		}
		else
		{
			break;
		}
	}

	*edgeIndex = bestEdge;
	return bestSeparation;
}

static void FindIncidentEdge(ClipVertex c[2], const btBox2dShape* poly1, const btTransform& xf1, int edge1, const btBox2dShape* poly2, const btTransform& xf2)
{
	const btVector3* normals1 = poly1->getNormals();

	int count2 = poly2->getVertexCount();
	const btVector3* vertices2 = poly2->getVertices();
	const btVector3* normals2 = poly2->getNormals();

	btAssert(0 <= edge1 && edge1 < poly1->getVertexCount());

	// Get the normal of the reference edge in poly2's frame.
	btVector3 normal1 = b2MulT(xf2.getBasis(), b2Mul(xf1.getBasis(), normals1[edge1]));

	// Find the incident edge on poly2.
	int index = 0;
	btScalar minDot = BT_LARGE_FLOAT;
	for (int i = 0; i < count2; ++i)
	{
		btScalar dot = b2Dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int i1 = index;
	int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = b2Mul(xf2, vertices2[i1]);
	c[1].v = b2Mul(xf2, vertices2[i2]);
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
void b2CollidePolygons(btManifoldResult* manifold,
					   const btBox2dShape* polyA, const btTransform& xfA,
					   const btBox2dShape* polyB, const btTransform& xfB)
{
	int edgeA = 0;
	btScalar separationA = FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
	if (separationA > 0.0f)
		return;

	int edgeB = 0;
	btScalar separationB = FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
	if (separationB > 0.0f)
		return;

	const btBox2dShape* poly1;  // reference poly
	const btBox2dShape* poly2;  // incident poly
	btTransform xf1, xf2;
	int edge1;  // reference edge
	unsigned char flip;
	const btScalar k_relativeTol = 0.98f;
	const btScalar k_absoluteTol = 0.001f;

	// TODO_ERIN use "radius" of poly for absolute tolerance.
	if (separationB > k_relativeTol * separationA + k_absoluteTol)
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		flip = 1;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		flip = 0;
	}

	ClipVertex incidentEdge[2];
	FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	int count1 = poly1->getVertexCount();
	const btVector3* vertices1 = poly1->getVertices();

	btVector3 v11 = vertices1[edge1];
	btVector3 v12 = edge1 + 1 < count1 ? vertices1[edge1 + 1] : vertices1[0];

	//btVector3 dv = v12 - v11;
	btVector3 sideNormal = b2Mul(xf1.getBasis(), v12 - v11);
	sideNormal.normalize();
	btVector3 frontNormal = btCrossS(sideNormal, 1.0f);

	v11 = b2Mul(xf1, v11);
	v12 = b2Mul(xf1, v12);

	btScalar frontOffset = b2Dot(frontNormal, v11);
	btScalar sideOffset1 = -b2Dot(sideNormal, v11);
	btScalar sideOffset2 = b2Dot(sideNormal, v12);

	// Clip incident edge against extruded edge1 side edges.
	ClipVertex clipPoints1[2];
	clipPoints1[0].v.setValue(0, 0, 0);
	clipPoints1[1].v.setValue(0, 0, 0);

	ClipVertex clipPoints2[2];
	clipPoints2[0].v.setValue(0, 0, 0);
	clipPoints2[1].v.setValue(0, 0, 0);

	int np;

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);

	if (np < 2)
		return;

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, sideOffset2);

	if (np < 2)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	btVector3 manifoldNormal = flip ? -frontNormal : frontNormal;

	int pointCount = 0;
	for (int i = 0; i < b2_maxManifoldPoints; ++i)
	{
		btScalar separation = b2Dot(frontNormal, clipPoints2[i].v) - frontOffset;

		if (separation <= 0.0f)
		{
			//b2ManifoldPoint* cp = manifold->points + pointCount;
			//btScalar separation = separation;
			//cp->localPoint1 = b2MulT(xfA, clipPoints2[i].v);
			//cp->localPoint2 = b2MulT(xfB, clipPoints2[i].v);

			manifold->addContactPoint(-manifoldNormal, clipPoints2[i].v, separation);

			//			cp->id = clipPoints2[i].id;
			//			cp->id.features.flip = flip;
			++pointCount;
		}
	}

	//	manifold->pointCount = pointCount;}
}

///box-box collision detection
class btBox2dBox2dCollisionAlgorithm : public btActivatingCollisionAlgorithm
{
	bool m_ownManifold;
	btPersistentManifold* m_manifoldPtr;

public:
	btBox2dBox2dCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
		: btActivatingCollisionAlgorithm(ci) {}

	virtual void processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		if (!m_manifoldPtr)
			return;

		const btBox2dShape* box0 = (const btBox2dShape*)body0Wrap->getCollisionShape();
		const btBox2dShape* box1 = (const btBox2dShape*)body1Wrap->getCollisionShape();

		resultOut->setPersistentManifold(m_manifoldPtr);

		b2CollidePolygons(resultOut, box0, body0Wrap->getWorldTransform(), box1, body1Wrap->getWorldTransform());

		//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
		if (m_ownManifold)
		{
			resultOut->refreshContactPoints();
		}
	}

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		return 1.f; // not yet
	}

	btBox2dBox2dCollisionAlgorithm(btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* obj0Wrap, const btCollisionObjectWrapper* obj1Wrap)
		: btActivatingCollisionAlgorithm(ci, obj0Wrap, obj1Wrap),
		  m_ownManifold(false),
		  m_manifoldPtr(mf)
	{
		if (!m_manifoldPtr && m_dispatcher->needsCollision(obj0Wrap->getCollisionObject(), obj1Wrap->getCollisionObject()))
		{
			m_manifoldPtr = m_dispatcher->getNewManifold(obj0Wrap->getCollisionObject(), obj1Wrap->getCollisionObject());
			m_ownManifold = true;
		}
	}

	virtual ~btBox2dBox2dCollisionAlgorithm()
	{
		if (m_ownManifold)
		{
			if (m_manifoldPtr)
				m_dispatcher->releaseManifold(m_manifoldPtr);
		}
	}

	virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	struct CreateFunc : public btCollisionAlgorithmCreateFunc
	{
		virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		{
			int bbsize = sizeof(btBox2dBox2dCollisionAlgorithm);
			void* ptr = ci.m_dispatcher1->allocateCollisionAlgorithm(bbsize);
			return new (ptr) btBox2dBox2dCollisionAlgorithm(0, ci, body0Wrap, body1Wrap);
		}
	};
};

#endif  //BT_BOX_2D_BOX_2D__COLLISION_ALGORITHM_H
