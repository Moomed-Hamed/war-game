#define BT_USE_SSE_IN_API

#include "btConvexShape.h"
#include "btTriangleShape.h"
#include "btSphereShape.h"
#include "btCylinderShape.h"
#include "btConeShape.h"
#include "btCapsuleShape.h"
#include "btConvexHullShape.h"
#include "btConvexPointCloudShape.h"

btVector3 btConvexShape::localGetSupportVertexWithoutMarginNonVirtual(const btVector3& localDir) const
{
	switch (m_shapeType)
	{
		case SPHERE_SHAPE_PROXYTYPE:
		{
			return btVector3(0, 0, 0);
		}
		case BOX_SHAPE_PROXYTYPE:
		{
			btBoxShape* convexShape = (btBoxShape*)this;
			const btVector3& halfExtents = convexShape->getImplicitShapeDimensions();

			return btVector3(btFsels(localDir.x(), halfExtents.x(), -halfExtents.x()),
							 btFsels(localDir.y(), halfExtents.y(), -halfExtents.y()),
							 btFsels(localDir.z(), halfExtents.z(), -halfExtents.z()));
		}
		case TRIANGLE_SHAPE_PROXYTYPE:
		{
			btTriangleShape* triangleShape = (btTriangleShape*)this;
			btVector3 dir(localDir.getX(), localDir.getY(), localDir.getZ());
			btVector3* vertices = &triangleShape->m_vertices1[0];
			btVector3 dots = dir.dot3(vertices[0], vertices[1], vertices[2]);
			btVector3 sup = vertices[dots.maxAxis()];
			return btVector3(sup.getX(), sup.getY(), sup.getZ());
		}
		case CYLINDER_SHAPE_PROXYTYPE:
		{
			btCylinderShape* cylShape = (btCylinderShape*)this;
			//mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)

			btVector3 halfExtents = cylShape->getImplicitShapeDimensions();
			btVector3 v(localDir.getX(), localDir.getY(), localDir.getZ());
			int cylinderUpAxis = cylShape->getUpAxis();
			int XX(1), YY(0), ZZ(2);

			switch (cylinderUpAxis)
			{
				case 0:
				{
					XX = 1;
					YY = 0;
					ZZ = 2;
				}
				break;
				case 1:
				{
					XX = 0;
					YY = 1;
					ZZ = 2;
				}
				break;
				case 2:
				{
					XX = 0;
					YY = 2;
					ZZ = 1;
				}
				break;
				default:
					btAssert(0);
					break;
			};

			btScalar radius = halfExtents[XX];
			btScalar halfHeight = halfExtents[cylinderUpAxis];

			btVector3 tmp;
			btScalar d;

			btScalar s = btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
			if (s != btScalar(0.0))
			{
				d = radius / s;
				tmp[XX] = v[XX] * d;
				tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				tmp[ZZ] = v[ZZ] * d;
				return btVector3(tmp.getX(), tmp.getY(), tmp.getZ());
			}
			else
			{
				tmp[XX] = radius;
				tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				tmp[ZZ] = btScalar(0.0);
				return btVector3(tmp.getX(), tmp.getY(), tmp.getZ());
			}
		}
		case CAPSULE_SHAPE_PROXYTYPE:
		{
			btVector3 vec0(localDir.getX(), localDir.getY(), localDir.getZ());

			btCapsuleShape* capsuleShape = (btCapsuleShape*)this;
			btScalar halfHeight = capsuleShape->getHalfHeight();
			int capsuleUpAxis = capsuleShape->getUpAxis();

			btVector3 supVec(0, 0, 0);

			btScalar maxDot(btScalar(-BT_LARGE_FLOAT));

			btVector3 vec = vec0;
			btScalar lenSqr = vec.length2();
			if (lenSqr < SIMD_EPSILON * SIMD_EPSILON)
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
			{
				btVector3 pos(0, 0, 0);
				pos[capsuleUpAxis] = halfHeight;

				vtx = pos;
				newDot = vec.dot(vtx);

				if (newDot > maxDot)
				{
					maxDot = newDot;
					supVec = vtx;
				}
			}
			{
				btVector3 pos(0, 0, 0);
				pos[capsuleUpAxis] = -halfHeight;

				vtx = pos;
				newDot = vec.dot(vtx);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					supVec = vtx;
				}
			}
			return btVector3(supVec.getX(), supVec.getY(), supVec.getZ());
		}
		case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
		{
			btConvexPointCloudShape* convexPointCloudShape = (btConvexPointCloudShape*)this;
			btVector3* points = convexPointCloudShape->getUnscaledPoints();
			int numPoints = convexPointCloudShape->getNumPoints();
			return convexHullSupport(localDir, points, numPoints, convexPointCloudShape->getLocalScalingNV());
		}
		case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			btConvexHullShape* convexHullShape = (btConvexHullShape*)this;
			btVector3* points = convexHullShape->getUnscaledPoints();
			int numPoints = convexHullShape->getNumPoints();
			return convexHullSupport(localDir, points, numPoints, convexHullShape->getLocalScalingNV());
		}
		default:
			return this->localGetSupportingVertexWithoutMargin(localDir);
	}

	// should never reach here
	btAssert(0);
	return btVector3(btScalar(0.0f), btScalar(0.0f), btScalar(0.0f));
}

btScalar btConvexShape::getMarginNonVirtual() const
{
	/* TODO: This function should be bumped up to btCollisionShape () */
	switch (m_shapeType)
	{
		case SPHERE_SHAPE_PROXYTYPE:
		{
			btSphereShape* sphereShape = (btSphereShape*)this;
			return sphereShape->getRadius();
		}
		case BOX_SHAPE_PROXYTYPE:
		{
			btBoxShape* convexShape = (btBoxShape*)this;
			return convexShape->getMarginNV();
		}
		case TRIANGLE_SHAPE_PROXYTYPE:
		{
			btTriangleShape* triangleShape = (btTriangleShape*)this;
			return triangleShape->getMarginNV();
		}
		case CYLINDER_SHAPE_PROXYTYPE:
		{
			btCylinderShape* cylShape = (btCylinderShape*)this;
			return cylShape->getMarginNV();
		}
		case CONE_SHAPE_PROXYTYPE:
		{
			btConeShape* conShape = (btConeShape*)this;
			return conShape->getMarginNV();
		}
		case CAPSULE_SHAPE_PROXYTYPE:
		{
			btCapsuleShape* capsuleShape = (btCapsuleShape*)this;
			return capsuleShape->getMarginNV();
		}
		case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
		/* fall through */
		case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			btPolyhedralConvexShape* convexHullShape = (btPolyhedralConvexShape*)this;
			return convexHullShape->getMarginNV();
		}
		default:
			return this->getMargin();
	}

	// should never reach here
	btAssert(0);
	return btScalar(0.0f);
}
void btConvexShape::getAabbNonVirtual(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
{
	switch (m_shapeType)
	{
		case SPHERE_SHAPE_PROXYTYPE:
		{
			btSphereShape* sphereShape = (btSphereShape*)this;
			btScalar radius = sphereShape->getImplicitShapeDimensions().getX();  // * convexShape->getLocalScaling().getX();
			btScalar margin = radius + sphereShape->getMarginNonVirtual();
			const btVector3& center = t.getOrigin();
			btVector3 extent(margin, margin, margin);
			aabbMin = center - extent;
			aabbMax = center + extent;
		}
		break;
		case CYLINDER_SHAPE_PROXYTYPE:
		/* fall through */
		case BOX_SHAPE_PROXYTYPE:
		{
			btBoxShape* convexShape = (btBoxShape*)this;
			btScalar margin = convexShape->getMarginNonVirtual();
			btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
			halfExtents += btVector3(margin, margin, margin);
			btMatrix3x3 abs_b = t.getBasis().absolute();
			btVector3 center = t.getOrigin();
			btVector3 extent = halfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);

			aabbMin = center - extent;
			aabbMax = center + extent;
			break;
		}
		case TRIANGLE_SHAPE_PROXYTYPE:
		{
			btTriangleShape* triangleShape = (btTriangleShape*)this;
			btScalar margin = triangleShape->getMarginNonVirtual();
			for (int i = 0; i < 3; i++)
			{
				btVector3 vec(btScalar(0.), btScalar(0.), btScalar(0.));
				vec[i] = btScalar(1.);

				btVector3 sv = localGetSupportVertexWithoutMarginNonVirtual(vec * t.getBasis());

				btVector3 tmp = t(sv);
				aabbMax[i] = tmp[i] + margin;
				vec[i] = btScalar(-1.);
				tmp = t(localGetSupportVertexWithoutMarginNonVirtual(vec * t.getBasis()));
				aabbMin[i] = tmp[i] - margin;
			}
		}
		break;
		case CAPSULE_SHAPE_PROXYTYPE:
		{
			btCapsuleShape* capsuleShape = (btCapsuleShape*)this;
			btVector3 halfExtents(capsuleShape->getRadius(), capsuleShape->getRadius(), capsuleShape->getRadius());
			int m_upAxis = capsuleShape->getUpAxis();
			halfExtents[m_upAxis] = capsuleShape->getRadius() + capsuleShape->getHalfHeight();
			btMatrix3x3 abs_b = t.getBasis().absolute();
			btVector3 center = t.getOrigin();
			btVector3 extent = halfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
			aabbMin = center - extent;
			aabbMax = center + extent;
		}
		break;
		case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
		case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			btPolyhedralConvexAabbCachingShape* convexHullShape = (btPolyhedralConvexAabbCachingShape*)this;
			btScalar margin = convexHullShape->getMarginNonVirtual();
			convexHullShape->getNonvirtualAabb(t, aabbMin, aabbMax, margin);
		}
		break;
		default:
			this->getAabb(t, aabbMin, aabbMax);
			break;
	}

	// should never reach here
	btAssert(0);
}
