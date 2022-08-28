#ifndef BT_CONE_MINKOWSKI_H
#define BT_CONE_MINKOWSKI_H

#include "btConvexInternalShape.h"

///The btConeShape implements a cone shape primitive, centered around the origin and aligned with the Y axis. The btConeShapeX is aligned around the X axis and btConeShapeZ around the Z axis.
ATTRIBUTE_ALIGNED16(class)
btConeShape : public btConvexInternalShape

{
	btScalar m_sinAngle;
	btScalar m_radius;
	btScalar m_height;
	int m_coneIndices[3];
	btVector3 coneLocalSupport(const btVector3& v) const
	{
		btScalar halfHeight = m_height * btScalar(0.5);

		if (v[m_coneIndices[1]] > v.length() * m_sinAngle)
		{
			btVector3 tmp;

			tmp[m_coneIndices[0]] = btScalar(0.);
			tmp[m_coneIndices[1]] = halfHeight;
			tmp[m_coneIndices[2]] = btScalar(0.);
			return tmp;
		}
		else
		{
			btScalar s = btSqrt(v[m_coneIndices[0]] * v[m_coneIndices[0]] + v[m_coneIndices[2]] * v[m_coneIndices[2]]);
			if (s > SIMD_EPSILON)
			{
				btScalar d = m_radius / s;
				btVector3 tmp;
				tmp[m_coneIndices[0]] = v[m_coneIndices[0]] * d;
				tmp[m_coneIndices[1]] = -halfHeight;
				tmp[m_coneIndices[2]] = v[m_coneIndices[2]] * d;
				return tmp;
			}
			else
			{
				btVector3 tmp;
				tmp[m_coneIndices[0]] = btScalar(0.);
				tmp[m_coneIndices[1]] = -halfHeight;
				tmp[m_coneIndices[2]] = btScalar(0.);
				return tmp;
			}
		}
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btConeShape(btScalar radius, btScalar height) : btConvexInternalShape(),
													m_radius(radius),
													m_height(height)
	{
		m_shapeType = CONE_SHAPE_PROXYTYPE;
		setConeUpIndex(1);
		btVector3 halfExtents;
		m_sinAngle = (m_radius / btSqrt(m_radius * m_radius + m_height * m_height));
	}

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
	{
		btVector3 supVertex = coneLocalSupport(vec);
		if (getMargin() != btScalar(0.))
		{
			btVector3 vecnorm = vec;
			if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
			{
				vecnorm.setValue(btScalar(-1.), btScalar(-1.), btScalar(-1.));
			}
			vecnorm.normalize();
			supVertex += getMargin() * vecnorm;
		}
		return supVertex;
	}
	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const
	{
		return coneLocalSupport(vec);
	}
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		for (int i = 0; i < numVectors; i++)
		{
			const btVector3& vec = vectors[i];
			supportVerticesOut[i] = coneLocalSupport(vec);
		}
	}

	btScalar getRadius() const { return m_radius; }
	btScalar getHeight() const { return m_height; }

	void setRadius(const btScalar radius)
	{
		m_radius = radius;
	}
	void setHeight(const btScalar height)
	{
		m_height = height;
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		btTransform identity;
		identity.setIdentity();
		btVector3 aabbMin, aabbMax;
		getAabb(identity, aabbMin, aabbMax);

		btVector3 halfExtents = (aabbMax - aabbMin) * btScalar(0.5);

		btScalar margin = getMargin();

		btScalar lx = btScalar(2.) * (halfExtents.x() + margin);
		btScalar ly = btScalar(2.) * (halfExtents.y() + margin);
		btScalar lz = btScalar(2.) * (halfExtents.z() + margin);
		const btScalar x2 = lx * lx;
		const btScalar y2 = ly * ly;
		const btScalar z2 = lz * lz;
		const btScalar scaledmass = mass * btScalar(0.08333333);

		inertia = scaledmass * (btVector3(y2 + z2, x2 + z2, x2 + y2));

		//		inertia.x() = scaledmass * (y2+z2);
		//		inertia.y() = scaledmass * (x2+z2);
		//		inertia.z() = scaledmass * (x2+y2);
	}

	virtual const char* getName() const
	{
		return "Cone";
	}

	///choose upAxis index
	void setConeUpIndex(int upIndex)
	{
		switch (upIndex)
		{
			case 0:
				m_coneIndices[0] = 1;
				m_coneIndices[1] = 0;
				m_coneIndices[2] = 2;
				break;
			case 1:
				m_coneIndices[0] = 0;
				m_coneIndices[1] = 1;
				m_coneIndices[2] = 2;
				break;
			case 2:
				m_coneIndices[0] = 0;
				m_coneIndices[1] = 2;
				m_coneIndices[2] = 1;
				break;
			default:
				btAssert(0);
		};

		m_implicitShapeDimensions[m_coneIndices[0]] = m_radius;
		m_implicitShapeDimensions[m_coneIndices[1]] = m_height;
		m_implicitShapeDimensions[m_coneIndices[2]] = m_radius;
	}

	int getConeUpIndex() const
	{
		return m_coneIndices[1];
	}

	virtual btVector3 getAnisotropicRollingFrictionDirection() const
	{
		return btVector3(0, 1, 0);
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		int axis = m_coneIndices[1];
		int r1 = m_coneIndices[0];
		int r2 = m_coneIndices[2];
		m_height *= scaling[axis] / m_localScaling[axis];
		m_radius *= (scaling[r1] / m_localScaling[r1] + scaling[r2] / m_localScaling[r2]) / 2;
		m_sinAngle = (m_radius / btSqrt(m_radius * m_radius + m_height * m_height));
		btConvexInternalShape::setLocalScaling(scaling);
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

///btConeShape implements a Cone shape, around the X axis
class btConeShapeX : public btConeShape
{
public:
	btConeShapeX(btScalar radius, btScalar height) : btConeShape(radius, height)
	{
		setConeUpIndex(0);
	}

	virtual btVector3 getAnisotropicRollingFrictionDirection() const
	{
		return btVector3(1, 0, 0);
	}

	//debugging
	virtual const char* getName() const
	{
		return "ConeX";
	}
};

///btConeShapeZ implements a Cone shape, around the Z axis
class btConeShapeZ : public btConeShape
{
public:
	btConeShapeZ(btScalar radius, btScalar height) : btConeShape(radius, height)
	{
		setConeUpIndex(2);
	}


	virtual btVector3 getAnisotropicRollingFrictionDirection() const
	{
		return btVector3(0, 0, 1);
	}

	//debugging
	virtual const char* getName() const
	{
		return "ConeZ";
	}
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btConeShapeData
{
	btConvexInternalShapeData m_convexInternalShapeData;

	int m_upIndex;

	char m_padding[4];
};

SIMD_FORCE_INLINE int btConeShape::calculateSerializeBufferSize() const
{
	return sizeof(btConeShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btConeShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btConeShapeData* shapeData = (btConeShapeData*)dataBuffer;

	btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	shapeData->m_upIndex = m_coneIndices[1];

	// Fill padding with zeros to appease msan.
	shapeData->m_padding[0] = 0;
	shapeData->m_padding[1] = 0;
	shapeData->m_padding[2] = 0;
	shapeData->m_padding[3] = 0;

	return "btConeShapeData";
}

#endif  //BT_CONE_MINKOWSKI_H
