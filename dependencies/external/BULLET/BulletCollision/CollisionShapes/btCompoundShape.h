#ifndef BT_COMPOUND_SHAPE_H
#define BT_COMPOUND_SHAPE_H

#include "btCollisionShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCompoundShapeChildData
{
	btTransformFloatData m_transform;
	btCollisionShapeData* m_childShape;
	int m_childShapeType;
	float m_childMargin;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCompoundShapeData
{
	btCollisionShapeData m_collisionShapeData;
	btCompoundShapeChildData* m_childShapePtr;

	int m_numChildShapes;

	float m_collisionMargin;
};

struct btDbvt;

ATTRIBUTE_ALIGNED16(struct)
btCompoundShapeChild
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btTransform m_transform;
	btCollisionShape* m_childShape;
	int m_childShapeType;
	btScalar m_childMargin;
	struct btDbvtNode* m_node;
};

SIMD_FORCE_INLINE bool operator==(const btCompoundShapeChild& c1, const btCompoundShapeChild& c2)
{
	return (c1.m_transform == c2.m_transform &&
			c1.m_childShape == c2.m_childShape &&
			c1.m_childShapeType == c2.m_childShapeType &&
			c1.m_childMargin == c2.m_childMargin);
}

/// The btCompoundShape allows to store multiple other btCollisionShapes
/// This allows for moving concave collision objects. This is more general then the static concave btBvhTriangleMeshShape.
/// It has an (optional) dynamic aabb tree to accelerate early rejection tests.
/// @todo: This aabb tree can also be use to speed up ray tests on btCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
/// Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor of btCompoundShape)
ATTRIBUTE_ALIGNED16(class)
btCompoundShape : public btCollisionShape
{
protected:
	btAlignedObjectArray<btCompoundShapeChild> m_children;
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;

	btDbvt* m_dynamicAabbTree;

	///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
	int m_updateRevision;

	btScalar m_collisionMargin;

	btVector3 m_localScaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	explicit btCompoundShape(bool enableDynamicAabbTree = true, const int initialChildCapacity = 0)
		: m_localAabbMin(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT)),
		  m_localAabbMax(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT)),
		  m_dynamicAabbTree(0),
		  m_updateRevision(1),
		  m_collisionMargin(btScalar(0.)),
		  m_localScaling(btScalar(1.), btScalar(1.), btScalar(1.))
	{
		m_shapeType = COMPOUND_SHAPE_PROXYTYPE;

		if (enableDynamicAabbTree)
		{
			void* mem = btAlignedAlloc(sizeof(btDbvt), 16);
			m_dynamicAabbTree = new (mem) btDbvt();
			btAssert(mem == m_dynamicAabbTree);
		}

		m_children.reserve(initialChildCapacity);
	}

	virtual ~btCompoundShape()
	{
		if (m_dynamicAabbTree)
		{
			m_dynamicAabbTree->~btDbvt();
			btAlignedFree(m_dynamicAabbTree);
		}
	}

	void addChildShape(const btTransform& localTransform, btCollisionShape* shape)
	{
		m_updateRevision++;
		//m_childTransforms.push_back(localTransform);
		//m_childShapes.push_back(shape);
		btCompoundShapeChild child;
		child.m_node = 0;
		child.m_transform = localTransform;
		child.m_childShape = shape;
		child.m_childShapeType = shape->getShapeType();
		child.m_childMargin = shape->getMargin();

		//extend the local aabbMin/aabbMax
		btVector3 localAabbMin, localAabbMax;
		shape->getAabb(localTransform, localAabbMin, localAabbMax);
		for (int i = 0; i < 3; i++)
		{
			if (m_localAabbMin[i] > localAabbMin[i])
			{
				m_localAabbMin[i] = localAabbMin[i];
			}
			if (m_localAabbMax[i] < localAabbMax[i])
			{
				m_localAabbMax[i] = localAabbMax[i];
			}
		}
		if (m_dynamicAabbTree)
		{
			const btDbvtVolume bounds = btDbvtVolume::FromMM(localAabbMin, localAabbMax);
			size_t index = m_children.size();
			child.m_node = m_dynamicAabbTree->insert(bounds, reinterpret_cast<void*>(index));
		}

		m_children.push_back(child);
	}

	/// Remove all children shapes that contain the specified shape
	virtual void removeChildShape(btCollisionShape * shape)
	{
		m_updateRevision++;
		// Find the children containing the shape specified, and remove those children.
		//note: there might be multiple children using the same shape!
		for (int i = m_children.size() - 1; i >= 0; i--)
		{
			if (m_children[i].m_childShape == shape)
			{
				removeChildShapeByIndex(i);
			}
		}

		recalculateLocalAabb();
	}

	void removeChildShapeByIndex(int childShapeIndex)
	{
		m_updateRevision++;
		btAssert(childShapeIndex >= 0 && childShapeIndex < m_children.size());
		if (m_dynamicAabbTree)
		{
			m_dynamicAabbTree->remove(m_children[childShapeIndex].m_node);
		}
		m_children.swap(childShapeIndex, m_children.size() - 1);
		if (m_dynamicAabbTree)
			m_children[childShapeIndex].m_node->dataAsInt = childShapeIndex;
		m_children.pop_back();
	}

	int getNumChildShapes() const
	{
		return int(m_children.size());
	}

	btCollisionShape* getChildShape(int index)
	{
		return m_children[index].m_childShape;
	}
	const btCollisionShape* getChildShape(int index) const
	{
		return m_children[index].m_childShape;
	}

	btTransform& getChildTransform(int index)
	{
		return m_children[index].m_transform;
	}
	const btTransform& getChildTransform(int index) const
	{
		return m_children[index].m_transform;
	}

	///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
	void updateChildTransform(int childIndex, const btTransform& newChildTransform, bool shouldRecalculateLocalAabb = true)
	{
		m_children[childIndex].m_transform = newChildTransform;

		if (m_dynamicAabbTree)
		{
			///update the dynamic aabb tree
			btVector3 localAabbMin, localAabbMax;
			m_children[childIndex].m_childShape->getAabb(newChildTransform, localAabbMin, localAabbMax);
			ATTRIBUTE_ALIGNED16(btDbvtVolume)
			bounds = btDbvtVolume::FromMM(localAabbMin, localAabbMax);
			//int index = m_children.size()-1;
			m_dynamicAabbTree->update(m_children[childIndex].m_node, bounds);
		}

		if (shouldRecalculateLocalAabb)
		{
			recalculateLocalAabb();
		}
	}

	btCompoundShapeChild* getChildList()
	{
		return &m_children[0];
	}

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	virtual void getAabb(const btTransform& trans, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btVector3 localHalfExtents = btScalar(0.5) * (m_localAabbMax - m_localAabbMin);
		btVector3 localCenter = btScalar(0.5) * (m_localAabbMax + m_localAabbMin);

		//avoid an illegal AABB when there are no children
		if (!m_children.size())
		{
			localHalfExtents.setValue(0, 0, 0);
			localCenter.setValue(0, 0, 0);
		}
		localHalfExtents += btVector3(getMargin(), getMargin(), getMargin());

		btMatrix3x3 abs_b = trans.getBasis().absolute();

		btVector3 center = trans(localCenter);

		btVector3 extent = localHalfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
		aabbMin = center - extent;
		aabbMax = center + extent;
	}

	/** Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
	Use this yourself if you modify the children or their transforms. */
	virtual void recalculateLocalAabb()
	{
		// Recalculate the local aabb
		// Brute force, it iterates over all the shapes left.

		m_localAabbMin = btVector3(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
		m_localAabbMax = btVector3(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT));

		//extend the local aabbMin/aabbMax
		for (int j = 0; j < m_children.size(); j++)
		{
			btVector3 localAabbMin, localAabbMax;
			m_children[j].m_childShape->getAabb(m_children[j].m_transform, localAabbMin, localAabbMax);
			for (int i = 0; i < 3; i++)
			{
				if (m_localAabbMin[i] > localAabbMin[i])
					m_localAabbMin[i] = localAabbMin[i];
				if (m_localAabbMax[i] < localAabbMax[i])
					m_localAabbMax[i] = localAabbMax[i];
			}
		}
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		for (int i = 0; i < m_children.size(); i++)
		{
			btTransform childTrans = getChildTransform(i);
			btVector3 childScale = m_children[i].m_childShape->getLocalScaling();
			//		childScale = childScale * (childTrans.getBasis() * scaling);
			childScale = childScale * scaling / m_localScaling;
			m_children[i].m_childShape->setLocalScaling(childScale);
			childTrans.setOrigin((childTrans.getOrigin()) * scaling / m_localScaling);
			updateChildTransform(i, childTrans, false);
		}

		m_localScaling = scaling;
		recalculateLocalAabb();
	}

	virtual const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		// approximation: take the inertia from the aabb for now
		btTransform ident;
		ident.setIdentity();
		btVector3 aabbMin, aabbMax;
		getAabb(ident, aabbMin, aabbMax);

		btVector3 halfExtents = (aabbMax - aabbMin) * btScalar(0.5);

		btScalar lx = btScalar(2.) * (halfExtents.x());
		btScalar ly = btScalar(2.) * (halfExtents.y());
		btScalar lz = btScalar(2.) * (halfExtents.z());

		inertia[0] = mass / (btScalar(12.0)) * (ly * ly + lz * lz);
		inertia[1] = mass / (btScalar(12.0)) * (lx * lx + lz * lz);
		inertia[2] = mass / (btScalar(12.0)) * (lx * lx + ly * ly);
	}

	virtual void setMargin(btScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual btScalar getMargin() const
	{
		return m_collisionMargin;
	}
	virtual const char* getName() const
	{
		return "Compound";
	}

	const btDbvt* getDynamicAabbTree() const
	{
		return m_dynamicAabbTree;
	}

	btDbvt* getDynamicAabbTree()
	{
		return m_dynamicAabbTree;
	}

	void createAabbTreeFromChildren()
	{
		if (!m_dynamicAabbTree)
		{
			void* mem = btAlignedAlloc(sizeof(btDbvt), 16);
			m_dynamicAabbTree = new (mem) btDbvt();
			btAssert(mem == m_dynamicAabbTree);

			for (int index = 0; index < m_children.size(); index++)
			{
				btCompoundShapeChild& child = m_children[index];

				//extend the local aabbMin/aabbMax
				btVector3 localAabbMin, localAabbMax;
				child.m_childShape->getAabb(child.m_transform, localAabbMin, localAabbMax);

				const btDbvtVolume bounds = btDbvtVolume::FromMM(localAabbMin, localAabbMax);
				size_t index2 = index;
				child.m_node = m_dynamicAabbTree->insert(bounds, reinterpret_cast<void*>(index2));
			}
		}
	}

	///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
	///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
	///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform.
	void calculatePrincipalAxisTransform(const btScalar* masses, btTransform& principal, btVector3& inertia) const
	{
		int n = m_children.size();

		btScalar totalMass = 0;
		btVector3 center(0, 0, 0);
		int k;

		for (k = 0; k < n; k++)
		{
			btAssert(masses[k] > 0);
			center += m_children[k].m_transform.getOrigin() * masses[k];
			totalMass += masses[k];
		}

		btAssert(totalMass > 0);

		center /= totalMass;
		principal.setOrigin(center);

		btMatrix3x3 tensor(0, 0, 0, 0, 0, 0, 0, 0, 0);
		for (k = 0; k < n; k++)
		{
			btVector3 i;
			m_children[k].m_childShape->calculateLocalInertia(masses[k], i);

			const btTransform& t = m_children[k].m_transform;
			btVector3 o = t.getOrigin() - center;

			//compute inertia tensor in coordinate system of compound shape
			btMatrix3x3 j = t.getBasis().transpose();
			j[0] *= i[0];
			j[1] *= i[1];
			j[2] *= i[2];
			j = t.getBasis() * j;

			//add inertia tensor
			tensor[0] += j[0];
			tensor[1] += j[1];
			tensor[2] += j[2];

			//compute inertia tensor of pointmass at o
			btScalar o2 = o.length2();
			j[0].setValue(o2, 0, 0);
			j[1].setValue(0, o2, 0);
			j[2].setValue(0, 0, o2);
			j[0] += o * -o.x();
			j[1] += o * -o.y();
			j[2] += o * -o.z();

			//add inertia tensor of pointmass
			tensor[0] += masses[k] * j[0];
			tensor[1] += masses[k] * j[1];
			tensor[2] += masses[k] * j[2];
		}

		tensor.diagonalize(principal.getBasis(), btScalar(0.00001), 20);
		inertia.setValue(tensor[0][0], tensor[1][1], tensor[2][2]);
	}

	int getUpdateRevision() const
	{
		return m_updateRevision;
	}
};

#endif  //BT_COMPOUND_SHAPE_H
