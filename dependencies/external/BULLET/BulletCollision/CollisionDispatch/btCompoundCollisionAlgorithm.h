#ifndef BT_COMPOUND_COLLISION_ALGORITHM_H
#define BT_COMPOUND_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "btCollisionCreateFunc.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "btManifoldResult.h"

class btCollisionShape;
typedef bool (*btShapePairCallback)(const btCollisionShape* pShape0, const btCollisionShape* pShape1);

static btShapePairCallback gCompoundChildShapePairCallback = 0;

struct btCompoundLeafCallback : btDbvt::ICollide
{
public:
	const btCollisionObjectWrapper* m_compoundColObjWrap;
	const btCollisionObjectWrapper* m_otherObjWrap;
	btDispatcher* m_dispatcher;
	const btDispatcherInfo& m_dispatchInfo;
	btManifoldResult* m_resultOut;
	btCollisionAlgorithm** m_childCollisionAlgorithms;
	btPersistentManifold* m_sharedManifold;

	btCompoundLeafCallback(const btCollisionObjectWrapper* compoundObjWrap, const btCollisionObjectWrapper* otherObjWrap, btDispatcher* dispatcher, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut, btCollisionAlgorithm** childCollisionAlgorithms, btPersistentManifold* sharedManifold)
		: m_compoundColObjWrap(compoundObjWrap), m_otherObjWrap(otherObjWrap), m_dispatcher(dispatcher), m_dispatchInfo(dispatchInfo), m_resultOut(resultOut), m_childCollisionAlgorithms(childCollisionAlgorithms), m_sharedManifold(sharedManifold)
	{
	}

	void ProcessChildShape(const btCollisionShape* childShape, int index)
	{
		btAssert(index >= 0);
		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(m_compoundColObjWrap->getCollisionShape());
		btAssert(index < compoundShape->getNumChildShapes());

		if (gCompoundChildShapePairCallback)
		{
			if (!gCompoundChildShapePairCallback(m_otherObjWrap->getCollisionShape(), childShape))
				return;
		}

		//backup
		btTransform orgTrans = m_compoundColObjWrap->getWorldTransform();

		const btTransform& childTrans = compoundShape->getChildTransform(index);
		btTransform newChildWorldTrans = orgTrans * childTrans;

		//perform an AABB check first
		btVector3 aabbMin0, aabbMax0;
		childShape->getAabb(newChildWorldTrans, aabbMin0, aabbMax0);

		btVector3 extendAabb(m_resultOut->m_closestPointDistanceThreshold, m_resultOut->m_closestPointDistanceThreshold, m_resultOut->m_closestPointDistanceThreshold);
		aabbMin0 -= extendAabb;
		aabbMax0 += extendAabb;

		btVector3 aabbMin1, aabbMax1;
		m_otherObjWrap->getCollisionShape()->getAabb(m_otherObjWrap->getWorldTransform(), aabbMin1, aabbMax1);

		if (TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
		{
			btTransform preTransform = childTrans;
			if (this->m_compoundColObjWrap->m_preTransform)
			{
				preTransform = preTransform * (*(this->m_compoundColObjWrap->m_preTransform));
			}
			btCollisionObjectWrapper compoundWrap(this->m_compoundColObjWrap, childShape, m_compoundColObjWrap->getCollisionObject(), newChildWorldTrans, preTransform, -1, index);

			btCollisionAlgorithm* algo = 0;
			bool allocatedAlgorithm = false;

			if (m_resultOut->m_closestPointDistanceThreshold > 0)
			{
				algo = m_dispatcher->findAlgorithm(&compoundWrap, m_otherObjWrap, 0, BT_CLOSEST_POINT_ALGORITHMS);
				allocatedAlgorithm = true;
			}
			else
			{
				//the contactpoint is still projected back using the original inverted worldtrans
				if (!m_childCollisionAlgorithms[index])
				{
					m_childCollisionAlgorithms[index] = m_dispatcher->findAlgorithm(&compoundWrap, m_otherObjWrap, m_sharedManifold, BT_CONTACT_POINT_ALGORITHMS);
				}
				algo = m_childCollisionAlgorithms[index];
			}

			const btCollisionObjectWrapper* tmpWrap = 0;

			///detect swapping case
			if (m_resultOut->getBody0Internal() == m_compoundColObjWrap->getCollisionObject())
			{
				tmpWrap = m_resultOut->getBody0Wrap();
				m_resultOut->setBody0Wrap(&compoundWrap);
				m_resultOut->setShapeIdentifiersA(-1, index);
			}
			else
			{
				tmpWrap = m_resultOut->getBody1Wrap();
				m_resultOut->setBody1Wrap(&compoundWrap);
				m_resultOut->setShapeIdentifiersB(-1, index);
			}

			algo->processCollision(&compoundWrap, m_otherObjWrap, m_dispatchInfo, m_resultOut);

			if (m_resultOut->getBody0Internal() == m_compoundColObjWrap->getCollisionObject())
			{
				m_resultOut->setBody0Wrap(tmpWrap);
			}
			else
			{
				m_resultOut->setBody1Wrap(tmpWrap);
			}
			if (allocatedAlgorithm)
			{
				algo->~btCollisionAlgorithm();
				m_dispatcher->freeCollisionAlgorithm(algo);
			}
		}
	}
	void Process(const btDbvtNode* leaf)
	{
		int index = leaf->dataAsInt;

		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(m_compoundColObjWrap->getCollisionShape());
		const btCollisionShape* childShape = compoundShape->getChildShape(index);

		ProcessChildShape(childShape, index);
	}
};

/// btCompoundCollisionAlgorithm supports collision between CompoundCollisionShapes and other collision shapes
class btCompoundCollisionAlgorithm : public btActivatingCollisionAlgorithm
{
	btNodeStack stack2;
	btManifoldArray manifoldArray;

protected:
	btAlignedObjectArray<btCollisionAlgorithm*> m_childCollisionAlgorithms;
	bool m_isSwapped;

	class btPersistentManifold* m_sharedManifold;
	bool m_ownsManifold;

	int m_compoundShapeRevision;  //to keep track of changes, so that childAlgorithm array can be updated

	void removeChildAlgorithms()
	{
		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		for (i = 0; i < numChildren; i++)
		{
			if (m_childCollisionAlgorithms[i])
			{
				m_childCollisionAlgorithms[i]->~btCollisionAlgorithm();
				m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
			}
		}
	}

	void preallocateChildAlgorithms(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
	{
		const btCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
		const btCollisionObjectWrapper* otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
		btAssert(colObjWrap->getCollisionShape()->isCompound());

		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(colObjWrap->getCollisionShape());

		int numChildren = compoundShape->getNumChildShapes();
		int i;

		m_childCollisionAlgorithms.resize(numChildren);
		for (i = 0; i < numChildren; i++)
		{
			if (compoundShape->getDynamicAabbTree())
			{
				m_childCollisionAlgorithms[i] = 0;
			}
			else
			{
				const btCollisionShape* childShape = compoundShape->getChildShape(i);

				btCollisionObjectWrapper childWrap(colObjWrap, childShape, colObjWrap->getCollisionObject(), colObjWrap->getWorldTransform(), -1, i);  //wrong child trans, but unused (hopefully)
				m_childCollisionAlgorithms[i] = m_dispatcher->findAlgorithm(&childWrap, otherObjWrap, m_sharedManifold, BT_CONTACT_POINT_ALGORITHMS);

				btAlignedObjectArray<btCollisionAlgorithm*> m_childCollisionAlgorithmsContact;
				btAlignedObjectArray<btCollisionAlgorithm*> m_childCollisionAlgorithmsClosestPoints;
			}
		}
	}

public:
	btCompoundCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, bool isSwapped)
		: btActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
		  m_isSwapped(isSwapped),
		  m_sharedManifold(ci.m_manifold)
	{
		m_ownsManifold = false;

		const btCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
		btAssert(colObjWrap->getCollisionShape()->isCompound());

		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(colObjWrap->getCollisionShape());
		m_compoundShapeRevision = compoundShape->getUpdateRevision();

		preallocateChildAlgorithms(body0Wrap, body1Wrap);
	}

	virtual ~btCompoundCollisionAlgorithm()
	{
		removeChildAlgorithms();
	}

	btCollisionAlgorithm* getChildAlgorithm(int n) const
	{
		return m_childCollisionAlgorithms[n];
	}

	virtual void processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		const btCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
		const btCollisionObjectWrapper* otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;

		btAssert(colObjWrap->getCollisionShape()->isCompound());
		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(colObjWrap->getCollisionShape());

		///btCompoundShape might have changed:
		////make sure the internal child collision algorithm caches are still valid
		if (compoundShape->getUpdateRevision() != m_compoundShapeRevision)
		{
			///clear and update all
			removeChildAlgorithms();

			preallocateChildAlgorithms(body0Wrap, body1Wrap);
			m_compoundShapeRevision = compoundShape->getUpdateRevision();
		}

		if (m_childCollisionAlgorithms.size() == 0)
			return;

		const btDbvt* tree = compoundShape->getDynamicAabbTree();
		//use a dynamic aabb tree to cull potential child-overlaps
		btCompoundLeafCallback callback(colObjWrap, otherObjWrap, m_dispatcher, dispatchInfo, resultOut, &m_childCollisionAlgorithms[0], m_sharedManifold);

		///we need to refresh all contact manifolds
		///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
		///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
		{
			int i;
			manifoldArray.resize(0);
			for (i = 0; i < m_childCollisionAlgorithms.size(); i++)
			{
				if (m_childCollisionAlgorithms[i])
				{
					m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
					for (int m = 0; m < manifoldArray.size(); m++)
					{
						if (manifoldArray[m]->getNumContacts())
						{
							resultOut->setPersistentManifold(manifoldArray[m]);
							resultOut->refreshContactPoints();
							resultOut->setPersistentManifold(0);  //??necessary?
						}
					}
					manifoldArray.resize(0);
				}
			}
		}

		if (tree)
		{
			btVector3 localAabbMin, localAabbMax;
			btTransform otherInCompoundSpace;
			otherInCompoundSpace = colObjWrap->getWorldTransform().inverse() * otherObjWrap->getWorldTransform();
			otherObjWrap->getCollisionShape()->getAabb(otherInCompoundSpace, localAabbMin, localAabbMax);
			btVector3 extraExtends(resultOut->m_closestPointDistanceThreshold, resultOut->m_closestPointDistanceThreshold, resultOut->m_closestPointDistanceThreshold);
			localAabbMin -= extraExtends;
			localAabbMax += extraExtends;

			const ATTRIBUTE_ALIGNED16(btDbvtVolume) bounds = btDbvtVolume::FromMM(localAabbMin, localAabbMax);
			//process all children, that overlap with  the given AABB bounds
			tree->collideTVNoStackAlloc(tree->m_root, bounds, stack2, callback);
		}
		else
		{
			//iterate over all children, perform an AABB check inside ProcessChildShape
			int numChildren = m_childCollisionAlgorithms.size();
			int i;
			for (i = 0; i < numChildren; i++)
			{
				callback.ProcessChildShape(compoundShape->getChildShape(i), i);
			}
		}

		{
			//iterate over all children, perform an AABB check inside ProcessChildShape
			int numChildren = m_childCollisionAlgorithms.size();
			int i;
			manifoldArray.resize(0);
			const btCollisionShape* childShape = 0;
			btTransform orgTrans;

			btTransform newChildWorldTrans;
			btVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;

			for (i = 0; i < numChildren; i++)
			{
				if (m_childCollisionAlgorithms[i])
				{
					childShape = compoundShape->getChildShape(i);
					//if not longer overlapping, remove the algorithm
					orgTrans = colObjWrap->getWorldTransform();

					const btTransform& childTrans = compoundShape->getChildTransform(i);
					newChildWorldTrans = orgTrans * childTrans;

					//perform an AABB check first
					childShape->getAabb(newChildWorldTrans, aabbMin0, aabbMax0);
					otherObjWrap->getCollisionShape()->getAabb(otherObjWrap->getWorldTransform(), aabbMin1, aabbMax1);

					if (!TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
					{
						m_childCollisionAlgorithms[i]->~btCollisionAlgorithm();
						m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
						m_childCollisionAlgorithms[i] = 0;
					}
				}
			}
		}
	}

	btScalar calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		btAssert(0);
		//needs to be fixed, using btCollisionObjectWrapper and NOT modifying internal data structures
		btCollisionObject* colObj = m_isSwapped ? body1 : body0;
		btCollisionObject* otherObj = m_isSwapped ? body0 : body1;

		btAssert(colObj->getCollisionShape()->isCompound());

		btCompoundShape* compoundShape = static_cast<btCompoundShape*>(colObj->getCollisionShape());

		//We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
		//If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
		//given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
		//determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
		//then use each overlapping node AABB against Tree0 & vise versa.

		btScalar hitFraction = btScalar(1.);

		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		btTransform orgTrans;
		btScalar frac;
		for (i = 0; i < numChildren; i++)
		{
			//btCollisionShape* childShape = compoundShape->getChildShape(i);

			//backup
			orgTrans = colObj->getWorldTransform();

			const btTransform& childTrans = compoundShape->getChildTransform(i);
			//btTransform	newChildWorldTrans = orgTrans*childTrans ;
			colObj->setWorldTransform(orgTrans * childTrans);

			//btCollisionShape* tmpShape = colObj->getCollisionShape();
			//colObj->internalSetTemporaryCollisionShape( childShape );
			frac = m_childCollisionAlgorithms[i]->calculateTimeOfImpact(colObj, otherObj, dispatchInfo, resultOut);
			if (frac < hitFraction)
			{
				hitFraction = frac;
			}
			//revert back
			//colObj->internalSetTemporaryCollisionShape( tmpShape);
			colObj->setWorldTransform(orgTrans);
		}
		return hitFraction;
	}

	virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
	{
		int i;
		for (i = 0; i < m_childCollisionAlgorithms.size(); i++)
		{
			if (m_childCollisionAlgorithms[i])
				m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
		}
	}

	struct CreateFunc : public btCollisionAlgorithmCreateFunc
	{
		virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btCompoundCollisionAlgorithm));
			return new (mem) btCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
		}
	};

	struct SwappedCreateFunc : public btCollisionAlgorithmCreateFunc
	{
		virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btCompoundCollisionAlgorithm));
			return new (mem) btCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
		}
	};
};

#endif  //BT_COMPOUND_COLLISION_ALGORITHM_H
