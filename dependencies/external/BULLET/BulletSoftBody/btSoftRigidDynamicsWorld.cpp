#include "btSoftRigidDynamicsWorld.h"

// note (Mohamed) : there's a struct btSoftSingleRayCallback; that is causing issues
// if you're getting errors, that's probably why. It's because btSoftRigidDynamicsWorld
// uses it and it uses btSoftRigidDynamicsWorld, so there's a circular dependency

void btSoftRigidDynamicsWorld::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	BT_PROFILE("rayTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	btSoftSingleRayCallback rayCB(rayFromWorld, rayToWorld, this, resultCallback);
	m_broadphasePairCache->rayTest(rayFromWorld, rayToWorld, rayCB);
}
void btSoftRigidDynamicsWorld::rayTestSingle(const btTransform& rayFromTrans, const btTransform& rayToTrans, btCollisionObject* collisionObject, const btCollisionShape* collisionShape, const btTransform& colObjWorldTransform, RayResultCallback& resultCallback)
{
	if (collisionShape->isSoftBody())
	{
		btSoftBody* softBody = btSoftBody::upcast(collisionObject);
		if (softBody)
		{
			btSoftBody::sRayCast softResult;
			if (softBody->rayTest(rayFromTrans.getOrigin(), rayToTrans.getOrigin(), softResult))
			{
				if (softResult.fraction <= resultCallback.m_closestHitFraction)
				{
					btCollisionWorld::LocalShapeInfo shapeInfo;
					shapeInfo.m_shapePart = 0;
					shapeInfo.m_triangleIndex = softResult.index;
					// get the normal
					btVector3 rayDir = rayToTrans.getOrigin() - rayFromTrans.getOrigin();
					btVector3 normal = -rayDir;
					normal.normalize();

					if (softResult.feature == btSoftBody::eFeature::Face)
					{
						normal = softBody->m_faces[softResult.index].m_normal;
						if (normal.dot(rayDir) > 0)
						{
							normal = -normal; // normal always point toward origin of the ray
						}
					}

					btCollisionWorld::LocalRayResult rayResult(collisionObject, &shapeInfo, normal, softResult.fraction);
					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(rayResult, normalInWorldSpace);
				}
			}
		}
	}
	else
	{
		btCollisionWorld::rayTestSingle(rayFromTrans, rayToTrans, collisionObject, collisionShape, colObjWorldTransform, resultCallback);
	}
}
