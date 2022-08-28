#ifndef BT_SOFT_RIGID_DYNAMICS_WORLD_H
#define BT_SOFT_RIGID_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btSoftBodyHelpers.h"
#include "btDefaultSoftBodySolver.h"

typedef btAlignedObjectArray<btSoftBody*> btSoftBodyArray;

struct btSoftSingleRayCallback;

class btSoftRigidDynamicsWorld : public btDiscreteDynamicsWorld
{
	btSoftBodyArray m_softBodies;
	int m_drawFlags;
	bool m_drawNodeTree;
	bool m_drawFaceTree;
	bool m_drawClusterTree;
	btSoftBodyWorldInfo m_sbi;
	// Solver classes that encapsulate multiple soft bodies for solving
	btSoftBodySolver* m_softBodySolver;
	bool m_ownsSolver;

protected:
	virtual void predictUnconstraintMotion(btScalar timeStep)
	{
		btDiscreteDynamicsWorld::predictUnconstraintMotion(timeStep);
		{
			BT_PROFILE("predictUnconstraintMotionSoftBody");
			m_softBodySolver->predictMotion(float(timeStep));
		}
	}

	virtual void internalSingleStepSimulation(btScalar timeStep)
	{
		// Let the solver grab the soft bodies and if necessary optimize for it
		m_softBodySolver->optimize(getSoftBodyArray());

		if (!m_softBodySolver->checkInitialized())
		{
			btAssert("Solver initialization failed\n");
		}

		btDiscreteDynamicsWorld::internalSingleStepSimulation(timeStep);

		///solve soft bodies constraints
		solveSoftBodiesConstraints(timeStep);

		//self collisions
		for (int i = 0; i < m_softBodies.size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)m_softBodies[i];
			psb->defaultCollisionHandler(psb);
		}

		///update soft bodies
		m_softBodySolver->updateSoftBodies();

		// End solver-wise simulation step
		// ///////////////////////////////
	}

	void solveSoftBodiesConstraints(btScalar timeStep)
	{
		BT_PROFILE("solveSoftConstraints");

		if (m_softBodies.size())
		{
			btSoftBody::solveClusters(m_softBodies);
		}

		// Solve constraints solver-wise
		m_softBodySolver->solveConstraints(timeStep * m_softBodySolver->getTimeScale());
	}

	void serializeSoftBodies(btSerializer* serializer)
	{
		//serialize all collision objects
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			if (colObj->getInternalType() & btCollisionObject::CO_SOFT_BODY)
			{
				int len = colObj->calculateSerializeBufferSize();
				btChunk* chunk = serializer->allocate(len, 1);
				const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
				serializer->finalizeChunk(chunk, structType, BT_SOFTBODY_CODE, colObj);
			}
		}
	}

public:
	btSoftRigidDynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration, btSoftBodySolver* softBodySolver = 0)
		: btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration),
		  m_softBodySolver(softBodySolver), m_ownsSolver(false)
	{
		if (!m_softBodySolver)
		{
			void* ptr = btAlignedAlloc(sizeof(btDefaultSoftBodySolver), 16);
			m_softBodySolver = new (ptr) btDefaultSoftBodySolver();
			m_ownsSolver = true;
		}

		m_drawFlags = fDrawFlags::Std;
		m_drawNodeTree = true;
		m_drawFaceTree = false;
		m_drawClusterTree = false;
		m_sbi.m_broadphase = pairCache;
		m_sbi.m_dispatcher = dispatcher;
		m_sbi.m_sparsesdf.Initialize();
		m_sbi.m_sparsesdf.Reset();

		m_sbi.air_density = (btScalar)1.2;
		m_sbi.water_density = 0;
		m_sbi.water_offset = 0;
		m_sbi.water_normal = btVector3(0, 0, 0);
		m_sbi.m_gravity.setValue(0, -10, 0);

		m_sbi.m_sparsesdf.Initialize();
	}

	virtual ~btSoftRigidDynamicsWorld()
	{
		if (m_ownsSolver)
		{
			m_softBodySolver->~btSoftBodySolver();
			btAlignedFree(m_softBodySolver);
		}
	}

	virtual void debugDrawWorld()
	{
		btDiscreteDynamicsWorld::debugDrawWorld();

		if (getDebugDrawer())
		{
			int i;
			for (i = 0; i < this->m_softBodies.size(); i++)
			{
				btSoftBody* psb = (btSoftBody*)this->m_softBodies[i];
				if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
				{
					btSoftBodyHelpers::DrawFrame(psb, m_debugDrawer);
					btSoftBodyHelpers::Draw(psb, m_debugDrawer, m_drawFlags);
				}

				if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
				{
					if (m_drawNodeTree) btSoftBodyHelpers::DrawNodeTree(psb, m_debugDrawer);
					if (m_drawFaceTree) btSoftBodyHelpers::DrawFaceTree(psb, m_debugDrawer);
					if (m_drawClusterTree) btSoftBodyHelpers::DrawClusterTree(psb, m_debugDrawer);
				}
			}
		}
	}

	void addSoftBody(btSoftBody* body, int collisionFilterGroup = btBroadphaseProxy::DefaultFilter, int collisionFilterMask = btBroadphaseProxy::AllFilter)
	{
		m_softBodies.push_back(body);

		// Set the soft body solver that will deal with this body to be the world's solver
		body->setSoftBodySolver(m_softBodySolver);

		btCollisionWorld::addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
	}

	void removeSoftBody(btSoftBody* body)
	{
		m_softBodies.remove(body);

		btCollisionWorld::removeCollisionObject(body);
	}

	// removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btDiscreteDynamicsWorld::removeCollisionObject
	virtual void removeCollisionObject(btCollisionObject* collisionObject)
	{
		btSoftBody* body = btSoftBody::upcast(collisionObject);
		if (body)
			removeSoftBody(body);
		else
			btDiscreteDynamicsWorld::removeCollisionObject(collisionObject);
	}

	int getDrawFlags() const { return (m_drawFlags); }
	void setDrawFlags(int f) { m_drawFlags = f; }

	btSoftBodyWorldInfo& getWorldInfo()
	{
		return m_sbi;
	}
	const btSoftBodyWorldInfo& getWorldInfo() const
	{
		return m_sbi;
	}

	virtual btDynamicsWorldType getWorldType() const
	{
		return BT_SOFT_RIGID_DYNAMICS_WORLD;
	}

	btSoftBodyArray& getSoftBodyArray()
	{
		return m_softBodies;
	}

	const btSoftBodyArray& getSoftBodyArray() const
	{
		return m_softBodies;
	}

	virtual void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const;

	/// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
	/// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
	/// This allows more customization.
	static void rayTestSingle(const btTransform& rayFromTrans, const btTransform& rayToTrans,
							  btCollisionObject* collisionObject,
							  const btCollisionShape* collisionShape,
							  const btTransform& colObjWorldTransform,
							  RayResultCallback& resultCallback);

	virtual void serialize(btSerializer* serializer)
	{
		serializer->startSerialization();
		serializeDynamicsWorldInfo(serializer);
		serializeSoftBodies(serializer);
		serializeRigidBodies(serializer);
		serializeCollisionObjects(serializer);
		serializer->finishSerialization();
	}
};

struct btSoftSingleRayCallback : public btBroadphaseRayCallback
{
	btVector3 m_rayFromWorld;
	btVector3 m_rayToWorld;
	btTransform m_rayFromTrans;
	btTransform m_rayToTrans;
	btVector3 m_hitNormal;

	const btSoftRigidDynamicsWorld* m_world;
	btCollisionWorld::RayResultCallback& m_resultCallback;

	btSoftSingleRayCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld, const btSoftRigidDynamicsWorld* world, btCollisionWorld::RayResultCallback& resultCallback)
		: m_rayFromWorld(rayFromWorld), m_rayToWorld(rayToWorld), m_world(world), m_resultCallback(resultCallback)
	{
		m_rayFromTrans.setIdentity();
		m_rayFromTrans.setOrigin(m_rayFromWorld);
		m_rayToTrans.setIdentity();
		m_rayToTrans.setOrigin(m_rayToWorld);

		btVector3 rayDir = (rayToWorld - rayFromWorld);

		rayDir.normalize();
		// what about division by zero? --> just set rayDirection[i] to INF/1e30
		m_rayDirectionInverse[0] = rayDir[0] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(m_rayToWorld - m_rayFromWorld);
	}

	virtual bool process(const btBroadphaseProxy* proxy)
	{
		// terminate further ray tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == btScalar(0.f))
			return false;

		btCollisionObject* collisionObject = (btCollisionObject*)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
			m_world->rayTestSingle(m_rayFromTrans, m_rayToTrans, collisionObject, collisionObject->getCollisionShape(), collisionObject->getWorldTransform(), m_resultCallback);

		return true;
	}
};

#endif  //BT_SOFT_RIGID_DYNAMICS_WORLD_H
