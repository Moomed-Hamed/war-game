#ifndef BT_SOFT_BODY_DEFAULT_SOLVER_H
#define BT_SOFT_BODY_DEFAULT_SOLVER_H

#include "BulletSoftBody/btSoftBodySolvers.h"
#include "btSoftBodySolverVertexBuffer.h"
#include "BulletSoftBody/btSoftBody.h"
struct btCollisionObjectWrapper;

class btDefaultSoftBodySolver : public btSoftBodySolver
{
protected:
	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	btAlignedObjectArray<btSoftBody *> m_softBodySet;

public:
	btDefaultSoftBodySolver()
	{
		// Initial we will clearly need to update solver constants
		// For now this is global for the cloths linked with this solver - we should probably make this body specific
		// for performance in future once we understand more clearly when constants need to be updated
		m_updateSolverConstants = true;
	}

	virtual ~btDefaultSoftBodySolver(){};

	virtual SolverTypes getSolverType() const
	{
		return DEFAULT_SOLVER;
	}

	virtual bool checkInitialized() { return true; };

	virtual void updateSoftBodies()
	{
		for (int i = 0; i < m_softBodySet.size(); i++)
		{
			btSoftBody *psb = (btSoftBody *)m_softBodySet[i];
			if (psb->isActive())
			{
				psb->integrateMotion();
			}
		}
	}

	virtual void optimize(btAlignedObjectArray<btSoftBody *> &softBodies, bool forceUpdate = false)
	{
		m_softBodySet.copyFromArray(softBodies);
	}

	// In this case the data is already in the soft bodies so there is no need for us to do anything
	virtual void copyBackToSoftBodies(bool bMove = true) {}

	virtual void solveConstraints(btScalar solverdt)
	{
		// Solve constraints for non-solver softbodies
		for (int i = 0; i < m_softBodySet.size(); ++i)
		{
			btSoftBody *psb = static_cast<btSoftBody *>(m_softBodySet[i]);
			if (psb->isActive())
			{
				psb->solveConstraints();
			}
		}
	}

	virtual void predictMotion(btScalar timeStep)
	{
		for (int i = 0; i < m_softBodySet.size(); ++i)
		{
			btSoftBody *psb = m_softBodySet[i];

			if (psb->isActive())
			{
				psb->predictMotion(timeStep);
			}
		}
	}


	virtual void copySoftBodyToVertexBuffer(const btSoftBody *const softBody, btVertexBufferDescriptor *vertexBuffer)
	{
		// Currently only support CPU output buffers
		// TODO: check for DX11 buffers. Take all offsets into the same DX11 buffer
		// and use them together on a single kernel call if possible by setting up a
		// per-cloth target buffer array for the copy kernel.

		if (vertexBuffer->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER)
		{
			const btAlignedObjectArray<btSoftBody::Node> &clothVertices(softBody->m_nodes);
			int numVertices = clothVertices.size();

			const btCPUVertexBufferDescriptor *cpuVertexBuffer = static_cast<btCPUVertexBufferDescriptor *>(vertexBuffer);
			float *basePointer = cpuVertexBuffer->getBasePointer();

			if (vertexBuffer->hasVertexPositions())
			{
				const int vertexOffset = cpuVertexBuffer->getVertexOffset();
				const int vertexStride = cpuVertexBuffer->getVertexStride();
				float *vertexPointer = basePointer + vertexOffset;

				for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
				{
					btVector3 position = clothVertices[vertexIndex].m_x;
					*(vertexPointer + 0) = (float)position.getX();
					*(vertexPointer + 1) = (float)position.getY();
					*(vertexPointer + 2) = (float)position.getZ();
					vertexPointer += vertexStride;
				}
			}
			if (vertexBuffer->hasNormals())
			{
				const int normalOffset = cpuVertexBuffer->getNormalOffset();
				const int normalStride = cpuVertexBuffer->getNormalStride();
				float *normalPointer = basePointer + normalOffset;

				for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
				{
					btVector3 normal = clothVertices[vertexIndex].m_n;
					*(normalPointer + 0) = (float)normal.getX();
					*(normalPointer + 1) = (float)normal.getY();
					*(normalPointer + 2) = (float)normal.getZ();
					normalPointer += normalStride;
				}
			}
		}
	}

	// For the default solver just leave the soft body to do its collision processing
	virtual void processCollision(btSoftBody *softBody, const btCollisionObjectWrapper *collisionObjectWrap)
	{
		softBody->defaultCollisionHandler(collisionObjectWrap);
	}

	virtual void processCollision(btSoftBody *softBody, btSoftBody *otherSoftBody)
	{
		softBody->defaultCollisionHandler(otherSoftBody);
	}
};

#endif  // #ifndef BT_ACCELERATED_SOFT_BODY_CPU_SOLVER_H
