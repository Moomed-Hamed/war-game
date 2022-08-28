#ifndef BT_MULTIBODY_SLIDER_CONSTRAINT_H
#define BT_MULTIBODY_SLIDER_CONSTRAINT_H

#include "btMultiBodyConstraint.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "LinearMath/btIDebugDraw.h"

#define BTMBSLIDERCONSTRAINT_DIM 5
#define EPSILON 0.000001

class btMultiBodySliderConstraint : public btMultiBodyConstraint
{
protected:
	btRigidBody* m_rigidBodyA;
	btRigidBody* m_rigidBodyB;
	btVector3 m_pivotInA;
	btVector3 m_pivotInB;
	btMatrix3x3 m_frameInA;
	btMatrix3x3 m_frameInB;
	btVector3 m_jointAxis;

public:
	btMultiBodySliderConstraint(btMultiBody* body, int link, btRigidBody* bodyB, const btVector3& pivotInA, const btVector3& pivotInB, const btMatrix3x3& frameInA, const btMatrix3x3& frameInB, const btVector3& jointAxis)
		: btMultiBodyConstraint(body, 0, link, -1, BTMBSLIDERCONSTRAINT_DIM, false, MULTIBODY_CONSTRAINT_SLIDER),
		  m_rigidBodyA(0),
		  m_rigidBodyB(bodyB),
		  m_pivotInA(pivotInA),
		  m_pivotInB(pivotInB),
		  m_frameInA(frameInA),
		  m_frameInB(frameInB),
		  m_jointAxis(jointAxis)
	{
		m_data.resize(BTMBSLIDERCONSTRAINT_DIM);  //at least store the applied impulses
	}
	btMultiBodySliderConstraint(btMultiBody* bodyA, int linkA, btMultiBody* bodyB, int linkB, const btVector3& pivotInA, const btVector3& pivotInB, const btMatrix3x3& frameInA, const btMatrix3x3& frameInB, const btVector3& jointAxis)
		: btMultiBodyConstraint(bodyA, bodyB, linkA, linkB, BTMBSLIDERCONSTRAINT_DIM, false, MULTIBODY_CONSTRAINT_SLIDER),
		  m_rigidBodyA(0),
		  m_rigidBodyB(0),
		  m_pivotInA(pivotInA),
		  m_pivotInB(pivotInB),
		  m_frameInA(frameInA),
		  m_frameInB(frameInB),
		  m_jointAxis(jointAxis)
	{
		m_data.resize(BTMBSLIDERCONSTRAINT_DIM);  //at least store the applied impulses
	}
	virtual ~btMultiBodySliderConstraint() {}

	virtual void finalizeMultiDof()
	{
		//not implemented yet
		btAssert(0);
	}

	virtual int getIslandIdA() const
	{
		if (m_rigidBodyA)
			return m_rigidBodyA->getIslandTag();

		if (m_bodyA)
		{
			if (m_linkA < 0)
			{
				btMultiBodyLinkCollider* col = m_bodyA->getBaseCollider();
				if (col)
					return col->getIslandTag();
			}
			else
			{
				if (m_bodyA->getLink(m_linkA).m_collider)
					return m_bodyA->getLink(m_linkA).m_collider->getIslandTag();
			}
		}
		return -1;
	}
	virtual int getIslandIdB() const
	{
		if (m_rigidBodyB)
			return m_rigidBodyB->getIslandTag();
		if (m_bodyB)
		{
			if (m_linkB < 0)
			{
				btMultiBodyLinkCollider* col = m_bodyB->getBaseCollider();
				if (col)
					return col->getIslandTag();
			}
			else
			{
				if (m_bodyB->getLink(m_linkB).m_collider)
					return m_bodyB->getLink(m_linkB).m_collider->getIslandTag();
			}
		}
		return -1;
	}

	virtual void createConstraintRows(btMultiBodyConstraintArray& constraintRows, btMultiBodyJacobianData& data, const btContactSolverInfo& infoGlobal)
	{
		// Convert local points back to world
		btVector3 pivotAworld = m_pivotInA;
		btMatrix3x3 frameAworld = m_frameInA;
		btVector3 jointAxis = m_jointAxis;
		if (m_rigidBodyA)
		{
			pivotAworld = m_rigidBodyA->getCenterOfMassTransform() * m_pivotInA;
			frameAworld = m_frameInA.transpose() * btMatrix3x3(m_rigidBodyA->getOrientation());
			jointAxis = quatRotate(m_rigidBodyA->getOrientation(), m_jointAxis);
		}
		else if (m_bodyA)
		{
			pivotAworld = m_bodyA->localPosToWorld(m_linkA, m_pivotInA);
			frameAworld = m_bodyA->localFrameToWorld(m_linkA, m_frameInA);
			jointAxis = m_bodyA->localDirToWorld(m_linkA, m_jointAxis);
		}
		btVector3 pivotBworld = m_pivotInB;
		btMatrix3x3 frameBworld = m_frameInB;
		if (m_rigidBodyB)
		{
			pivotBworld = m_rigidBodyB->getCenterOfMassTransform() * m_pivotInB;
			frameBworld = m_frameInB.transpose() * btMatrix3x3(m_rigidBodyB->getOrientation());
		}
		else if (m_bodyB)
		{
			pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotInB);
			frameBworld = m_bodyB->localFrameToWorld(m_linkB, m_frameInB);
		}

		btVector3 constraintAxis[2];
		for (int i = 0; i < 3; ++i)
		{
			constraintAxis[0] = frameAworld.getColumn(i).cross(jointAxis);
			if (constraintAxis[0].safeNorm() > EPSILON)
			{
				constraintAxis[0] = constraintAxis[0].normalized();
				constraintAxis[1] = jointAxis.cross(constraintAxis[0]);
				constraintAxis[1] = constraintAxis[1].normalized();
				break;
			}
		}

		btMatrix3x3 relRot = frameAworld.inverse() * frameBworld;
		btVector3 angleDiff;
		btGeneric6DofSpring2Constraint::matrixToEulerXYZ(relRot, angleDiff);

		int numDim = BTMBSLIDERCONSTRAINT_DIM;
		for (int i = 0; i < numDim; i++)
		{
			btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();
			constraintRow.m_orgConstraint = this;
			constraintRow.m_orgDofIndex = i;
			constraintRow.m_relpos1CrossNormal.setValue(0, 0, 0);
			constraintRow.m_contactNormal1.setValue(0, 0, 0);
			constraintRow.m_relpos2CrossNormal.setValue(0, 0, 0);
			constraintRow.m_contactNormal2.setValue(0, 0, 0);
			constraintRow.m_angularComponentA.setValue(0, 0, 0);
			constraintRow.m_angularComponentB.setValue(0, 0, 0);

			constraintRow.m_solverBodyIdA = data.m_fixedBodyId;
			constraintRow.m_solverBodyIdB = data.m_fixedBodyId;

			if (m_rigidBodyA)
			{
				constraintRow.m_solverBodyIdA = m_rigidBodyA->getCompanionId();
			}
			if (m_rigidBodyB)
			{
				constraintRow.m_solverBodyIdB = m_rigidBodyB->getCompanionId();
			}

			btVector3 constraintNormalLin(0, 0, 0);
			btVector3 constraintNormalAng(0, 0, 0);
			btScalar posError = 0.0;
			if (i < 2)
			{
				constraintNormalLin = constraintAxis[i];
				posError = (pivotAworld - pivotBworld).dot(constraintNormalLin);
				fillMultiBodyConstraint(constraintRow, data, 0, 0, constraintNormalAng,
										constraintNormalLin, pivotAworld, pivotBworld,
										posError,
										infoGlobal,
										-m_maxAppliedImpulse, m_maxAppliedImpulse);
			}
			else
			{  //i>=2
				constraintNormalAng = frameAworld.getColumn(i % 3);
				posError = angleDiff[i % 3];
				fillMultiBodyConstraint(constraintRow, data, 0, 0, constraintNormalAng,
										constraintNormalLin, pivotAworld, pivotBworld,
										posError,
										infoGlobal,
										-m_maxAppliedImpulse, m_maxAppliedImpulse, true);
			}
		}
	}

	const btVector3& getPivotInA() const
	{
		return m_pivotInA;
	}

	void setPivotInA(const btVector3& pivotInA)
	{
		m_pivotInA = pivotInA;
	}

	const btVector3& getPivotInB() const
	{
		return m_pivotInB;
	}

	virtual void setPivotInB(const btVector3& pivotInB)
	{
		m_pivotInB = pivotInB;
	}

	const btMatrix3x3& getFrameInA() const
	{
		return m_frameInA;
	}

	void setFrameInA(const btMatrix3x3& frameInA)
	{
		m_frameInA = frameInA;
	}

	const btMatrix3x3& getFrameInB() const
	{
		return m_frameInB;
	}

	virtual void setFrameInB(const btMatrix3x3& frameInB)
	{
		m_frameInB = frameInB;
	}

	const btVector3& getJointAxis() const
	{
		return m_jointAxis;
	}

	void setJointAxis(const btVector3& jointAxis)
	{
		m_jointAxis = jointAxis;
	}

	virtual void debugDraw(class btIDebugDraw* drawer)
	{
		btTransform tr;
		tr.setIdentity();

		if (m_rigidBodyA)
		{
			btVector3 pivot = m_rigidBodyA->getCenterOfMassTransform() * m_pivotInA;
			tr.setOrigin(pivot);
			drawer->drawTransform(tr, 0.1);
		}
		if (m_bodyA)
		{
			btVector3 pivotAworld = m_bodyA->localPosToWorld(m_linkA, m_pivotInA);
			tr.setOrigin(pivotAworld);
			drawer->drawTransform(tr, 0.1);
		}
		if (m_rigidBodyB)
		{
			// that ideally should draw the same frame
			btVector3 pivot = m_rigidBodyB->getCenterOfMassTransform() * m_pivotInB;
			tr.setOrigin(pivot);
			drawer->drawTransform(tr, 0.1);
		}
		if (m_bodyB)
		{
			btVector3 pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotInB);
			tr.setOrigin(pivotBworld);
			drawer->drawTransform(tr, 0.1);
		}
	}
};

#endif  //BT_MULTIBODY_SLIDER_CONSTRAINT_H
