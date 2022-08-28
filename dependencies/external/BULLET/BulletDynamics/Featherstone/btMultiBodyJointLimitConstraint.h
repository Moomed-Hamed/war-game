#ifndef BT_MULTIBODY_JOINT_LIMIT_CONSTRAINT_H
#define BT_MULTIBODY_JOINT_LIMIT_CONSTRAINT_H

#include "btMultiBodyConstraint.h"
#include "btMultiBodyLinkCollider.h"
struct btSolverInfo;

class btMultiBodyJointLimitConstraint : public btMultiBodyConstraint
{
protected:
	btScalar m_lowerBound;
	btScalar m_upperBound;

public:
	btMultiBodyJointLimitConstraint(btMultiBody* body, int link, btScalar lower, btScalar upper)
		: btMultiBodyConstraint(body, body, link, body->getLink(link).m_parent, 2, true, MULTIBODY_CONSTRAINT_LIMIT),
		  m_lowerBound(lower),
		  m_upperBound(upper)
	{
	}
	virtual ~btMultiBodyJointLimitConstraint() {}

	virtual void finalizeMultiDof()
	{
		// the data.m_jacobians never change, so may as well
		// initialize them here

		allocateJacobiansMultiDof();

		unsigned int offset = 6 + m_bodyA->getLink(m_linkA).m_dofOffset;

		// row 0: the lower bound
		jacobianA(0)[offset] = 1;
		// row 1: the upper bound
		//jacobianA(1)[offset] = -1;
		jacobianB(1)[offset] = -1;

		m_numDofsFinalized = m_jacSizeBoth;
	}

	virtual int getIslandIdA() const
	{
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
		// only positions need to be updated -- data.m_jacobians and force
		// directions were set in the ctor and never change.

		if (m_numDofsFinalized != m_jacSizeBoth)
		{
			finalizeMultiDof();
		}

		// row 0: the lower bound
		setPosition(0, m_bodyA->getJointPos(m_linkA) - m_lowerBound);  //multidof: this is joint-type dependent

		// row 1: the upper bound
		setPosition(1, m_upperBound - m_bodyA->getJointPos(m_linkA));

		for (int row = 0; row < getNumRows(); row++)
		{
			btScalar penetration = getPosition(row);

			//todo: consider adding some safety threshold here
			if (penetration > 0)
			{
				continue;
			}
			btScalar direction = row ? -1 : 1;

			btMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();
			constraintRow.m_orgConstraint = this;
			constraintRow.m_orgDofIndex = row;

			constraintRow.m_multiBodyA = m_bodyA;
			constraintRow.m_multiBodyB = m_bodyB;
			const btScalar posError = 0;  //why assume it's zero?
			const btVector3 dummy(0, 0, 0);

			btScalar rel_vel = fillMultiBodyConstraint(constraintRow, data, jacobianA(row), jacobianB(row), dummy, dummy, dummy, dummy, posError, infoGlobal, 0, m_maxAppliedImpulse);

			{
				//expect either prismatic or revolute joint type for now
				btAssert((m_bodyA->getLink(m_linkA).m_jointType == btMultibodyLink::eRevolute) || (m_bodyA->getLink(m_linkA).m_jointType == btMultibodyLink::ePrismatic));
				switch (m_bodyA->getLink(m_linkA).m_jointType)
				{
					case btMultibodyLink::eRevolute:
					{
						constraintRow.m_contactNormal1.setZero();
						constraintRow.m_contactNormal2.setZero();
						btVector3 revoluteAxisInWorld = direction * quatRotate(m_bodyA->getLink(m_linkA).m_cachedWorldTransform.getRotation(), m_bodyA->getLink(m_linkA).m_axes[0].m_topVec);
						constraintRow.m_relpos1CrossNormal = revoluteAxisInWorld;
						constraintRow.m_relpos2CrossNormal = -revoluteAxisInWorld;

						break;
					}
					case btMultibodyLink::ePrismatic:
					{
						btVector3 prismaticAxisInWorld = direction * quatRotate(m_bodyA->getLink(m_linkA).m_cachedWorldTransform.getRotation(), m_bodyA->getLink(m_linkA).m_axes[0].m_bottomVec);
						constraintRow.m_contactNormal1 = prismaticAxisInWorld;
						constraintRow.m_contactNormal2 = -prismaticAxisInWorld;
						constraintRow.m_relpos1CrossNormal.setZero();
						constraintRow.m_relpos2CrossNormal.setZero();

						break;
					}
					default:
					{
						btAssert(0);
					}
				};
			}

			{
				btScalar positionalError = 0.f;
				btScalar velocityError = -rel_vel;  // * damping;
				btScalar erp = infoGlobal.m_erp2;
				if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
				{
					erp = infoGlobal.m_erp;
				}
				if (penetration > 0)
				{
					positionalError = 0;
					velocityError = -penetration / infoGlobal.m_timeStep;
				}
				else
				{
					positionalError = -penetration * erp / infoGlobal.m_timeStep;
				}

				btScalar penetrationImpulse = positionalError * constraintRow.m_jacDiagABInv;
				btScalar velocityImpulse = velocityError * constraintRow.m_jacDiagABInv;
				if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
				{
					//combine position and velocity into rhs
					constraintRow.m_rhs = penetrationImpulse + velocityImpulse;
					constraintRow.m_rhsPenetration = 0.f;
				}
				else
				{
					//split position and velocity into rhs and m_rhsPenetration
					constraintRow.m_rhs = velocityImpulse;
					constraintRow.m_rhsPenetration = penetrationImpulse;
				}
			}
		}
	}

	virtual void debugDraw(class btIDebugDraw* drawer) {}
	btScalar getLowerBound() const
	{
		return m_lowerBound;
	}
	btScalar getUpperBound() const
	{
		return m_upperBound;
	}
	void setLowerBound(btScalar lower)
	{
		m_lowerBound = lower;
	}
	void setUpperBound(btScalar upper)
	{
		m_upperBound = upper;
	}
};

#endif  //BT_MULTIBODY_JOINT_LIMIT_CONSTRAINT_H
