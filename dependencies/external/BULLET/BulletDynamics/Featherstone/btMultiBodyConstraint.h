#ifndef BT_MULTIBODY_CONSTRAINT_H
#define BT_MULTIBODY_CONSTRAINT_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btMultiBody.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"

//Don't change any of the existing enum values, so add enum types at the end for serialization compatibility
enum btTypedMultiBodyConstraintType
{
	MULTIBODY_CONSTRAINT_LIMIT=3,
	MULTIBODY_CONSTRAINT_1DOF_JOINT_MOTOR,
	MULTIBODY_CONSTRAINT_GEAR,
	MULTIBODY_CONSTRAINT_POINT_TO_POINT,
	MULTIBODY_CONSTRAINT_SLIDER,
	MULTIBODY_CONSTRAINT_SPHERICAL_MOTOR,
	MULTIBODY_CONSTRAINT_FIXED,
	MULTIBODY_CONSTRAINT_SPHERICAL_LIMIT,
	MAX_MULTIBODY_CONSTRAINT_TYPE,
};

class btMultiBody;
struct btSolverInfo;

#include "btMultiBodySolverConstraint.h"

struct btMultiBodyJacobianData
{
	btAlignedObjectArray<btScalar> m_jacobians;
	btAlignedObjectArray<btScalar> m_deltaVelocitiesUnitImpulse;  //holds the joint-space response of the corresp. tree to the test impulse in each constraint space dimension
	btAlignedObjectArray<btScalar> m_deltaVelocities;             //holds joint-space vectors of all the constrained trees accumulating the effect of corrective impulses applied in SI
	btAlignedObjectArray<btScalar> scratch_r;
	btAlignedObjectArray<btVector3> scratch_v;
	btAlignedObjectArray<btMatrix3x3> scratch_m;
	btAlignedObjectArray<btSolverBody>* m_solverBodyPool;
	int m_fixedBodyId;
};

ATTRIBUTE_ALIGNED16(class)
btMultiBodyConstraint
{
protected:
	btMultiBody* m_bodyA;
	btMultiBody* m_bodyB;
	int m_linkA;
	int m_linkB;

	int m_type; //btTypedMultiBodyConstraintType

	int m_numRows;
	int m_jacSizeA;
	int m_jacSizeBoth;
	int m_posOffset;

	bool m_isUnilateral;
	int m_numDofsFinalized;
	btScalar m_maxAppliedImpulse;

	// warning: the data block lay out is not consistent for all constraints
	// data block laid out as follows:
	// cached impulses. (one per row.)
	// jacobians. (interleaved, row1 body1 then row1 body2 then row2 body 1 etc)
	// positions. (one per row.)
	btAlignedObjectArray<btScalar> m_data;

	void applyDeltaVee(btMultiBodyJacobianData & data, btScalar * delta_vee, btScalar impulse, int velocityIndex, int ndof)
	{
		for (int i = 0; i < ndof; ++i)
			data.m_deltaVelocities[velocityIndex + i] += delta_vee[i] * impulse;
	}

	btScalar fillMultiBodyConstraint(btMultiBodySolverConstraint & solverConstraint,
									 btMultiBodyJacobianData & data,
									 btScalar * jacOrgA, btScalar * jacOrgB,
									 const btVector3& constraintNormalAng,

									 const btVector3& constraintNormalLin,
									 const btVector3& posAworld, const btVector3& posBworld,
									 btScalar posError,
									 const btContactSolverInfo& infoGlobal,
									 btScalar lowerLimit, btScalar upperLimit,
									 bool angConstraint = false,

									 btScalar relaxation = 1.f,
									 bool isFriction = false, btScalar desiredVelocity = 0, btScalar cfmSlip = 0, btScalar damping = 1.0)
	{
		solverConstraint.m_multiBodyA = m_bodyA;
		solverConstraint.m_multiBodyB = m_bodyB;
		solverConstraint.m_linkA = m_linkA;
		solverConstraint.m_linkB = m_linkB;

		btMultiBody* multiBodyA = solverConstraint.m_multiBodyA;
		btMultiBody* multiBodyB = solverConstraint.m_multiBodyB;

		btSolverBody* bodyA = multiBodyA ? 0 : &data.m_solverBodyPool->at(solverConstraint.m_solverBodyIdA);
		btSolverBody* bodyB = multiBodyB ? 0 : &data.m_solverBodyPool->at(solverConstraint.m_solverBodyIdB);

		btRigidBody* rb0 = multiBodyA ? 0 : bodyA->m_originalBody;
		btRigidBody* rb1 = multiBodyB ? 0 : bodyB->m_originalBody;

		btVector3 rel_pos1, rel_pos2;  //these two used to be inited to posAworld and posBworld (respectively) but it does not seem necessary
		if (bodyA)
			rel_pos1 = posAworld - bodyA->getWorldTransform().getOrigin();
		if (bodyB)
			rel_pos2 = posBworld - bodyB->getWorldTransform().getOrigin();

		if (multiBodyA)
		{
			if (solverConstraint.m_linkA < 0)
			{
				rel_pos1 = posAworld - multiBodyA->getBasePos();
			}
			else
			{
				rel_pos1 = posAworld - multiBodyA->getLink(solverConstraint.m_linkA).m_cachedWorldTransform.getOrigin();
			}

			const int ndofA = multiBodyA->getNumDofs() + 6;

			solverConstraint.m_deltaVelAindex = multiBodyA->getCompanionId();

			if (solverConstraint.m_deltaVelAindex < 0)
			{
				solverConstraint.m_deltaVelAindex = data.m_deltaVelocities.size();
				multiBodyA->setCompanionId(solverConstraint.m_deltaVelAindex);
				data.m_deltaVelocities.resize(data.m_deltaVelocities.size() + ndofA);
			}
			else
			{
				btAssert(data.m_deltaVelocities.size() >= solverConstraint.m_deltaVelAindex + ndofA);
			}

			//determine jacobian of this 1D constraint in terms of multibodyA's degrees of freedom
			//resize..
			solverConstraint.m_jacAindex = data.m_jacobians.size();
			data.m_jacobians.resize(data.m_jacobians.size() + ndofA);
			//copy/determine
			if (jacOrgA)
			{
				for (int i = 0; i < ndofA; i++)
					data.m_jacobians[solverConstraint.m_jacAindex + i] = jacOrgA[i];
			}
			else
			{
				btScalar* jac1 = &data.m_jacobians[solverConstraint.m_jacAindex];
				//multiBodyA->fillContactJacobianMultiDof(solverConstraint.m_linkA, posAworld, constraintNormalLin, jac1, data.scratch_r, data.scratch_v, data.scratch_m);
				multiBodyA->fillConstraintJacobianMultiDof(solverConstraint.m_linkA, posAworld, constraintNormalAng, constraintNormalLin, jac1, data.scratch_r, data.scratch_v, data.scratch_m);
			}

			//determine the velocity response of multibodyA to reaction impulses of this constraint (i.e. A[i,i] for i=1,...n_con: multibody's inverse inertia with respect to this 1D constraint)
			//resize..
			data.m_deltaVelocitiesUnitImpulse.resize(data.m_deltaVelocitiesUnitImpulse.size() + ndofA);  //=> each constraint row has the constrained tree dofs allocated in m_deltaVelocitiesUnitImpulse
			btAssert(data.m_jacobians.size() == data.m_deltaVelocitiesUnitImpulse.size());
			btScalar* delta = &data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacAindex];
			//determine..
			multiBodyA->calcAccelerationDeltasMultiDof(&data.m_jacobians[solverConstraint.m_jacAindex], delta, data.scratch_r, data.scratch_v);

			btVector3 torqueAxis0;
			if (angConstraint)
			{
				torqueAxis0 = constraintNormalAng;
			}
			else
			{
				torqueAxis0 = rel_pos1.cross(constraintNormalLin);
			}
			solverConstraint.m_relpos1CrossNormal = torqueAxis0;
			solverConstraint.m_contactNormal1 = constraintNormalLin;
		}
		else  //if(rb0)
		{
			btVector3 torqueAxis0;
			if (angConstraint)
			{
				torqueAxis0 = constraintNormalAng;
			}
			else
			{
				torqueAxis0 = rel_pos1.cross(constraintNormalLin);
			}
			solverConstraint.m_angularComponentA = rb0 ? rb0->getInvInertiaTensorWorld() * torqueAxis0 * rb0->getAngularFactor() : btVector3(0, 0, 0);
			solverConstraint.m_relpos1CrossNormal = torqueAxis0;
			solverConstraint.m_contactNormal1 = constraintNormalLin;
		}

		if (multiBodyB)
		{
			if (solverConstraint.m_linkB < 0)
			{
				rel_pos2 = posBworld - multiBodyB->getBasePos();
			}
			else
			{
				rel_pos2 = posBworld - multiBodyB->getLink(solverConstraint.m_linkB).m_cachedWorldTransform.getOrigin();
			}

			const int ndofB = multiBodyB->getNumDofs() + 6;

			solverConstraint.m_deltaVelBindex = multiBodyB->getCompanionId();
			if (solverConstraint.m_deltaVelBindex < 0)
			{
				solverConstraint.m_deltaVelBindex = data.m_deltaVelocities.size();
				multiBodyB->setCompanionId(solverConstraint.m_deltaVelBindex);
				data.m_deltaVelocities.resize(data.m_deltaVelocities.size() + ndofB);
			}

			//determine jacobian of this 1D constraint in terms of multibodyB's degrees of freedom
			//resize..
			solverConstraint.m_jacBindex = data.m_jacobians.size();
			data.m_jacobians.resize(data.m_jacobians.size() + ndofB);
			//copy/determine..
			if (jacOrgB)
			{
				for (int i = 0; i < ndofB; i++)
					data.m_jacobians[solverConstraint.m_jacBindex + i] = jacOrgB[i];
			}
			else
			{
				//multiBodyB->fillContactJacobianMultiDof(solverConstraint.m_linkB, posBworld, -constraintNormalLin, &data.m_jacobians[solverConstraint.m_jacBindex], data.scratch_r, data.scratch_v, data.scratch_m);
				multiBodyB->fillConstraintJacobianMultiDof(solverConstraint.m_linkB, posBworld, -constraintNormalAng, -constraintNormalLin, &data.m_jacobians[solverConstraint.m_jacBindex], data.scratch_r, data.scratch_v, data.scratch_m);
			}

			//determine velocity response of multibodyB to reaction impulses of this constraint (i.e. A[i,i] for i=1,...n_con: multibody's inverse inertia with respect to this 1D constraint)
			//resize..
			data.m_deltaVelocitiesUnitImpulse.resize(data.m_deltaVelocitiesUnitImpulse.size() + ndofB);
			btAssert(data.m_jacobians.size() == data.m_deltaVelocitiesUnitImpulse.size());
			btScalar* delta = &data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacBindex];
			//determine..
			multiBodyB->calcAccelerationDeltasMultiDof(&data.m_jacobians[solverConstraint.m_jacBindex], delta, data.scratch_r, data.scratch_v);

			btVector3 torqueAxis1;
			if (angConstraint)
			{
				torqueAxis1 = constraintNormalAng;
			}
			else
			{
				torqueAxis1 = rel_pos2.cross(constraintNormalLin);
			}
			solverConstraint.m_relpos2CrossNormal = -torqueAxis1;
			solverConstraint.m_contactNormal2 = -constraintNormalLin;
		}
		else  //if(rb1)
		{
			btVector3 torqueAxis1;
			if (angConstraint)
			{
				torqueAxis1 = constraintNormalAng;
			}
			else
			{
				torqueAxis1 = rel_pos2.cross(constraintNormalLin);
			}
			solverConstraint.m_angularComponentB = rb1 ? rb1->getInvInertiaTensorWorld() * -torqueAxis1 * rb1->getAngularFactor() : btVector3(0, 0, 0);
			solverConstraint.m_relpos2CrossNormal = -torqueAxis1;
			solverConstraint.m_contactNormal2 = -constraintNormalLin;
		}
		{
			btVector3 vec;
			btScalar denom0 = 0.f;
			btScalar denom1 = 0.f;
			btScalar* jacB = 0;
			btScalar* jacA = 0;
			btScalar* deltaVelA = 0;
			btScalar* deltaVelB = 0;
			int ndofA = 0;
			//determine the "effective mass" of the constrained multibodyA with respect to this 1D constraint (i.e. 1/A[i,i])
			if (multiBodyA)
			{
				ndofA = multiBodyA->getNumDofs() + 6;
				jacA = &data.m_jacobians[solverConstraint.m_jacAindex];
				deltaVelA = &data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacAindex];
				for (int i = 0; i < ndofA; ++i)
				{
					btScalar j = jacA[i];
					btScalar l = deltaVelA[i];
					denom0 += j * l;
				}
			}
			else if (rb0)
			{
				vec = (solverConstraint.m_angularComponentA).cross(rel_pos1);
				if (angConstraint)
				{
					denom0 = constraintNormalAng.dot(solverConstraint.m_angularComponentA);
				}
				else
				{
					denom0 = rb0->getInvMass() + constraintNormalLin.dot(vec);
				}
			}
			//
			if (multiBodyB)
			{
				const int ndofB = multiBodyB->getNumDofs() + 6;
				jacB = &data.m_jacobians[solverConstraint.m_jacBindex];
				deltaVelB = &data.m_deltaVelocitiesUnitImpulse[solverConstraint.m_jacBindex];
				for (int i = 0; i < ndofB; ++i)
				{
					btScalar j = jacB[i];
					btScalar l = deltaVelB[i];
					denom1 += j * l;
				}
			}
			else if (rb1)
			{
				vec = (-solverConstraint.m_angularComponentB).cross(rel_pos2);
				if (angConstraint)
				{
					denom1 = constraintNormalAng.dot(-solverConstraint.m_angularComponentB);
				}
				else
				{
					denom1 = rb1->getInvMass() + constraintNormalLin.dot(vec);
				}
			}

			//
			btScalar d = denom0 + denom1;
			if (d > SIMD_EPSILON)
			{
				solverConstraint.m_jacDiagABInv = relaxation / (d);
			}
			else
			{
				//disable the constraint row to handle singularity/redundant constraint
				solverConstraint.m_jacDiagABInv = 0.f;
			}
		}

		//compute rhs and remaining solverConstraint fields
		btScalar penetration = isFriction ? 0 : posError;

		btScalar rel_vel = 0.f;
		int ndofA = 0;
		int ndofB = 0;
		{
			btVector3 vel1, vel2;
			if (multiBodyA)
			{
				ndofA = multiBodyA->getNumDofs() + 6;
				btScalar* jacA = &data.m_jacobians[solverConstraint.m_jacAindex];
				for (int i = 0; i < ndofA; ++i)
					rel_vel += multiBodyA->getVelocityVector()[i] * jacA[i];
			}
			else if (rb0)
			{
				rel_vel += rb0->getLinearVelocity().dot(solverConstraint.m_contactNormal1);
				rel_vel += rb0->getAngularVelocity().dot(solverConstraint.m_relpos1CrossNormal);
			}
			if (multiBodyB)
			{
				ndofB = multiBodyB->getNumDofs() + 6;
				btScalar* jacB = &data.m_jacobians[solverConstraint.m_jacBindex];
				for (int i = 0; i < ndofB; ++i)
					rel_vel += multiBodyB->getVelocityVector()[i] * jacB[i];
			}
			else if (rb1)
			{
				rel_vel += rb1->getLinearVelocity().dot(solverConstraint.m_contactNormal2);
				rel_vel += rb1->getAngularVelocity().dot(solverConstraint.m_relpos2CrossNormal);
			}

			solverConstraint.m_friction = 0.f;  //cp.m_combinedFriction;
		}

		solverConstraint.m_appliedImpulse = 0.f;
		solverConstraint.m_appliedPushImpulse = 0.f;

		{
			btScalar positionalError = 0.f;
			btScalar velocityError = (desiredVelocity - rel_vel) * damping;

			btScalar erp = infoGlobal.m_erp2;

			//split impulse is not implemented yet for btMultiBody*
			//if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
			{
				erp = infoGlobal.m_erp;
			}

			positionalError = -penetration * erp / infoGlobal.m_timeStep;

			btScalar penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
			btScalar velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;

			//split impulse is not implemented yet for btMultiBody*

			//  if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
			{
				//combine position and velocity into rhs
				solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
				solverConstraint.m_rhsPenetration = 0.f;
			}
			/*else
        {
            //split position and velocity into rhs and m_rhsPenetration
            solverConstraint.m_rhs = velocityImpulse;
            solverConstraint.m_rhsPenetration = penetrationImpulse;
        }
        */

			solverConstraint.m_cfm = 0.f;
			solverConstraint.m_lowerLimit = lowerLimit;
			solverConstraint.m_upperLimit = upperLimit;
		}

		return rel_vel;
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btMultiBodyConstraint(btMultiBody * bodyA, btMultiBody * bodyB, int linkA, int linkB, int numRows, bool isUnilateral, int type)
		: m_bodyA(bodyA),
		  m_bodyB(bodyB),
		  m_linkA(linkA),
		  m_linkB(linkB),
		  m_type(type),
		  m_numRows(numRows),
		  m_jacSizeA(0),
		  m_jacSizeBoth(0),
		  m_isUnilateral(isUnilateral),
		  m_numDofsFinalized(-1),
		  m_maxAppliedImpulse(100)
	{
	}
	virtual ~btMultiBodyConstraint() {}

	void updateJacobianSizes()
	{
		if (m_bodyA)
		{
			m_jacSizeA = (6 + m_bodyA->getNumDofs());
		}

		if (m_bodyB)
		{
			m_jacSizeBoth = m_jacSizeA + 6 + m_bodyB->getNumDofs();
		}
		else
			m_jacSizeBoth = m_jacSizeA;
	}
	void allocateJacobiansMultiDof()
	{
		updateJacobianSizes();

		m_posOffset = ((1 + m_jacSizeBoth) * m_numRows);
		m_data.resize((2 + m_jacSizeBoth) * m_numRows);
	}

	int getConstraintType() const
	{
		return m_type;
	}
	//many constraints have setFrameInB/setPivotInB. Will use 'getConstraintType' later.
	virtual void setFrameInB(const btMatrix3x3& frameInB) {}
	virtual void setPivotInB(const btVector3& pivotInB) {}

	virtual void finalizeMultiDof() = 0;

	virtual int getIslandIdA() const = 0;
	virtual int getIslandIdB() const = 0;

	virtual void createConstraintRows(btMultiBodyConstraintArray & constraintRows,
									  btMultiBodyJacobianData & data,
									  const btContactSolverInfo& infoGlobal) = 0;

	int getNumRows() const
	{
		return m_numRows;
	}

	btMultiBody* getMultiBodyA()
	{
		return m_bodyA;
	}
	btMultiBody* getMultiBodyB()
	{
		return m_bodyB;
	}

	int getLinkA() const
	{
		return m_linkA;
	}
	int getLinkB() const
	{
		return m_linkB;
	}
	void internalSetAppliedImpulse(int dof, btScalar appliedImpulse)
	{
		btAssert(dof >= 0);
		btAssert(dof < getNumRows());
		m_data[dof] = appliedImpulse;
	}

	btScalar getAppliedImpulse(int dof)
	{
		btAssert(dof >= 0);
		btAssert(dof < getNumRows());
		return m_data[dof];
	}
	// current constraint position
	// constraint is pos >= 0 for unilateral, or pos = 0 for bilateral
	// NOTE: ignored position for friction rows.
	btScalar getPosition(int row) const
	{
		return m_data[m_posOffset + row];
	}

	void setPosition(int row, btScalar pos)
	{
		m_data[m_posOffset + row] = pos;
	}

	bool isUnilateral() const
	{
		return m_isUnilateral;
	}

	// jacobian blocks.
	// each of size 6 + num_links. (jacobian2 is null if no body2.)
	// format: 3 'omega' coefficients, 3 'v' coefficients, then the 'qdot' coefficients.
	btScalar* jacobianA(int row)
	{
		return &m_data[m_numRows + row * m_jacSizeBoth];
	}
	const btScalar* jacobianA(int row) const
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth)];
	}
	btScalar* jacobianB(int row)
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth) + m_jacSizeA];
	}
	const btScalar* jacobianB(int row) const
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth) + m_jacSizeA];
	}

	btScalar getMaxAppliedImpulse() const
	{
		return m_maxAppliedImpulse;
	}
	void setMaxAppliedImpulse(btScalar maxImp)
	{
		m_maxAppliedImpulse = maxImp;
	}

	virtual void debugDraw(class btIDebugDraw * drawer) = 0;

	virtual void setGearRatio(btScalar ratio) {}
	virtual void setGearAuxLink(int gearAuxLink) {}
	virtual void setRelativePositionTarget(btScalar relPosTarget) {}
	virtual void setErp(btScalar erp) {}
};

#endif  //BT_MULTIBODY_CONSTRAINT_H
