#ifndef BT_CONTACT_CONSTRAINT_H
#define BT_CONTACT_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btContactSolverInfo.h"
#include "LinearMath/btMinMax.h"

///btContactConstraint can be automatically created to solve contact constraints using the unified btTypedConstraint interface
ATTRIBUTE_ALIGNED16(class)
btContactConstraint : public btTypedConstraint
{
protected:
	btPersistentManifold m_contactManifold;

protected:
	btContactConstraint(btPersistentManifold * contactManifold, btRigidBody & rbA, btRigidBody & rbB)
		: btTypedConstraint(CONTACT_CONSTRAINT_TYPE, rbA, rbB),
		  m_contactManifold(*contactManifold)
	{
	}

public:
	void setContactManifold(btPersistentManifold * contactManifold)
	{
		m_contactManifold = *contactManifold;
	}

	btPersistentManifold* getContactManifold()
	{
		return &m_contactManifold;
	}

	const btPersistentManifold* getContactManifold() const
	{
		return &m_contactManifold;
	}

	virtual ~btContactConstraint() {}

	virtual void getInfo1(btConstraintInfo1 * info) {}
	virtual void getInfo2(btConstraintInfo2 * info) {}

	///obsolete methods
	virtual void buildJacobian() {}
};

// very basic collision resolution without friction
// response  between two dynamic objects without friction and no restitution, assuming 0 penetration depth
static btScalar resolveSingleCollision(btRigidBody* body1, btCollisionObject* colObj2, const btVector3& contactPositionWorld, const btVector3& contactNormalOnB, const btContactSolverInfo& solverInfo, btScalar distance)
{
	btRigidBody* body2 = btRigidBody::upcast(colObj2);

	const btVector3& normal = contactNormalOnB;

	btVector3 rel_pos1 = contactPositionWorld - body1->getWorldTransform().getOrigin();
	btVector3 rel_pos2 = contactPositionWorld - colObj2->getWorldTransform().getOrigin();

	btVector3 vel1 = body1->getVelocityInLocalPoint(rel_pos1);
	btVector3 vel2 = body2 ? body2->getVelocityInLocalPoint(rel_pos2) : btVector3(0, 0, 0);
	btVector3 vel = vel1 - vel2;
	btScalar rel_vel;
	rel_vel = normal.dot(vel);

	btScalar combinedRestitution = 0.f;
	btScalar restitution = combinedRestitution * -rel_vel;

	btScalar positionalError = solverInfo.m_erp * -distance / solverInfo.m_timeStep;
	btScalar velocityError = -(1.0f + restitution) * rel_vel;  // * damping;
	btScalar denom0 = body1->computeImpulseDenominator(contactPositionWorld, normal);
	btScalar denom1 = body2 ? body2->computeImpulseDenominator(contactPositionWorld, normal) : 0.f;
	btScalar relaxation = 1.f;
	btScalar jacDiagABInv = relaxation / (denom0 + denom1);

	btScalar penetrationImpulse = positionalError * jacDiagABInv;
	btScalar velocityImpulse = velocityError * jacDiagABInv;

	btScalar normalImpulse = penetrationImpulse + velocityImpulse;
	normalImpulse = 0.f > normalImpulse ? 0.f : normalImpulse;

	body1->applyImpulse(normal * (normalImpulse), rel_pos1);
	if (body2)
		body2->applyImpulse(-normal * (normalImpulse), rel_pos2);

	return normalImpulse;
}

// resolveSingleBilateral is an obsolete methods used for vehicle friction between two dynamic objects
// bilateral constraint between two dynamic objects
static void resolveSingleBilateral(btRigidBody& body1, const btVector3& pos1, btRigidBody& body2, const btVector3& pos2, btScalar distance, const btVector3& normal, btScalar& impulse, btScalar timeStep)
{
	(void)timeStep;
	(void)distance;

	btScalar normalLenSqr = normal.length2();
	btAssert(btFabs(normalLenSqr) < btScalar(1.1));
	if (normalLenSqr > btScalar(1.1))
	{
		impulse = btScalar(0.);
		return;
	}
	btVector3 rel_pos1 = pos1 - body1.getCenterOfMassPosition();
	btVector3 rel_pos2 = pos2 - body2.getCenterOfMassPosition();
	//this jacobian entry could be re-used for all iterations

	btVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
	btVector3 vel2 = body2.getVelocityInLocalPoint(rel_pos2);
	btVector3 vel = vel1 - vel2;

	btJacobianEntry jac(body1.getCenterOfMassTransform().getBasis().transpose(),
						body2.getCenterOfMassTransform().getBasis().transpose(),
						rel_pos1, rel_pos2, normal, body1.getInvInertiaDiagLocal(), body1.getInvMass(),
						body2.getInvInertiaDiagLocal(), body2.getInvMass());

	btScalar jacDiagAB = jac.getDiagonal();
	btScalar jacDiagABInv = btScalar(1.) / jacDiagAB;

	btScalar rel_vel = jac.getRelativeVelocity(
		body1.getLinearVelocity(),
		body1.getCenterOfMassTransform().getBasis().transpose() * body1.getAngularVelocity(),
		body2.getLinearVelocity(),
		body2.getCenterOfMassTransform().getBasis().transpose() * body2.getAngularVelocity());

	rel_vel = normal.dot(vel);

	//todo: move this into proper structure
	btScalar contactDamping = btScalar(0.2);

	btScalar velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
	impulse = velocityImpulse;
}

#endif  //BT_CONTACT_CONSTRAINT_H
