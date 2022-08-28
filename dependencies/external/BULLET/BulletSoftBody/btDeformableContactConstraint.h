/*
	Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
	Bullet Continuous Collision Detection and Physics Library
	Copyright (c) 2019 Google Inc. http://bulletphysics.org
*/

#ifndef BT_DEFORMABLE_CONTACT_CONSTRAINT_H
#define BT_DEFORMABLE_CONTACT_CONSTRAINT_H
#include "btSoftBody.h"

// btDeformableContactConstraint is an abstract class specifying the method that each type of contact constraint needs to implement
class btDeformableContactConstraint
{
public:
	bool m_static; // true if friction is static, false if friction is dynamic
	const btContactSolverInfo* m_infoGlobal;

	// normal of the contact
	btVector3 m_normal;

	btDeformableContactConstraint(const btVector3& normal, const btContactSolverInfo& infoGlobal) : m_static(false), m_normal(normal), m_infoGlobal(&infoGlobal)
	{
	}

	btDeformableContactConstraint(bool isStatic, const btVector3& normal, const btContactSolverInfo& infoGlobal) : m_static(isStatic), m_normal(normal), m_infoGlobal(&infoGlobal)
	{
	}

	btDeformableContactConstraint() : m_static(false) {}

	btDeformableContactConstraint(const btDeformableContactConstraint& other)
		: m_static(other.m_static), m_normal(other.m_normal), m_infoGlobal(other.m_infoGlobal)
	{
	}

	virtual ~btDeformableContactConstraint() {}

	// solve the constraint with inelastic impulse and return the error, which is the square of normal component of velocity diffrerence
	// the constraint is solved by calculating the impulse between object A and B in the contact and apply the impulse to both objects involved in the contact
	virtual btScalar solveConstraint(const btContactSolverInfo& infoGlobal) = 0;

	// get the velocity of the object A in the contact
	virtual btVector3 getVa() const = 0;

	// get the velocity of the object B in the contact
	virtual btVector3 getVb() const = 0;

	// get the velocity change of the soft body node in the constraint
	virtual btVector3 getDv(const btSoftBody::Node*) const = 0;

	// apply impulse to the soft body node and/or face involved
	virtual void applyImpulse(const btVector3& impulse) = 0;

	// scale the penetration depth by erp
	virtual void setPenetrationScale(btScalar scale) = 0;
};

//
// Constraint that a certain node in the deformable objects cannot move
class btDeformableStaticConstraint : public btDeformableContactConstraint
{
public:
	btSoftBody::Node* m_node;

	btDeformableStaticConstraint(btSoftBody::Node* node, const btContactSolverInfo& infoGlobal) : m_node(node), btDeformableContactConstraint(false, btVector3(0, 0, 0), infoGlobal)
	{
	}
	btDeformableStaticConstraint() {}
	btDeformableStaticConstraint(const btDeformableStaticConstraint& other)
		: m_node(other.m_node), btDeformableContactConstraint(other)
	{
	}

	virtual ~btDeformableStaticConstraint() {}

	virtual btScalar solveConstraint(const btContactSolverInfo& infoGlobal)
	{
		return 0;
	}

	virtual btVector3 getVa() const
	{
		return btVector3(0, 0, 0);
	}

	virtual btVector3 getVb() const
	{
		return btVector3(0, 0, 0);
	}

	virtual btVector3 getDv(const btSoftBody::Node* n) const
	{
		return btVector3(0, 0, 0);
	}

	virtual void applyImpulse(const btVector3& impulse) {}
	virtual void setPenetrationScale(btScalar scale) {}
};

// -------------------   Deformable Node Anchor   ------------------- //
// ------  Anchor Constraint between rigid and deformable node -----  //
class btDeformableNodeAnchorConstraint : public btDeformableContactConstraint
{
public:
	const btSoftBody::DeformableNodeRigidAnchor* m_anchor;

	btDeformableNodeAnchorConstraint(const btSoftBody::DeformableNodeRigidAnchor& a, const btContactSolverInfo& infoGlobal)
		: m_anchor(&a), btDeformableContactConstraint(a.m_cti.m_normal, infoGlobal)
	{
	}
	btDeformableNodeAnchorConstraint(const btDeformableNodeAnchorConstraint& other)
		: m_anchor(other.m_anchor), btDeformableContactConstraint(other)
	{
	}
	btDeformableNodeAnchorConstraint() {}
	virtual ~btDeformableNodeAnchorConstraint()
	{
	}
	virtual btScalar solveConstraint(const btContactSolverInfo& infoGlobal)
	{
		const btSoftBody::sCti& cti = m_anchor->m_cti;
		btVector3 va = getVa();
		btVector3 vb = getVb();
		btVector3 vr = (vb - va);
		// + (m_anchor->m_node->m_x - cti.m_colObj->getWorldTransform() * m_anchor->m_local) * 10.0
		const btScalar dn = btDot(vr, vr);
		// dn is the normal component of velocity diffrerence. Approximates the residual. // todo xuchenhan@: this prob needs to be scaled by dt
		btScalar residualSquare = dn * dn;
		btVector3 impulse = m_anchor->m_c0 * vr;
		// apply impulse to deformable nodes involved and change their velocities
		applyImpulse(impulse);

		// apply impulse to the rigid/multibodies involved and change their velocities
		if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
		{
			btRigidBody* rigidCol = 0;
			rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
			if (rigidCol)
			{
				rigidCol->applyImpulse(impulse, m_anchor->m_c1);
			}
		}
		else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
		{
			btMultiBodyLinkCollider* multibodyLinkCol = 0;
			multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
			if (multibodyLinkCol)
			{
				const btScalar* deltaV_normal = &m_anchor->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
				// apply normal component of the impulse
				multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_normal, impulse.dot(cti.m_normal));
				// apply tangential component of the impulse
				const btScalar* deltaV_t1 = &m_anchor->jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
				multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t1, impulse.dot(m_anchor->t1));
				const btScalar* deltaV_t2 = &m_anchor->jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];
				multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t2, impulse.dot(m_anchor->t2));
			}
		}
		return residualSquare;
	}

	// object A is the rigid/multi body, and object B is the deformable node/face
	virtual btVector3 getVa() const
	{
		const btSoftBody::sCti& cti = m_anchor->m_cti;
		btVector3 va(0, 0, 0);
		if (cti.m_colObj->hasContactResponse())
		{
			btRigidBody* rigidCol = 0;
			btMultiBodyLinkCollider* multibodyLinkCol = 0;

			// grab the velocity of the rigid body
			if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
			{
				rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
				va = rigidCol ? (rigidCol->getVelocityInLocalPoint(m_anchor->m_c1)) : btVector3(0, 0, 0);
			}
			else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
			{
				multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
				if (multibodyLinkCol)
				{
					const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
					const btScalar* J_n = &m_anchor->jacobianData_normal.m_jacobians[0];
					const btScalar* J_t1 = &m_anchor->jacobianData_t1.m_jacobians[0];
					const btScalar* J_t2 = &m_anchor->jacobianData_t2.m_jacobians[0];
					const btScalar* local_v = multibodyLinkCol->m_multiBody->getVelocityVector();
					const btScalar* local_dv = multibodyLinkCol->m_multiBody->getDeltaVelocityVector();
					// add in the normal component of the va
					btScalar vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += (local_v[k] + local_dv[k]) * J_n[k];
					}
					va = cti.m_normal * vel;
					// add in the tangential components of the va
					vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += (local_v[k] + local_dv[k]) * J_t1[k];
					}
					va += m_anchor->t1 * vel;
					vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += (local_v[k] + local_dv[k]) * J_t2[k];
					}
					va += m_anchor->t2 * vel;
				}
			}
		}
		return va;
	}
	// get the velocity of the deformable node in contact
	virtual btVector3 getVb() const
	{
		return m_anchor->m_node->m_v;
	}
	virtual btVector3 getDv(const btSoftBody::Node* n) const
	{
		return btVector3(0, 0, 0);
	}
	virtual void applyImpulse(const btVector3& impulse)
	{
		btVector3 dv = impulse * m_anchor->m_c2;
		m_anchor->m_node->m_v -= dv;
	}

	virtual void setPenetrationScale(btScalar scale) {}
};

// -------------------   Deformable vs. Rigid   ------------------- //
// -- Constraint between rigid/multi body and deformable objects -- //
class btDeformableRigidContactConstraint : public btDeformableContactConstraint
{
public:
	btVector3 m_total_normal_dv;
	btVector3 m_total_tangent_dv;
	btScalar m_penetration;
	btScalar m_total_split_impulse;
	bool m_binding;
	const btSoftBody::DeformableRigidContact* m_contact;

	btDeformableRigidContactConstraint(const btSoftBody::DeformableRigidContact& c, const btContactSolverInfo& infoGlobal)
		: m_contact(&c), btDeformableContactConstraint(c.m_cti.m_normal, infoGlobal)
	{
		m_total_normal_dv.setZero();
		m_total_tangent_dv.setZero();
		// The magnitude of penetration is the depth of penetration.
		m_penetration = c.m_cti.m_offset;
		m_total_split_impulse = 0;
		m_binding = false;
	}
	btDeformableRigidContactConstraint(const btDeformableRigidContactConstraint& other)
		: m_contact(other.m_contact), btDeformableContactConstraint(other), m_penetration(other.m_penetration), m_total_split_impulse(other.m_total_split_impulse), m_binding(other.m_binding)
	{
		m_total_normal_dv = other.m_total_normal_dv;
		m_total_tangent_dv = other.m_total_tangent_dv;
	}
	btDeformableRigidContactConstraint() : m_binding(false) {}
	virtual ~btDeformableRigidContactConstraint()
	{
	}

	// object A is the rigid/multi body, and object B is the deformable node/face
	virtual btVector3 getVa() const
	{
		const btSoftBody::sCti& cti = m_contact->m_cti;
		btVector3 va(0, 0, 0);
		if (cti.m_colObj->hasContactResponse())
		{
			btRigidBody* rigidCol = 0;
			btMultiBodyLinkCollider* multibodyLinkCol = 0;

			// grab the velocity of the rigid body
			if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
			{
				rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
				va = rigidCol ? (rigidCol->getVelocityInLocalPoint(m_contact->m_c1)) : btVector3(0, 0, 0);
			}
			else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
			{
				multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
				if (multibodyLinkCol)
				{
					const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
					const btScalar* J_n = &m_contact->jacobianData_normal.m_jacobians[0];
					const btScalar* J_t1 = &m_contact->jacobianData_t1.m_jacobians[0];
					const btScalar* J_t2 = &m_contact->jacobianData_t2.m_jacobians[0];
					const btScalar* local_v = multibodyLinkCol->m_multiBody->getVelocityVector();
					const btScalar* local_dv = multibodyLinkCol->m_multiBody->getDeltaVelocityVector();
					// add in the normal component of the va
					btScalar vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += (local_v[k] + local_dv[k]) * J_n[k];
					}
					va = cti.m_normal * vel;
					// add in the tangential components of the va
					vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += (local_v[k] + local_dv[k]) * J_t1[k];
					}
					va += m_contact->t1 * vel;
					vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += (local_v[k] + local_dv[k]) * J_t2[k];
					}
					va += m_contact->t2 * vel;
				}
			}
		}
		return va;
	}

	// get the split impulse velocity of the deformable face at the contact point
	virtual btVector3 getSplitVb() const = 0;

	// get the split impulse velocity of the rigid/multibdoy at the contaft
	virtual btVector3 getSplitVa() const
	{
		const btSoftBody::sCti& cti = m_contact->m_cti;
		btVector3 va(0, 0, 0);
		if (cti.m_colObj->hasContactResponse())
		{
			btRigidBody* rigidCol = 0;
			btMultiBodyLinkCollider* multibodyLinkCol = 0;

			// grab the velocity of the rigid body
			if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
			{
				rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
				va = rigidCol ? (rigidCol->getPushVelocityInLocalPoint(m_contact->m_c1)) : btVector3(0, 0, 0);
			}
			else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
			{
				multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
				if (multibodyLinkCol)
				{
					const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
					const btScalar* J_n = &m_contact->jacobianData_normal.m_jacobians[0];
					const btScalar* J_t1 = &m_contact->jacobianData_t1.m_jacobians[0];
					const btScalar* J_t2 = &m_contact->jacobianData_t2.m_jacobians[0];
					const btScalar* local_split_v = multibodyLinkCol->m_multiBody->getSplitVelocityVector();
					// add in the normal component of the va
					btScalar vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += local_split_v[k] * J_n[k];
					}
					va = cti.m_normal * vel;
					// add in the tangential components of the va
					vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += local_split_v[k] * J_t1[k];
					}
					va += m_contact->t1 * vel;
					vel = 0.0;
					for (int k = 0; k < ndof; ++k)
					{
						vel += local_split_v[k] * J_t2[k];
					}
					va += m_contact->t2 * vel;
				}
			}
		}
		return va;
	}

	virtual btScalar solveConstraint(const btContactSolverInfo& infoGlobal)
	{
		const btSoftBody::sCti& cti = m_contact->m_cti;
		btVector3 va = getVa();
		btVector3 vb = getVb();
		btVector3 vr = vb - va;
		btScalar dn = btDot(vr, cti.m_normal) + m_total_normal_dv.dot(cti.m_normal) * infoGlobal.m_deformable_cfm;
		if (m_penetration > 0)
		{
			dn += m_penetration / infoGlobal.m_timeStep;
		}
		if (!infoGlobal.m_splitImpulse)
		{
			dn += m_penetration * infoGlobal.m_deformable_erp / infoGlobal.m_timeStep;
		}
		// dn is the normal component of velocity difference. Approximates the residual. // todo xuchenhan@: this prob needs to be scaled by dt
		btVector3 impulse = m_contact->m_c0 * (vr + m_total_normal_dv * infoGlobal.m_deformable_cfm + ((m_penetration > 0) ? m_penetration / infoGlobal.m_timeStep * cti.m_normal : btVector3(0, 0, 0)));
		if (!infoGlobal.m_splitImpulse)
		{
			impulse += m_contact->m_c0 * (m_penetration * infoGlobal.m_deformable_erp / infoGlobal.m_timeStep * cti.m_normal);
		}
		btVector3 impulse_normal = m_contact->m_c0 * (cti.m_normal * dn);
		btVector3 impulse_tangent = impulse - impulse_normal;
		if (dn > 0)
		{
			return 0;
		}
		m_binding = true;
		btScalar residualSquare = dn * dn;
		btVector3 old_total_tangent_dv = m_total_tangent_dv;
		// m_c5 is the inverse mass of the deformable node/face
		m_total_normal_dv -= m_contact->m_c5 * impulse_normal;
		m_total_tangent_dv -= m_contact->m_c5 * impulse_tangent;

		if (m_total_normal_dv.dot(cti.m_normal) < 0)
		{
			// separating in the normal direction
			m_binding = false;
			m_static = false;
			impulse_tangent.setZero();
		}
		else
		{
			if (m_total_normal_dv.norm() * m_contact->m_c3 < m_total_tangent_dv.norm())
			{
				// dynamic friction
				// with dynamic friction, the impulse are still applied to the two objects colliding, however, it does not pose a constraint in the cg solve, hence the change to dv merely serves to update velocity in the contact iterations.
				m_static = false;
				if (m_total_tangent_dv.safeNorm() < SIMD_EPSILON)
				{
					m_total_tangent_dv = btVector3(0, 0, 0);
				}
				else
				{
					m_total_tangent_dv = m_total_tangent_dv.normalized() * m_total_normal_dv.safeNorm() * m_contact->m_c3;
				}
				//            impulse_tangent = -btScalar(1)/m_contact->m_c2 * (m_total_tangent_dv - old_total_tangent_dv);
				impulse_tangent = m_contact->m_c5.inverse() * (old_total_tangent_dv - m_total_tangent_dv);
			}
			else
			{
				// static friction
				m_static = true;
			}
		}
		impulse = impulse_normal + impulse_tangent;
		// apply impulse to deformable nodes involved and change their velocities
		applyImpulse(impulse);
		// apply impulse to the rigid/multibodies involved and change their velocities
		if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
		{
			btRigidBody* rigidCol = 0;
			rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
			if (rigidCol)
			{
				rigidCol->applyImpulse(impulse, m_contact->m_c1);
			}
		}
		else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
		{
			btMultiBodyLinkCollider* multibodyLinkCol = 0;
			multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
			if (multibodyLinkCol)
			{
				const btScalar* deltaV_normal = &m_contact->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
				// apply normal component of the impulse
				multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_normal, impulse.dot(cti.m_normal));
				if (impulse_tangent.norm() > SIMD_EPSILON)
				{
					// apply tangential component of the impulse
					const btScalar* deltaV_t1 = &m_contact->jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
					multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t1, impulse.dot(m_contact->t1));
					const btScalar* deltaV_t2 = &m_contact->jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];
					multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t2, impulse.dot(m_contact->t2));
				}
			}
		}
		return residualSquare;
	}

	virtual void setPenetrationScale(btScalar scale)
	{
		m_penetration *= scale;
	}

	btScalar solveSplitImpulse(const btContactSolverInfo& infoGlobal)
	{
		btScalar MAX_PENETRATION_CORRECTION = infoGlobal.m_deformable_maxErrorReduction;
		const btSoftBody::sCti& cti = m_contact->m_cti;
		btVector3 vb = getSplitVb();
		btVector3 va = getSplitVa();
		btScalar p = m_penetration;
		if (p > 0)
		{
			return 0;
		}
		btVector3 vr = vb - va;
		btScalar dn = btDot(vr, cti.m_normal) + p * infoGlobal.m_deformable_erp / infoGlobal.m_timeStep;
		if (dn > 0)
		{
			return 0;
		}
		if (m_total_split_impulse + dn > MAX_PENETRATION_CORRECTION)
		{
			dn = MAX_PENETRATION_CORRECTION - m_total_split_impulse;
		}
		if (m_total_split_impulse + dn < -MAX_PENETRATION_CORRECTION)
		{
			dn = -MAX_PENETRATION_CORRECTION - m_total_split_impulse;
		}
		m_total_split_impulse += dn;

		btScalar residualSquare = dn * dn;
		const btVector3 impulse = m_contact->m_c0 * (cti.m_normal * dn);
		applySplitImpulse(impulse);

		// apply split impulse to the rigid/multibodies involved and change their velocities
		if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
		{
			btRigidBody* rigidCol = 0;
			rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
			if (rigidCol)
			{
				rigidCol->applyPushImpulse(impulse, m_contact->m_c1);
			}
		}
		else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
		{
			btMultiBodyLinkCollider* multibodyLinkCol = 0;
			multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
			if (multibodyLinkCol)
			{
				const btScalar* deltaV_normal = &m_contact->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
				// apply normal component of the impulse
				multibodyLinkCol->m_multiBody->applyDeltaSplitVeeMultiDof(deltaV_normal, impulse.dot(cti.m_normal));
			}
		}
		return residualSquare;
	}

	virtual void applySplitImpulse(const btVector3& impulse) = 0;
};

// ------------------------   Node vs. Rigid   ------------------------ //
// - Constraint between rigid/multi body and deformable objects nodes - //
class btDeformableNodeRigidContactConstraint : public btDeformableRigidContactConstraint
{
public:
	// the deformable node in contact
	btSoftBody::Node* m_node;

	btDeformableNodeRigidContactConstraint(const btSoftBody::DeformableNodeRigidContact& contact, const btContactSolverInfo& infoGlobal)
		: m_node(contact.m_node), btDeformableRigidContactConstraint(contact, infoGlobal)
	{
	}
	btDeformableNodeRigidContactConstraint(const btDeformableNodeRigidContactConstraint& other)
		: m_node(other.m_node), btDeformableRigidContactConstraint(other)
	{
	}
	btDeformableNodeRigidContactConstraint() {}
	virtual ~btDeformableNodeRigidContactConstraint()
	{
	}

	// get the velocity of the deformable node in contact
	virtual btVector3 getVb() const
	{
		return m_node->m_v;
	}

	// get the split impulse velocity of the deformable face at the contact point
	virtual btVector3 getSplitVb() const
	{
		return m_node->m_splitv;
	}

	// get the velocity change of the input soft body node in the constraint
	virtual btVector3 getDv(const btSoftBody::Node* node) const
	{
		return m_total_normal_dv + m_total_tangent_dv;
	}

	// cast the contact to the desired type
	const btSoftBody::DeformableNodeRigidContact* getContact() const
	{
		return static_cast<const btSoftBody::DeformableNodeRigidContact*>(m_contact);
	}

	virtual void applyImpulse(const btVector3& impulse)
	{
		const btSoftBody::DeformableNodeRigidContact* contact = getContact();
		btVector3 dv = contact->m_c5 * impulse;
		contact->m_node->m_v -= dv;
	}

	virtual void applySplitImpulse(const btVector3& impulse)
	{
		const btSoftBody::DeformableNodeRigidContact* contact = getContact();
		btVector3 dv = contact->m_c5 * impulse;
		contact->m_node->m_splitv -= dv;
	}
};

// ------------------------   Face vs. Rigid   ------------------------ //
// - Constraint between rigid/multi body and deformable objects faces - //
class btDeformableFaceRigidContactConstraint : public btDeformableRigidContactConstraint
{
public:
	btSoftBody::Face* m_face;
	bool m_useStrainLimiting;
	btDeformableFaceRigidContactConstraint(const btSoftBody::DeformableFaceRigidContact& contact, const btContactSolverInfo& infoGlobal, bool useStrainLimiting)
		: m_face(contact.m_face), m_useStrainLimiting(useStrainLimiting), btDeformableRigidContactConstraint(contact, infoGlobal)
	{
	}
	btDeformableFaceRigidContactConstraint(const btDeformableFaceRigidContactConstraint& other)
		: m_face(other.m_face), m_useStrainLimiting(other.m_useStrainLimiting), btDeformableRigidContactConstraint(other)
	{
	}
	btDeformableFaceRigidContactConstraint() : m_useStrainLimiting(false) {}
	virtual ~btDeformableFaceRigidContactConstraint()
	{
	}

	// get the velocity of the deformable face at the contact point
	virtual btVector3 getVb() const
	{
		const btSoftBody::DeformableFaceRigidContact* contact = getContact();
		btVector3 vb = m_face->m_n[0]->m_v * contact->m_bary[0] + m_face->m_n[1]->m_v * contact->m_bary[1] + m_face->m_n[2]->m_v * contact->m_bary[2];
		return vb;
	}

	// get the split impulse velocity of the deformable face at the contact point
	virtual btVector3 getSplitVb() const
	{
		const btSoftBody::DeformableFaceRigidContact* contact = getContact();
		btVector3 vb = (m_face->m_n[0]->m_splitv) * contact->m_bary[0] + (m_face->m_n[1]->m_splitv) * contact->m_bary[1] + (m_face->m_n[2]->m_splitv) * contact->m_bary[2];
		return vb;
	}

	// get the velocity change of the input soft body node in the constraint
	virtual btVector3 getDv(const btSoftBody::Node* node) const
	{
		btVector3 face_dv = m_total_normal_dv + m_total_tangent_dv;
		const btSoftBody::DeformableFaceRigidContact* contact = getContact();
		if (m_face->m_n[0] == node)
		{
			return face_dv * contact->m_weights[0];
		}
		if (m_face->m_n[1] == node)
		{
			return face_dv * contact->m_weights[1];
		}
		btAssert(node == m_face->m_n[2]);
		return face_dv * contact->m_weights[2];
	}

	// cast the contact to the desired type
	const btSoftBody::DeformableFaceRigidContact* getContact() const
	{
		return static_cast<const btSoftBody::DeformableFaceRigidContact*>(m_contact);
	}

	virtual void applyImpulse(const btVector3& impulse)
	{
		const btSoftBody::DeformableFaceRigidContact* contact = getContact();
		btVector3 dv = impulse * contact->m_c2;
		btSoftBody::Face* face = contact->m_face;

		// save applied impulse
		contact->m_cti.m_impulse = impulse;

		btVector3& v0 = face->m_n[0]->m_v;
		btVector3& v1 = face->m_n[1]->m_v;
		btVector3& v2 = face->m_n[2]->m_v;
		const btScalar& im0 = face->m_n[0]->m_im;
		const btScalar& im1 = face->m_n[1]->m_im;
		const btScalar& im2 = face->m_n[2]->m_im;
		if (im0 > 0)
			v0 -= dv * contact->m_weights[0];
		if (im1 > 0)
			v1 -= dv * contact->m_weights[1];
		if (im2 > 0)
			v2 -= dv * contact->m_weights[2];
		if (m_useStrainLimiting)
		{
			btScalar relaxation = 1. / btScalar(m_infoGlobal->m_numIterations);
			btScalar m01 = (relaxation / (im0 + im1));
			btScalar m02 = (relaxation / (im0 + im2));
			btScalar m12 = (relaxation / (im1 + im2));

			// apply strain limiting to prevent undamped modes
			btVector3 dv0 = im0 * (m01 * (v1 - v0) + m02 * (v2 - v0));
			btVector3 dv1 = im1 * (m01 * (v0 - v1) + m12 * (v2 - v1));
			btVector3 dv2 = im2 * (m12 * (v1 - v2) + m02 * (v0 - v2));

			v0 += dv0;
			v1 += dv1;
			v2 += dv2;
		}
	}

	virtual void applySplitImpulse(const btVector3& impulse)
	{
		const btSoftBody::DeformableFaceRigidContact* contact = getContact();
		btVector3 dv = impulse * contact->m_c2;
		btSoftBody::Face* face = contact->m_face;
		btVector3& v0 = face->m_n[0]->m_splitv;
		btVector3& v1 = face->m_n[1]->m_splitv;
		btVector3& v2 = face->m_n[2]->m_splitv;
		const btScalar& im0 = face->m_n[0]->m_im;
		const btScalar& im1 = face->m_n[1]->m_im;
		const btScalar& im2 = face->m_n[2]->m_im;

		if (im0 > 0)
			v0 -= dv * contact->m_weights[0];

		if (im1 > 0)
			v1 -= dv * contact->m_weights[1];

		if (im2 > 0)
			v2 -= dv * contact->m_weights[2];
	}
};

// -----------------------------   Face vs. Node   ----------------------------- //
// - Constraint between  deformable objects faces and deformable objects nodes - //
class btDeformableFaceNodeContactConstraint : public btDeformableContactConstraint
{
public:
	btSoftBody::Node* m_node;
	btSoftBody::Face* m_face;
	const btSoftBody::DeformableFaceNodeContact* m_contact;
	btVector3 m_total_normal_dv;
	btVector3 m_total_tangent_dv;

	btDeformableFaceNodeContactConstraint(const btSoftBody::DeformableFaceNodeContact& contact, const btContactSolverInfo& infoGlobal)
		: m_node(contact.m_node), m_face(contact.m_face), m_contact(&contact), btDeformableContactConstraint(contact.m_normal, infoGlobal)
	{
		m_total_normal_dv.setZero();
		m_total_tangent_dv.setZero();
	}
	btDeformableFaceNodeContactConstraint() {}
	virtual ~btDeformableFaceNodeContactConstraint() {}

	virtual btScalar solveConstraint(const btContactSolverInfo& infoGlobal)
	{
		btVector3 va = getVa();
		btVector3 vb = getVb();
		btVector3 vr = vb - va;
		const btScalar dn = btDot(vr, m_contact->m_normal);
		// dn is the normal component of velocity diffrerence. Approximates the residual. // todo xuchenhan@: this prob needs to be scaled by dt
		btScalar residualSquare = dn * dn;
		btVector3 impulse = m_contact->m_c0 * vr;
		const btVector3 impulse_normal = m_contact->m_c0 * (m_contact->m_normal * dn);
		btVector3 impulse_tangent = impulse - impulse_normal;

		btVector3 old_total_tangent_dv = m_total_tangent_dv;
		// m_c2 is the inverse mass of the deformable node/face
		if (m_node->m_im > 0)
		{
			m_total_normal_dv -= impulse_normal * m_node->m_im;
			m_total_tangent_dv -= impulse_tangent * m_node->m_im;
		}
		else
		{
			m_total_normal_dv -= impulse_normal * m_contact->m_imf;
			m_total_tangent_dv -= impulse_tangent * m_contact->m_imf;
		}

		if (m_total_normal_dv.dot(m_contact->m_normal) > 0)
		{
			// separating in the normal direction
			m_static = false;
			m_total_tangent_dv = btVector3(0, 0, 0);
			impulse_tangent.setZero();
		}
		else
		{
			if (m_total_normal_dv.norm() * m_contact->m_friction < m_total_tangent_dv.norm())
			{
				// dynamic friction
				// with dynamic friction, the impulse are still applied to the two objects colliding, however, it does not pose a constraint in the cg solve, hence the change to dv merely serves to update velocity in the contact iterations.
				m_static = false;
				if (m_total_tangent_dv.safeNorm() < SIMD_EPSILON)
				{
					m_total_tangent_dv = btVector3(0, 0, 0);
				}
				else
				{
					m_total_tangent_dv = m_total_tangent_dv.normalized() * m_total_normal_dv.safeNorm() * m_contact->m_friction;
				}
				impulse_tangent = -btScalar(1) / m_node->m_im * (m_total_tangent_dv - old_total_tangent_dv);
			}
			else
			{
				// static friction
				m_static = true;
			}
		}
		impulse = impulse_normal + impulse_tangent;
		// apply impulse to deformable nodes involved and change their velocities
		applyImpulse(impulse);
		return residualSquare;
	}

	// get the velocity of the object A in the contact
	virtual btVector3 getVa() const
	{
		return m_node->m_v;
	}

	// get the velocity of the object B in the contact
	virtual btVector3 getVb() const
	{
		const btSoftBody::DeformableFaceNodeContact* contact = getContact();
		btVector3 vb = m_face->m_n[0]->m_v * contact->m_bary[0] + m_face->m_n[1]->m_v * contact->m_bary[1] + m_face->m_n[2]->m_v * contact->m_bary[2];
		return vb;
	}

	// get the velocity change of the input soft body node in the constraint
	virtual btVector3 getDv(const btSoftBody::Node* n) const
	{
		btVector3 dv = m_total_normal_dv + m_total_tangent_dv;
		if (n == m_node)
			return dv;
		const btSoftBody::DeformableFaceNodeContact* contact = getContact();
		if (m_face->m_n[0] == n)
		{
			return dv * contact->m_weights[0];
		}
		if (m_face->m_n[1] == n)
		{
			return dv * contact->m_weights[1];
		}
		btAssert(n == m_face->m_n[2]);
		return dv * contact->m_weights[2];
	}

	// cast the contact to the desired type
	const btSoftBody::DeformableFaceNodeContact* getContact() const
	{
		return static_cast<const btSoftBody::DeformableFaceNodeContact*>(m_contact);
	}

	virtual void applyImpulse(const btVector3& impulse)
	{
		const btSoftBody::DeformableFaceNodeContact* contact = getContact();
		btVector3 dva = impulse * contact->m_node->m_im;
		btVector3 dvb = impulse * contact->m_imf;

		if (contact->m_node->m_im > 0)
			contact->m_node->m_v += dva;

		btSoftBody::Face* face = contact->m_face;
		btVector3& v0 = face->m_n[0]->m_v;
		btVector3& v1 = face->m_n[1]->m_v;
		btVector3& v2 = face->m_n[2]->m_v;
		const btScalar& im0 = face->m_n[0]->m_im;
		const btScalar& im1 = face->m_n[1]->m_im;
		const btScalar& im2 = face->m_n[2]->m_im;

		if (im0 > 0)
			v0 -= dvb * contact->m_weights[0];

		if (im1 > 0)
			v1 -= dvb * contact->m_weights[1];

		if (im2 > 0)
			v2 -= dvb * contact->m_weights[2];
	}

	virtual void setPenetrationScale(btScalar scale) {}
};
#endif /* BT_DEFORMABLE_CONTACT_CONSTRAINT_H */
