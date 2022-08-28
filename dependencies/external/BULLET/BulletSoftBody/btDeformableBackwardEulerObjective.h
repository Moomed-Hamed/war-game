/*
 Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 */

#ifndef BT_BACKWARD_EULER_OBJECTIVE_H
#define BT_BACKWARD_EULER_OBJECTIVE_H

#include "btDeformableLagrangianForce.h"
#include "btDeformableMassSpringForce.h"
#include "btDeformableGravityForce.h"
#include "btDeformableCorotatedForce.h"
#include "btDeformableMousePickingForce.h"
#include "btDeformableLinearElasticityForce.h"
#include "btDeformableNeoHookeanForce.h"
#include "btDeformableContactProjection.h"
#include "btPreconditioner.h"

class btDeformableBackwardEulerObjective
{
public:
	typedef btAlignedObjectArray<btVector3> TVStack;
	btScalar m_dt;
	btAlignedObjectArray<btDeformableLagrangianForce*> m_lf;
	btAlignedObjectArray<btSoftBody*>& m_softBodies;
	Preconditioner* m_preconditioner;
	btDeformableContactProjection m_projection;
	const TVStack& m_backupVelocity;
	btAlignedObjectArray<btSoftBody::Node*> m_nodes;
	bool m_implicit;
	MassPreconditioner* m_massPreconditioner;
	KKTPreconditioner* m_KKTPreconditioner;

	btDeformableBackwardEulerObjective(btAlignedObjectArray<btSoftBody*>& softBodies, const TVStack& backup_v)
		: m_softBodies(softBodies), m_projection(softBodies), m_backupVelocity(backup_v), m_implicit(false)
	{
		m_massPreconditioner = new MassPreconditioner(m_softBodies);
		m_KKTPreconditioner = new KKTPreconditioner(m_softBodies, m_projection, m_lf, m_dt, m_implicit);
		m_preconditioner = m_KKTPreconditioner;
	}

	virtual ~btDeformableBackwardEulerObjective()
	{
		delete m_KKTPreconditioner;
		delete m_massPreconditioner;
	}

	void initialize() {}

	// compute the rhs for CG solve, i.e, add the dt scaled implicit force to residual
	void computeResidual(btScalar dt, TVStack& residual)
	{
		BT_PROFILE("computeResidual");
		// add implicit force
		for (int i = 0; i < m_lf.size(); ++i)
		{
			// Always integrate picking force implicitly for stability.
			if (m_implicit || m_lf[i]->getForceType() == BT_MOUSE_PICKING_FORCE)
			{
				m_lf[i]->addScaledForces(dt, residual);
			}
			else
			{
				m_lf[i]->addScaledDampingForce(dt, residual);
			}
		}
	}

	// add explicit force to the velocity
	void applyExplicitForce(TVStack& force)
	{
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			m_softBodies[i]->advanceDeformation();
		}
		if (m_implicit)
		{
			// apply forces except gravity force
			btVector3 gravity;
			for (int i = 0; i < m_lf.size(); ++i)
			{
				if (m_lf[i]->getForceType() == BT_GRAVITY_FORCE)
				{
					gravity = static_cast<btDeformableGravityForce*>(m_lf[i])->m_gravity;
				}
				else
				{
					m_lf[i]->addScaledForces(m_dt, force);
				}
			}
			for (int i = 0; i < m_lf.size(); ++i)
			{
				m_lf[i]->addScaledHessian(m_dt);
			}
			for (int i = 0; i < m_softBodies.size(); ++i)
			{
				btSoftBody* psb = m_softBodies[i];
				if (psb->isActive())
				{
					for (int j = 0; j < psb->m_nodes.size(); ++j)
					{
						// add gravity explicitly
						psb->m_nodes[j].m_v += m_dt * psb->m_gravityFactor * gravity;
					}
				}
			}
		}
		else
		{
			for (int i = 0; i < m_lf.size(); ++i)
			{
				m_lf[i]->addScaledExplicitForce(m_dt, force);
			}
		}
		// calculate inverse mass matrix for all nodes
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			if (psb->isActive())
			{
				for (int j = 0; j < psb->m_nodes.size(); ++j)
				{
					if (psb->m_nodes[j].m_im > 0)
					{
						psb->m_nodes[j].m_effectiveMass_inv = psb->m_nodes[j].m_effectiveMass.inverse();
					}
				}
			}
		}
		applyForce(force, true);
	}

	// apply force to velocity and optionally reset the force to zero
	void applyForce(TVStack& force, bool setZero)
	{
		size_t counter = 0;
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			if (!psb->isActive())
			{
				counter += psb->m_nodes.size();
				continue;
			}
			if (m_implicit)
			{
				for (int j = 0; j < psb->m_nodes.size(); ++j)
				{
					if (psb->m_nodes[j].m_im != 0)
					{
						psb->m_nodes[j].m_v += psb->m_nodes[j].m_effectiveMass_inv * force[counter++];
					}
				}
			}
			else
			{
				for (int j = 0; j < psb->m_nodes.size(); ++j)
				{
					btScalar one_over_mass = (psb->m_nodes[j].m_im == 0) ? 0 : psb->m_nodes[j].m_im;
					psb->m_nodes[j].m_v += one_over_mass * force[counter++];
				}
			}
		}
		if (setZero)
		{
			for (int i = 0; i < force.size(); ++i)
				force[i].setZero();
		}
	}

	// compute the norm of the residual
	btScalar computeNorm(const TVStack& residual) const
	{
		btScalar mag = 0;
		for (int i = 0; i < residual.size(); ++i)
		{
			mag += residual[i].length2();
		}
		return std::sqrt(mag);
	}

	// compute one step of the solve (there is only one solve if the system is linear)
	void computeStep(TVStack& dv, const TVStack& residual, const btScalar& dt);

	// perform A*x = b
	void multiply(const TVStack& x, TVStack& b) const
	{
		BT_PROFILE("multiply");
		// add in the mass term
		size_t counter = 0;
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				const btSoftBody::Node& node = psb->m_nodes[j];
				b[counter] = (node.m_im == 0) ? btVector3(0, 0, 0) : x[counter] / node.m_im;
				++counter;
			}
		}

		for (int i = 0; i < m_lf.size(); ++i)
		{
			// add damping matrix
			m_lf[i]->addScaledDampingForceDifferential(-m_dt, x, b);
			// Always integrate picking force implicitly for stability.
			if (m_implicit || m_lf[i]->getForceType() == BT_MOUSE_PICKING_FORCE)
			{
				m_lf[i]->addScaledElasticForceDifferential(-m_dt * m_dt, x, b);
			}
		}
		int offset = m_nodes.size();
		for (int i = offset; i < b.size(); ++i)
		{
			b[i].setZero();
		}
		// add in the lagrange multiplier terms

		for (int c = 0; c < m_projection.m_lagrangeMultipliers.size(); ++c)
		{
			// C^T * lambda
			const LagrangeMultiplier& lm = m_projection.m_lagrangeMultipliers[c];
			for (int i = 0; i < lm.m_num_nodes; ++i)
			{
				for (int j = 0; j < lm.m_num_constraints; ++j)
				{
					b[lm.m_indices[i]] += x[offset + c][j] * lm.m_weights[i] * lm.m_dirs[j];
				}
			}
			// C * x
			for (int d = 0; d < lm.m_num_constraints; ++d)
			{
				for (int i = 0; i < lm.m_num_nodes; ++i)
				{
					b[offset + c][d] += lm.m_weights[i] * x[lm.m_indices[i]].dot(lm.m_dirs[d]);
				}
			}
		}
	}

	// set initial guess for CG solve
	void initialGuess(TVStack& dv, const TVStack& residual)
	{
		size_t counter = 0;
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				dv[counter] = psb->m_nodes[j].m_im * residual[counter];
				++counter;
			}
		}
	}

	// reset data structure and reset dt
	void reinitialize(bool nodeUpdated, btScalar dt)
	{
		BT_PROFILE("reinitialize");
		if (dt > 0)
		{
			setDt(dt);
		}
		if (nodeUpdated)
		{
			updateId();
		}
		for (int i = 0; i < m_lf.size(); ++i)
		{
			m_lf[i]->reinitialize(nodeUpdated);
		}
		btMatrix3x3 I;
		I.setIdentity();
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				if (psb->m_nodes[j].m_im > 0)
					psb->m_nodes[j].m_effectiveMass = I * (1.0 / psb->m_nodes[j].m_im);
			}
		}
		m_projection.reinitialize(nodeUpdated);
	}

	void setDt(btScalar dt)
	{
		m_dt = dt;
	}

	// add friction force to residual
	void applyDynamicFriction(TVStack& r)
	{
		m_projection.applyDynamicFriction(r);
	}

	// add dv to velocity
	void updateVelocity(const TVStack& dv)
	{
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				btSoftBody::Node& node = psb->m_nodes[j];
				node.m_v = m_backupVelocity[node.index] + dv[node.index];
			}
		}
	}

	//set constraints as projections
	void setConstraints(const btContactSolverInfo& infoGlobal)
	{
		m_projection.setConstraints(infoGlobal);
	}

	// update the projections and project the residual
	void project(TVStack& r)
	{
		BT_PROFILE("project");
		m_projection.project(r);
	}

	// perform precondition M^(-1) x = b
	void precondition(const TVStack& x, TVStack& b)
	{
		m_preconditioner->operator()(x, b);
	}

	// reindex all the vertices
	virtual void updateId()
	{
		size_t node_id = 0;
		size_t face_id = 0;
		m_nodes.clear();
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				psb->m_nodes[j].index = node_id;
				m_nodes.push_back(&psb->m_nodes[j]);
				++node_id;
			}
			for (int j = 0; j < psb->m_faces.size(); ++j)
			{
				psb->m_faces[j].m_index = face_id;
				++face_id;
			}
		}
	}

	const btAlignedObjectArray<btSoftBody::Node*>* getIndices() const
	{
		return &m_nodes;
	}

	void setImplicit(bool implicit)
	{
		m_implicit = implicit;
	}

	// Calculate the total potential energy in the system
	btScalar totalEnergy(btScalar dt)
	{
		btScalar e = 0;
		for (int i = 0; i < m_lf.size(); ++i)
		{
			e += m_lf[i]->totalEnergy(dt);
		}
		return e;
	}

	void addLagrangeMultiplier(const TVStack& vec, TVStack& extended_vec)
	{
		extended_vec.resize(vec.size() + m_projection.m_lagrangeMultipliers.size());
		for (int i = 0; i < vec.size(); ++i)
		{
			extended_vec[i] = vec[i];
		}
		int offset = vec.size();
		for (int i = 0; i < m_projection.m_lagrangeMultipliers.size(); ++i)
		{
			extended_vec[offset + i].setZero();
		}
	}

	void addLagrangeMultiplierRHS(const TVStack& residual, const TVStack& m_dv, TVStack& extended_residual)
	{
		extended_residual.resize(residual.size() + m_projection.m_lagrangeMultipliers.size());
		for (int i = 0; i < residual.size(); ++i)
		{
			extended_residual[i] = residual[i];
		}
		int offset = residual.size();
		for (int i = 0; i < m_projection.m_lagrangeMultipliers.size(); ++i)
		{
			const LagrangeMultiplier& lm = m_projection.m_lagrangeMultipliers[i];
			extended_residual[offset + i].setZero();
			for (int d = 0; d < lm.m_num_constraints; ++d)
			{
				for (int n = 0; n < lm.m_num_nodes; ++n)
				{
					extended_residual[offset + i][d] += lm.m_weights[n] * m_dv[lm.m_indices[n]].dot(lm.m_dirs[d]);
				}
			}
		}
	}

	void calculateContactForce(const TVStack& dv, const TVStack& rhs, TVStack& f)
	{
		size_t counter = 0;
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				const btSoftBody::Node& node = psb->m_nodes[j];
				f[counter] = (node.m_im == 0) ? btVector3(0, 0, 0) : dv[counter] / node.m_im;
				++counter;
			}
		}
		for (int i = 0; i < m_lf.size(); ++i)
		{
			// add damping matrix
			m_lf[i]->addScaledDampingForceDifferential(-m_dt, dv, f);
		}
		counter = 0;
		for (; counter < f.size(); ++counter)
		{
			f[counter] = rhs[counter] - f[counter];
		}
	}
};

#endif /* btBackwardEulerObjective_h */
