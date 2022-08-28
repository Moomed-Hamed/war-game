/*
	Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
	Bullet Continuous Collision Detection and Physics Library
	Copyright (c) 2019 Google Inc. http://bulletphysics.org
*/

#ifndef BT_CONTACT_PROJECTION_H
#define BT_CONTACT_PROJECTION_H

#include "btDeformableContactConstraint.h"

struct LagrangeMultiplier
{
	int m_num_constraints; // Number of constraints
	int m_num_nodes;       // Number of nodes in these constraints
	btScalar m_weights[3]; // weights of the nodes involved, same size as m_num_nodes
	btVector3 m_dirs[3];   // Constraint directions, same size of m_num_constraints;
	int m_indices[3];      // indices of the nodes involved, same size as m_num_nodes;
};

class btDeformableContactProjection
{
public:
	typedef btAlignedObjectArray<btVector3> TVStack;
	btAlignedObjectArray<btSoftBody*>& m_softBodies;

	// all constraints involving face
	btAlignedObjectArray<btDeformableContactConstraint*> m_allFaceConstraints;

	// map from node index to projection directions
	btHashMap<btHashInt, btAlignedObjectArray<btVector3> > m_projectionsDict;

	btAlignedObjectArray<LagrangeMultiplier> m_lagrangeMultipliers;

	// map from node index to static constraint
	btAlignedObjectArray<btAlignedObjectArray<btDeformableStaticConstraint> > m_staticConstraints;
	// map from node index to node rigid constraint
	btAlignedObjectArray<btAlignedObjectArray<btDeformableNodeRigidContactConstraint> > m_nodeRigidConstraints;
	// map from node index to face rigid constraint
	btAlignedObjectArray<btAlignedObjectArray<btDeformableFaceRigidContactConstraint> > m_faceRigidConstraints;
	// map from node index to deformable constraint
	btAlignedObjectArray<btAlignedObjectArray<btDeformableFaceNodeContactConstraint> > m_deformableConstraints;
	// map from node index to node anchor constraint
	btAlignedObjectArray<btAlignedObjectArray<btDeformableNodeAnchorConstraint> > m_nodeAnchorConstraints;

	bool m_useStrainLimiting;

	btDeformableContactProjection(btAlignedObjectArray<btSoftBody*>& softBodies)
		: m_softBodies(softBodies)
	{
	}

	virtual ~btDeformableContactProjection()
	{
	}

	// apply the constraints to the rhs of the linear solve
	virtual void project(TVStack& x)
	{
		const int dim = 3;
		for (int index = 0; index < m_projectionsDict.size(); ++index)
		{
			btAlignedObjectArray<btVector3>& projectionDirs = *m_projectionsDict.getAtIndex(index);
			size_t i = m_projectionsDict.getKeyAtIndex(index).getUid1();
			if (projectionDirs.size() >= dim)
			{
				// static node
				x[i].setZero();
				continue;
			}
			else if (projectionDirs.size() == 2)
			{
				btVector3 dir0 = projectionDirs[0];
				btVector3 dir1 = projectionDirs[1];
				btVector3 free_dir = btCross(dir0, dir1);
				if (free_dir.safeNorm() < SIMD_EPSILON)
				{
					x[i] -= x[i].dot(dir0) * dir0;
				}
				else
				{
					free_dir.normalize();
					x[i] = x[i].dot(free_dir) * free_dir;
				}
			}
			else
			{
				btAssert(projectionDirs.size() == 1);
				btVector3 dir0 = projectionDirs[0];
				x[i] -= x[i].dot(dir0) * dir0;
			}
		}
	}

	// add friction force to the rhs of the linear solve
	virtual void applyDynamicFriction(TVStack& f)
	{
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			for (int j = 0; j < m_nodeRigidConstraints[i].size(); ++j)
			{
				const btDeformableNodeRigidContactConstraint& constraint = m_nodeRigidConstraints[i][j];
				const btSoftBody::Node* node = constraint.m_node;
				if (node->m_im != 0)
				{
					int index = node->index;
					f[index] += constraint.getDv(node) * (1. / node->m_im);
				}
			}
			for (int j = 0; j < m_faceRigidConstraints[i].size(); ++j)
			{
				const btDeformableFaceRigidContactConstraint& constraint = m_faceRigidConstraints[i][j];
				const btSoftBody::Face* face = constraint.getContact()->m_face;
				for (int k = 0; k < 3; ++k)
				{
					const btSoftBody::Node* node = face->m_n[k];
					if (node->m_im != 0)
					{
						int index = node->index;
						f[index] += constraint.getDv(node) * (1. / node->m_im);
					}
				}
			}
			for (int j = 0; j < m_deformableConstraints[i].size(); ++j)
			{
				const btDeformableFaceNodeContactConstraint& constraint = m_deformableConstraints[i][j];
				const btSoftBody::Face* face = constraint.getContact()->m_face;
				const btSoftBody::Node* node = constraint.getContact()->m_node;
				if (node->m_im != 0)
				{
					int index = node->index;
					f[index] += constraint.getDv(node) * (1. / node->m_im);
				}
				for (int k = 0; k < 3; ++k)
				{
					const btSoftBody::Node* node = face->m_n[k];
					if (node->m_im != 0)
					{
						int index = node->index;
						f[index] += constraint.getDv(node) * (1. / node->m_im);
					}
				}
			}
		}
	}

	// update and solve the constraints
	virtual btScalar update(btCollisionObject** deformableBodies, int numDeformableBodies, const btContactSolverInfo& infoGlobal)
	{
		btScalar residualSquare = 0;
		for (int i = 0; i < numDeformableBodies; ++i)
		{
			for (int j = 0; j < m_softBodies.size(); ++j)
			{
				btCollisionObject* psb = m_softBodies[j];
				if (psb != deformableBodies[i])
				{
					continue;
				}
				for (int k = 0; k < m_nodeRigidConstraints[j].size(); ++k)
				{
					btDeformableNodeRigidContactConstraint& constraint = m_nodeRigidConstraints[j][k];
					btScalar localResidualSquare = constraint.solveConstraint(infoGlobal);
					residualSquare = btMax(residualSquare, localResidualSquare);
				}
				for (int k = 0; k < m_nodeAnchorConstraints[j].size(); ++k)
				{
					btDeformableNodeAnchorConstraint& constraint = m_nodeAnchorConstraints[j][k];
					btScalar localResidualSquare = constraint.solveConstraint(infoGlobal);
					residualSquare = btMax(residualSquare, localResidualSquare);
				}
				for (int k = 0; k < m_faceRigidConstraints[j].size(); ++k)
				{
					btDeformableFaceRigidContactConstraint& constraint = m_faceRigidConstraints[j][k];
					btScalar localResidualSquare = constraint.solveConstraint(infoGlobal);
					residualSquare = btMax(residualSquare, localResidualSquare);
				}
				for (int k = 0; k < m_deformableConstraints[j].size(); ++k)
				{
					btDeformableFaceNodeContactConstraint& constraint = m_deformableConstraints[j][k];
					btScalar localResidualSquare = constraint.solveConstraint(infoGlobal);
					residualSquare = btMax(residualSquare, localResidualSquare);
				}
			}
		}
		return residualSquare;
	}

	// Add constraints to m_constraints. In addition, the constraints that each vertex own are recorded in m_constraintsDict.
	virtual void setConstraints(const btContactSolverInfo& infoGlobal)
	{
		//BT_PROFILE("setConstraints");
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			if (!psb->isActive())
			{
				continue;
			}

			// set Dirichlet constraint
			for (int j = 0; j < psb->m_nodes.size(); ++j)
			{
				if (psb->m_nodes[j].m_im == 0)
				{
					btDeformableStaticConstraint static_constraint(&psb->m_nodes[j], infoGlobal);
					m_staticConstraints[i].push_back(static_constraint);
				}
			}

			// set up deformable anchors
			for (int j = 0; j < psb->m_deformableAnchors.size(); ++j)
			{
				btSoftBody::DeformableNodeRigidAnchor& anchor = psb->m_deformableAnchors[j];
				// skip fixed points
				if (anchor.m_node->m_im == 0)
				{
					continue;
				}
				anchor.m_c1 = anchor.m_cti.m_colObj->getWorldTransform().getBasis() * anchor.m_local;
				btDeformableNodeAnchorConstraint constraint(anchor, infoGlobal);
				m_nodeAnchorConstraints[i].push_back(constraint);
			}

			// set Deformable Node vs. Rigid constraint
			for (int j = 0; j < psb->m_nodeRigidContacts.size(); ++j)
			{
				const btSoftBody::DeformableNodeRigidContact& contact = psb->m_nodeRigidContacts[j];
				// skip fixed points
				if (contact.m_node->m_im == 0)
				{
					continue;
				}
				btDeformableNodeRigidContactConstraint constraint(contact, infoGlobal);
				m_nodeRigidConstraints[i].push_back(constraint);
			}

			// set Deformable Face vs. Rigid constraint
			for (int j = 0; j < psb->m_faceRigidContacts.size(); ++j)
			{
				const btSoftBody::DeformableFaceRigidContact& contact = psb->m_faceRigidContacts[j];
				// skip fixed faces
				if (contact.m_c2 == 0)
				{
					continue;
				}
				btDeformableFaceRigidContactConstraint constraint(contact, infoGlobal, m_useStrainLimiting);
				m_faceRigidConstraints[i].push_back(constraint);
			}
		}
	}

	// Set up projections for each vertex by adding the projection direction to
	virtual void setProjection()
	{
		//BT_PROFILE("btDeformableContactProjection::setProjection");
		btAlignedObjectArray<btVector3> units;
		units.push_back(btVector3(1, 0, 0));
		units.push_back(btVector3(0, 1, 0));
		units.push_back(btVector3(0, 0, 1));
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			if (!psb->isActive())
			{
				continue;
			}
			for (int j = 0; j < m_staticConstraints[i].size(); ++j)
			{
				int index = m_staticConstraints[i][j].m_node->index;
				m_staticConstraints[i][j].m_node->m_constrained = true;
				if (m_projectionsDict.find(index) == NULL)
				{
					m_projectionsDict.insert(index, units);
				}
				else
				{
					btAlignedObjectArray<btVector3>& projections = *m_projectionsDict[index];
					for (int k = 0; k < 3; ++k)
					{
						projections.push_back(units[k]);
					}
				}
			}
			for (int j = 0; j < m_nodeAnchorConstraints[i].size(); ++j)
			{
				int index = m_nodeAnchorConstraints[i][j].m_anchor->m_node->index;
				m_nodeAnchorConstraints[i][j].m_anchor->m_node->m_constrained = true;
				if (m_projectionsDict.find(index) == NULL)
				{
					m_projectionsDict.insert(index, units);
				}
				else
				{
					btAlignedObjectArray<btVector3>& projections = *m_projectionsDict[index];
					for (int k = 0; k < 3; ++k)
					{
						projections.push_back(units[k]);
					}
				}
			}
			for (int j = 0; j < m_nodeRigidConstraints[i].size(); ++j)
			{
				int index = m_nodeRigidConstraints[i][j].m_node->index;
				m_nodeRigidConstraints[i][j].m_node->m_constrained = true;
				if (m_nodeRigidConstraints[i][j].m_binding)
				{
					if (m_nodeRigidConstraints[i][j].m_static)
					{
						if (m_projectionsDict.find(index) == NULL)
						{
							m_projectionsDict.insert(index, units);
						}
						else
						{
							btAlignedObjectArray<btVector3>& projections = *m_projectionsDict[index];
							for (int k = 0; k < 3; ++k)
							{
								projections.push_back(units[k]);
							}
						}
					}
					else
					{
						if (m_projectionsDict.find(index) == NULL)
						{
							btAlignedObjectArray<btVector3> projections;
							projections.push_back(m_nodeRigidConstraints[i][j].m_normal);
							m_projectionsDict.insert(index, projections);
						}
						else
						{
							btAlignedObjectArray<btVector3>& projections = *m_projectionsDict[index];
							projections.push_back(m_nodeRigidConstraints[i][j].m_normal);
						}
					}
				}
			}
			for (int j = 0; j < m_faceRigidConstraints[i].size(); ++j)
			{
				const btSoftBody::Face* face = m_faceRigidConstraints[i][j].m_face;
				if (m_faceRigidConstraints[i][j].m_binding)
				{
					for (int k = 0; k < 3; ++k)
					{
						face->m_n[k]->m_constrained = true;
					}
				}
				for (int k = 0; k < 3; ++k)
				{
					btSoftBody::Node* node = face->m_n[k];
					int index = node->index;
					if (m_faceRigidConstraints[i][j].m_static)
					{
						if (m_projectionsDict.find(index) == NULL)
						{
							m_projectionsDict.insert(index, units);
						}
						else
						{
							btAlignedObjectArray<btVector3>& projections = *m_projectionsDict[index];
							for (int l = 0; l < 3; ++l)
							{
								projections.push_back(units[l]);
							}
						}
					}
					else
					{
						if (m_projectionsDict.find(index) == NULL)
						{
							btAlignedObjectArray<btVector3> projections;
							projections.push_back(m_faceRigidConstraints[i][j].m_normal);
							m_projectionsDict.insert(index, projections);
						}
						else
						{
							btAlignedObjectArray<btVector3>& projections = *m_projectionsDict[index];
							projections.push_back(m_faceRigidConstraints[i][j].m_normal);
						}
					}
				}
			}
		}
	}

	virtual void reinitialize(bool nodeUpdated)
	{
		int N = m_softBodies.size();
		if (nodeUpdated)
		{
			m_staticConstraints.resize(N);
			m_nodeAnchorConstraints.resize(N);
			m_nodeRigidConstraints.resize(N);
			m_faceRigidConstraints.resize(N);
			m_deformableConstraints.resize(N);
		}
		for (int i = 0; i < N; ++i)
		{
			m_staticConstraints[i].clear();
			m_nodeAnchorConstraints[i].clear();
			m_nodeRigidConstraints[i].clear();
			m_faceRigidConstraints[i].clear();
			m_deformableConstraints[i].clear();
		}
#ifndef USE_MGS
		m_projectionsDict.clear();
#else
		m_projections.clear();
#endif
		m_lagrangeMultipliers.clear();
	}

	btScalar solveSplitImpulse(btCollisionObject** deformableBodies, int numDeformableBodies, const btContactSolverInfo& infoGlobal)
	{
		btScalar residualSquare = 0;
		for (int i = 0; i < numDeformableBodies; ++i)
		{
			for (int j = 0; j < m_softBodies.size(); ++j)
			{
				btCollisionObject* psb = m_softBodies[j];
				if (psb != deformableBodies[i])
				{
					continue;
				}
				for (int k = 0; k < m_nodeRigidConstraints[j].size(); ++k)
				{
					btDeformableNodeRigidContactConstraint& constraint = m_nodeRigidConstraints[j][k];
					btScalar localResidualSquare = constraint.solveSplitImpulse(infoGlobal);
					residualSquare = btMax(residualSquare, localResidualSquare);
				}
				for (int k = 0; k < m_faceRigidConstraints[j].size(); ++k)
				{
					btDeformableFaceRigidContactConstraint& constraint = m_faceRigidConstraints[j][k];
					btScalar localResidualSquare = constraint.solveSplitImpulse(infoGlobal);
					residualSquare = btMax(residualSquare, localResidualSquare);
				}
			}
		}
		return residualSquare;
	}

	virtual void setLagrangeMultiplier()
	{
		for (int i = 0; i < m_softBodies.size(); ++i)
		{
			btSoftBody* psb = m_softBodies[i];
			if (!psb->isActive())
			{
				continue;
			}
			for (int j = 0; j < m_staticConstraints[i].size(); ++j)
			{
				int index = m_staticConstraints[i][j].m_node->index;
				m_staticConstraints[i][j].m_node->m_constrained = true;
				LagrangeMultiplier lm;
				lm.m_num_nodes = 1;
				lm.m_indices[0] = index;
				lm.m_weights[0] = 1.0;
				lm.m_num_constraints = 3;
				lm.m_dirs[0] = btVector3(1, 0, 0);
				lm.m_dirs[1] = btVector3(0, 1, 0);
				lm.m_dirs[2] = btVector3(0, 0, 1);
				m_lagrangeMultipliers.push_back(lm);
			}
			for (int j = 0; j < m_nodeAnchorConstraints[i].size(); ++j)
			{
				int index = m_nodeAnchorConstraints[i][j].m_anchor->m_node->index;
				m_nodeAnchorConstraints[i][j].m_anchor->m_node->m_constrained = true;
				LagrangeMultiplier lm;
				lm.m_num_nodes = 1;
				lm.m_indices[0] = index;
				lm.m_weights[0] = 1.0;
				lm.m_num_constraints = 3;
				lm.m_dirs[0] = btVector3(1, 0, 0);
				lm.m_dirs[1] = btVector3(0, 1, 0);
				lm.m_dirs[2] = btVector3(0, 0, 1);
				m_lagrangeMultipliers.push_back(lm);
			}

			for (int j = 0; j < m_nodeRigidConstraints[i].size(); ++j)
			{
				if (!m_nodeRigidConstraints[i][j].m_binding)
				{
					continue;
				}
				int index = m_nodeRigidConstraints[i][j].m_node->index;
				m_nodeRigidConstraints[i][j].m_node->m_constrained = true;
				LagrangeMultiplier lm;
				lm.m_num_nodes = 1;
				lm.m_indices[0] = index;
				lm.m_weights[0] = 1.0;
				if (m_nodeRigidConstraints[i][j].m_static)
				{
					lm.m_num_constraints = 3;
					lm.m_dirs[0] = btVector3(1, 0, 0);
					lm.m_dirs[1] = btVector3(0, 1, 0);
					lm.m_dirs[2] = btVector3(0, 0, 1);
				}
				else
				{
					lm.m_num_constraints = 1;
					lm.m_dirs[0] = m_nodeRigidConstraints[i][j].m_normal;
				}
				m_lagrangeMultipliers.push_back(lm);
			}

			for (int j = 0; j < m_faceRigidConstraints[i].size(); ++j)
			{
				if (!m_faceRigidConstraints[i][j].m_binding)
				{
					continue;
				}
				btSoftBody::Face* face = m_faceRigidConstraints[i][j].m_face;

				btVector3 bary = m_faceRigidConstraints[i][j].getContact()->m_bary;
				LagrangeMultiplier lm;
				lm.m_num_nodes = 3;

				for (int k = 0; k < 3; ++k)
				{
					face->m_n[k]->m_constrained = true;
					lm.m_indices[k] = face->m_n[k]->index;
					lm.m_weights[k] = bary[k];
				}
				if (m_faceRigidConstraints[i][j].m_static)
				{
					face->m_pcontact[3] = 1;
					lm.m_num_constraints = 3;
					lm.m_dirs[0] = btVector3(1, 0, 0);
					lm.m_dirs[1] = btVector3(0, 1, 0);
					lm.m_dirs[2] = btVector3(0, 0, 1);
				}
				else
				{
					face->m_pcontact[3] = 0;
					lm.m_num_constraints = 1;
					lm.m_dirs[0] = m_faceRigidConstraints[i][j].m_normal;
				}
				m_lagrangeMultipliers.push_back(lm);
			}
		}
	}

	void checkConstraints(const TVStack& x)
	{
		for (int i = 0; i < m_lagrangeMultipliers.size(); ++i)
		{
			btVector3 d(0, 0, 0);
			const LagrangeMultiplier& lm = m_lagrangeMultipliers[i];
			for (int j = 0; j < lm.m_num_constraints; ++j)
			{
				for (int k = 0; k < lm.m_num_nodes; ++k)
				{
					d[j] += lm.m_weights[k] * x[lm.m_indices[k]].dot(lm.m_dirs[j]);
				}
			}
			// printf("d = %f, %f, %f\n", d[0], d[1], d[2]);
			// printf("val = %f, %f, %f\n", lm.m_vals[0], lm.m_vals[1], lm.m_vals[2]);
		}
	}
};
#endif /* btDeformableContactProjection_h */
