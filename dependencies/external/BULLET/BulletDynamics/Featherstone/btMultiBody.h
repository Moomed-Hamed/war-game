/* PURPOSE:
 *   Class representing an articulated rigid body. Stores the body's
 *   current state, allows forces and torques to be set, handles
 *   timestepping and implements Featherstone's algorithm.
 *   
 * COPYRIGHT:
 *   Copyright (C) Stephen Thompson, <stephen@solarflare.org.uk>, 2011-2013
 *   Portions written By Erwin Coumans: connection to LCP solver, various multibody constraints, replacing Eigen math library by Bullet LinearMath and a dedicated 6x6 matrix inverse (solveImatrix)
 *   Portions written By Jakub Stepien: support for multi-DOF constraints, introduction of spatial algebra and several other improvements
 */

#ifndef BT_MULTIBODY_H
#define BT_MULTIBODY_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btAlignedObjectArray.h"

// serialization data, don't change if you aren't familiar with the details of the serialization mechanisms
#define btMultiBodyData btMultiBodyFloatData
#define btMultiBodyDataName "btMultiBodyFloatData"
#define btMultiBodyLinkData btMultiBodyLinkFloatData
#define btMultiBodyLinkDataName "btMultiBodyLinkFloatData"

#include "btMultiBodyLink.h"
class btMultiBodyLinkCollider;

static const btScalar INITIAL_SLEEP_EPSILON = btScalar(0.05);  // this is a squared velocity (m^2 s^-2)
static const btScalar INITIAL_SLEEP_TIMEOUT = btScalar(2);     // in seconds

ATTRIBUTE_ALIGNED16(class)
btMultiBody
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

// initialization

	btMultiBody(int n_links,           // NOT including the base
				btScalar mass,            // mass of base
				const btVector3 &inertia, // inertia of base, in base frame; assumed diagonal
				bool fixedBase,           // whether the base is fixed (true) or can move (false)
				bool canSleep, bool deprecatedMultiDof = true)
		: m_baseCollider(0),
		  m_baseName(0),
		  m_basePos(0, 0, 0),
		  m_baseQuat(0, 0, 0, 1),
		  m_basePos_interpolate(0, 0, 0),
		  m_baseQuat_interpolate(0, 0, 0, 1),
		  m_baseMass(mass),
		  m_baseInertia(inertia),

		  m_fixedBase(fixedBase),
		  m_awake(true),
		  m_canSleep(canSleep),
		  m_canWakeup(true),
		  m_sleepTimer(0),
		  m_sleepEpsilon(INITIAL_SLEEP_EPSILON),
		  m_sleepTimeout(INITIAL_SLEEP_TIMEOUT),

		  m_userObjectPointer(0),
		  m_userIndex2(-1),
		  m_userIndex(-1),
		  m_companionId(-1),
		  m_linearDamping(0.04f),
		  m_angularDamping(0.04f),
		  m_useGyroTerm(true),
		  m_maxAppliedImpulse(1000.f),
		  m_maxCoordinateVelocity(100.f),
		  m_hasSelfCollision(true),
		  __posUpdated(false),
		  m_dofCount(0),
		  m_posVarCnt(0),
		  m_useRK4(false),
		  m_useGlobalVelocities(false),
		  m_internalNeedsJointFeedback(false),
		  m_kinematic_calculate_velocity(false)
	{
		m_cachedInertiaTopLeft.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
		m_cachedInertiaTopRight.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
		m_cachedInertiaLowerLeft.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
		m_cachedInertiaLowerRight.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
		m_cachedInertiaValid = false;

		m_links.resize(n_links);
		m_matrixBuf.resize(n_links + 1);

		m_baseForce.setValue(0, 0, 0);
		m_baseTorque.setValue(0, 0, 0);

		clearConstraintForces();
		clearForcesAndTorques();
	}

	virtual ~btMultiBody() {}

	//note: fixed link collision with parent is always disabled
	void setupFixed(int i, //linkIndex
					btScalar mass,
					const btVector3 &inertia,
					int parent,
					const btQuaternion &rotParentToThis,
					const btVector3 &parentComToThisPivotOffset,
					const btVector3 &thisPivotToThisComOffset, bool deprecatedDisableParentCollision = true)
	{
		m_links[i].m_mass = mass;
		m_links[i].m_inertiaLocal = inertia;
		m_links[i].m_parent = parent;
		m_links[i].setAxisTop(0, 0., 0., 0.);
		m_links[i].setAxisBottom(0, btVector3(0, 0, 0));
		m_links[i].m_zeroRotParentToThis = rotParentToThis;
		m_links[i].m_dVector = thisPivotToThisComOffset;
		m_links[i].m_eVector = parentComToThisPivotOffset;

		m_links[i].m_jointType = btMultibodyLink::eFixed;
		m_links[i].m_dofCount = 0;
		m_links[i].m_posVarCount = 0;

		m_links[i].m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;

		m_links[i].updateCacheMultiDof();

		updateLinksDofOffsets();
	}

	void setupPrismatic(int i,
						btScalar mass,
						const btVector3 &inertia,
						int parent,
						const btQuaternion &rotParentToThis,
						const btVector3 &jointAxis,
						const btVector3 &parentComToThisPivotOffset,
						const btVector3 &thisPivotToThisComOffset,
						bool disableParentCollision)
	{
		m_dofCount += 1;
		m_posVarCnt += 1;

		m_links[i].m_mass = mass;
		m_links[i].m_inertiaLocal = inertia;
		m_links[i].m_parent = parent;
		m_links[i].m_zeroRotParentToThis = rotParentToThis;
		m_links[i].setAxisTop(0, 0., 0., 0.);
		m_links[i].setAxisBottom(0, jointAxis);
		m_links[i].m_eVector = parentComToThisPivotOffset;
		m_links[i].m_dVector = thisPivotToThisComOffset;
		m_links[i].m_cachedRotParentToThis = rotParentToThis;

		m_links[i].m_jointType = btMultibodyLink::ePrismatic;
		m_links[i].m_dofCount = 1;
		m_links[i].m_posVarCount = 1;
		m_links[i].m_jointPos[0] = 0.f;
		m_links[i].m_jointTorque[0] = 0.f;

		if (disableParentCollision)
			m_links[i].m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;

		m_links[i].updateCacheMultiDof();

		updateLinksDofOffsets();
	}

	void setupRevolute(int i,  // 0 to num_links-1
					   btScalar mass,
					   const btVector3 &inertia,
					   int parentIndex,
					   const btQuaternion &rotParentToThis,          // rotate points in parent frame to this frame, when q = 0
					   const btVector3 &jointAxis,                   // in my frame
					   const btVector3 &parentComToThisPivotOffset,  // vector from parent COM to joint axis, in PARENT frame
					   const btVector3 &thisPivotToThisComOffset,    // vector from joint axis to my COM, in MY frame
					   bool disableParentCollision = false)
	{
		m_dofCount += 1;
		m_posVarCnt += 1;

		m_links[i].m_mass = mass;
		m_links[i].m_inertiaLocal = inertia;
		m_links[i].m_parent = parentIndex;
		m_links[i].m_zeroRotParentToThis = rotParentToThis;
		m_links[i].setAxisTop(0, jointAxis);
		m_links[i].setAxisBottom(0, jointAxis.cross(thisPivotToThisComOffset));
		m_links[i].m_dVector = thisPivotToThisComOffset;
		m_links[i].m_eVector = parentComToThisPivotOffset;

		m_links[i].m_jointType = btMultibodyLink::eRevolute;
		m_links[i].m_dofCount = 1;
		m_links[i].m_posVarCount = 1;
		m_links[i].m_jointPos[0] = 0.f;
		m_links[i].m_jointTorque[0] = 0.f;

		if (disableParentCollision)
			m_links[i].m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;

		m_links[i].updateCacheMultiDof();

		updateLinksDofOffsets();
	}

	void setupSpherical(int i,  // linkIndex, 0 to num_links-1
						btScalar mass,
						const btVector3 &inertia,
						int parent,
						const btQuaternion &rotParentToThis,          // rotate points in parent frame to this frame, when q = 0
						const btVector3 &parentComToThisPivotOffset,  // vector from parent COM to joint axis, in PARENT frame
						const btVector3 &thisPivotToThisComOffset,    // vector from joint axis to my COM, in MY frame
						bool disableParentCollision = false)
	{
		m_dofCount += 3;
		m_posVarCnt += 4;

		m_links[i].m_mass = mass;
		m_links[i].m_inertiaLocal = inertia;
		m_links[i].m_parent = parent;
		m_links[i].m_zeroRotParentToThis = rotParentToThis;
		m_links[i].m_dVector = thisPivotToThisComOffset;
		m_links[i].m_eVector = parentComToThisPivotOffset;

		m_links[i].m_jointType = btMultibodyLink::eSpherical;
		m_links[i].m_dofCount = 3;
		m_links[i].m_posVarCount = 4;
		m_links[i].setAxisTop(0, 1.f, 0.f, 0.f);
		m_links[i].setAxisTop(1, 0.f, 1.f, 0.f);
		m_links[i].setAxisTop(2, 0.f, 0.f, 1.f);
		m_links[i].setAxisBottom(0, m_links[i].getAxisTop(0).cross(thisPivotToThisComOffset));
		m_links[i].setAxisBottom(1, m_links[i].getAxisTop(1).cross(thisPivotToThisComOffset));
		m_links[i].setAxisBottom(2, m_links[i].getAxisTop(2).cross(thisPivotToThisComOffset));
		m_links[i].m_jointPos[0] = m_links[i].m_jointPos[1] = m_links[i].m_jointPos[2] = 0.f;
		m_links[i].m_jointPos[3] = 1.f;
		m_links[i].m_jointTorque[0] = m_links[i].m_jointTorque[1] = m_links[i].m_jointTorque[2] = 0.f;

		if (disableParentCollision)
			m_links[i].m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;

		m_links[i].updateCacheMultiDof();

		updateLinksDofOffsets();
	}

	void setupPlanar(int i,  // 0 to num_links-1
					 btScalar mass,
					 const btVector3 &inertia,
					 int parent,
					 const btQuaternion &rotParentToThis,  // rotate points in parent frame to this frame, when q = 0
					 const btVector3 &rotationAxis,
					 const btVector3 &parentComToThisComOffset,  // vector from parent COM to this COM, in PARENT frame
					 bool disableParentCollision = false)
	{
		m_dofCount += 3;
		m_posVarCnt += 3;

		m_links[i].m_mass = mass;
		m_links[i].m_inertiaLocal = inertia;
		m_links[i].m_parent = parent;
		m_links[i].m_zeroRotParentToThis = rotParentToThis;
		m_links[i].m_dVector.setZero();
		m_links[i].m_eVector = parentComToThisComOffset;

		btVector3 vecNonParallelToRotAxis(1, 0, 0);
		if (rotationAxis.normalized().dot(vecNonParallelToRotAxis) > 0.999)
			vecNonParallelToRotAxis.setValue(0, 1, 0);

		m_links[i].m_jointType = btMultibodyLink::ePlanar;
		m_links[i].m_dofCount = 3;
		m_links[i].m_posVarCount = 3;
		btVector3 n = rotationAxis.normalized();
		m_links[i].setAxisTop(0, n[0], n[1], n[2]);
		m_links[i].setAxisTop(1, 0, 0, 0);
		m_links[i].setAxisTop(2, 0, 0, 0);
		m_links[i].setAxisBottom(0, 0, 0, 0);
		btVector3 cr = m_links[i].getAxisTop(0).cross(vecNonParallelToRotAxis);
		m_links[i].setAxisBottom(1, cr[0], cr[1], cr[2]);
		cr = m_links[i].getAxisBottom(1).cross(m_links[i].getAxisTop(0));
		m_links[i].setAxisBottom(2, cr[0], cr[1], cr[2]);
		m_links[i].m_jointPos[0] = m_links[i].m_jointPos[1] = m_links[i].m_jointPos[2] = 0.f;
		m_links[i].m_jointTorque[0] = m_links[i].m_jointTorque[1] = m_links[i].m_jointTorque[2] = 0.f;

		if (disableParentCollision)
			m_links[i].m_flags |= BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;

		m_links[i].updateCacheMultiDof();

		updateLinksDofOffsets();

		m_links[i].setAxisBottom(1, m_links[i].getAxisBottom(1).normalized());
		m_links[i].setAxisBottom(2, m_links[i].getAxisBottom(2).normalized());
	}

	const btMultibodyLink &getLink(int index) const
	{
		return m_links[index];
	}

	btMultibodyLink &getLink(int index)
	{
		return m_links[index];
	}

	void setBaseCollider(btMultiBodyLinkCollider * collider)  //collider can be NULL to disable collision for the base
	{
		m_baseCollider = collider;
	}
	const btMultiBodyLinkCollider *getBaseCollider() const
	{
		return m_baseCollider;
	}
	btMultiBodyLinkCollider *getBaseCollider()
	{
		return m_baseCollider;
	}

	const btMultiBodyLinkCollider *getLinkCollider(int index) const
	{
		if (index >= 0 && index < getNumLinks())
		{
			return getLink(index).m_collider;
		}
		return 0;
	}

	btMultiBodyLinkCollider *getLinkCollider(int index)
	{
		if (index >= 0 && index < getNumLinks())
		{
			return getLink(index).m_collider;
		}
		return 0;
	}

	// get parent
	// input: link num from 0 to num_links-1
	// output: link num from 0 to num_links-1, OR -1 to mean the base.
	int getParent(int link_num) const;

	// get number of m_links, masses, moments of inertia

	int getNumLinks() const { return m_links.size(); }
	int getNumDofs() const { return m_dofCount; }
	int getNumPosVars() const { return m_posVarCnt; }
	btScalar getBaseMass() const { return m_baseMass; }
	const btVector3 &getBaseInertia() const { return m_baseInertia; }
	btScalar getLinkMass(int i) const;
	const btVector3 &getLinkInertia(int i) const;

	// change mass (incomplete: can only change base mass and inertia at present)

	void setBaseMass(btScalar mass) { m_baseMass = mass; }
	void setBaseInertia(const btVector3 &inertia) { m_baseInertia = inertia; }

	// get/set pos/vel/rot/omega for the base link

	const btVector3 &getBasePos() const 
	{ 
		return m_basePos; 
	}  // in world frame
	const btVector3 getBaseVel() const
	{
		return btVector3(m_realBuf[3], m_realBuf[4], m_realBuf[5]);
	}  // in world frame
	const btQuaternion &getWorldToBaseRot() const
	{
		return m_baseQuat;
	}
    
    const btVector3 &getInterpolateBasePos() const
    {
        return m_basePos_interpolate;
    }  // in world frame
    const btQuaternion &getInterpolateWorldToBaseRot() const
    {
        return m_baseQuat_interpolate;
    }
    
    // rotates world vectors into base frame
	btVector3 getBaseOmega() const { return btVector3(m_realBuf[0], m_realBuf[1], m_realBuf[2]); }  // in world frame

	void setBasePos(const btVector3 &pos)
	{
		m_basePos = pos;
		if(!isBaseKinematic())
			m_basePos_interpolate = pos;
	}

	void setInterpolateBasePos(const btVector3 &pos)
	{
		m_basePos_interpolate = pos;
	}

	void setBaseWorldTransform(const btTransform &tr)
	{
		setBasePos(tr.getOrigin());
		setWorldToBaseRot(tr.getRotation().inverse());
	}

	btTransform getBaseWorldTransform() const
	{
		btTransform tr;
		tr.setOrigin(getBasePos());
		tr.setRotation(getWorldToBaseRot().inverse());
		return tr;
	}

	void setInterpolateBaseWorldTransform(const btTransform &tr)
	{
		setInterpolateBasePos(tr.getOrigin());
		setInterpolateWorldToBaseRot(tr.getRotation().inverse());
	}

	btTransform getInterpolateBaseWorldTransform() const
	{
		btTransform tr;
		tr.setOrigin(getInterpolateBasePos());
		tr.setRotation(getInterpolateWorldToBaseRot().inverse());
		return tr;
	}

	void setBaseVel(const btVector3 &vel)
	{
		m_realBuf[3] = vel[0];
		m_realBuf[4] = vel[1];
		m_realBuf[5] = vel[2];
	}

	void setWorldToBaseRot(const btQuaternion &rot)
	{
		m_baseQuat = rot;  //m_baseQuat asumed to ba alias!?
		if(!isBaseKinematic())
			m_baseQuat_interpolate = rot;
	}

	void setInterpolateWorldToBaseRot(const btQuaternion &rot)
	{
		m_baseQuat_interpolate = rot;
	}

	void setBaseOmega(const btVector3 &omega)
	{
		m_realBuf[0] = omega[0];
		m_realBuf[1] = omega[1];
		m_realBuf[2] = omega[2];
	}

	void saveKinematicState(btScalar timeStep);

	// get/set pos/vel for child m_links (i = 0 to num_links-1)

	btScalar getJointPos(int i) const;
	btScalar getJointVel(int i) const;

	btScalar *getJointVelMultiDof(int i);
	btScalar *getJointPosMultiDof(int i);

	const btScalar *getJointVelMultiDof(int i) const;
	const btScalar *getJointPosMultiDof(int i) const;

	void setJointPos(int i, btScalar q);
	void setJointVel(int i, btScalar qdot);
	void setJointPosMultiDof(int i, const double *q);
	void setJointVelMultiDof(int i, const double *qdot);
	void setJointPosMultiDof(int i, const float *q);
	void setJointVelMultiDof(int i, const float *qdot);

	// direct access to velocities as a vector of 6 + num_links elements.
	// (omega first, then v, then joint velocities.)

	const btScalar *getVelocityVector() const
	{
		return &m_realBuf[0];
	}
    
    const btScalar *getDeltaVelocityVector() const
    {
        return &m_deltaV[0];
    }
    
    const btScalar *getSplitVelocityVector() const
    {
        return &m_splitV[0];
    }

	//
	// get the frames of reference (positions and orientations) of the child m_links
	// (i = 0 to num_links-1)
	//

	const btVector3 &getRVector(int i) const;              // vector from COM(parent(i)) to COM(i), in frame i's coords
	const btQuaternion &getParentToLocalRot(int i) const;  // rotates vectors in frame parent(i) to vectors in frame i.
    const btVector3 &getInterpolateRVector(int i) const;              // vector from COM(parent(i)) to COM(i), in frame i's coords
    const btQuaternion &getInterpolateParentToLocalRot(int i) const;  // rotates vectors in frame parent(i) to vectors in frame i.

	//
	// transform vectors in local frame of link i to world frame (or vice versa)
	//
	btVector3 localPosToWorld(int i, const btVector3 &local_pos) const;
	btVector3 localDirToWorld(int i, const btVector3 &local_dir) const;
	btVector3 worldPosToLocal(int i, const btVector3 &world_pos) const;
	btVector3 worldDirToLocal(int i, const btVector3 &world_dir) const;

	//
	// transform a frame in local coordinate to a frame in world coordinate
	//
	btMatrix3x3 localFrameToWorld(int i, const btMatrix3x3 &local_frame) const;


	//
	// set external forces and torques. Note all external forces/torques are given in the WORLD frame.
	//

	void clearForcesAndTorques();
	void clearConstraintForces();

	void clearVelocities();

	void addBaseForce(const btVector3 &f)
	{
		m_baseForce += f;
	}
	void addBaseTorque(const btVector3 &t) { m_baseTorque += t; }
	void addLinkForce(int i, const btVector3 &f);
	void addLinkTorque(int i, const btVector3 &t);

	void addBaseConstraintForce(const btVector3 &f)
	{
		m_baseConstraintForce += f;
	}
	void addBaseConstraintTorque(const btVector3 &t) { m_baseConstraintTorque += t; }
	void addLinkConstraintForce(int i, const btVector3 &f);
	void addLinkConstraintTorque(int i, const btVector3 &t);

	void addJointTorque(int i, btScalar Q);
	void addJointTorqueMultiDof(int i, int dof, btScalar Q);
	void addJointTorqueMultiDof(int i, const btScalar *Q);

	const btVector3 &getBaseForce() const { return m_baseForce; }
	const btVector3 &getBaseTorque() const { return m_baseTorque; }
	const btVector3 &getLinkForce(int i) const;
	const btVector3 &getLinkTorque(int i) const;
	btScalar getJointTorque(int i) const;
	btScalar *getJointTorqueMultiDof(int i);

	// dynamics routines.

	// timestep the velocities (given the external forces/torques set using addBaseForce etc).
	// also sets up caches for calcAccelerationDeltas.
	//
	// Note: the caller must provide three vectors which are used as
	// temporary scratch space. The idea here is to reduce dynamic
	// memory allocation: the same scratch vectors can be re-used
	// again and again for different Multibodies, instead of each
	// btMultiBody allocating (and then deallocating) their own
	// individual scratch buffers. This gives a considerable speed
	// improvement, at least on Windows (where dynamic memory
	// allocation appears to be fairly slow).
	//

	void computeAccelerationsArticulatedBodyAlgorithmMultiDof(btScalar dt,
															  btAlignedObjectArray<btScalar> & scratch_r,
															  btAlignedObjectArray<btVector3> & scratch_v,
															  btAlignedObjectArray<btMatrix3x3> & scratch_m,
															  bool isConstraintPass,
                                                              bool jointFeedbackInWorldSpace,
                                                              bool jointFeedbackInJointFrame
                                                              );

	///stepVelocitiesMultiDof is deprecated, use computeAccelerationsArticulatedBodyAlgorithmMultiDof instead
	//void stepVelocitiesMultiDof(btScalar dt,
	//							btAlignedObjectArray<btScalar> & scratch_r,
	//							btAlignedObjectArray<btVector3> & scratch_v,
	//							btAlignedObjectArray<btMatrix3x3> & scratch_m,
	//							bool isConstraintPass = false)
	//{
	//	computeAccelerationsArticulatedBodyAlgorithmMultiDof(dt, scratch_r, scratch_v, scratch_m, isConstraintPass, false, false);
	//}

	// calcAccelerationDeltasMultiDof
	// input: force vector (in same format as jacobian, i.e.:
	//                      3 torque values, 3 force values, num_links joint torque values)
	// output: 3 omegadot values, 3 vdot values, num_links q_double_dot values
	// (existing contents of output array are replaced)
	// calcAccelerationDeltasMultiDof must have been called first.
	void calcAccelerationDeltasMultiDof(const btScalar *force, btScalar *output,
										btAlignedObjectArray<btScalar> &scratch_r,
										btAlignedObjectArray<btVector3> &scratch_v) const;

	void applyDeltaVeeMultiDof2(const btScalar *delta_vee, btScalar multiplier)
	{
		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		{
			m_deltaV[dof] += delta_vee[dof] * multiplier;
		}
	}
    void applyDeltaSplitVeeMultiDof(const btScalar *delta_vee, btScalar multiplier)
    {
        for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
        {
            m_splitV[dof] += delta_vee[dof] * multiplier;
        }
    }
    void addSplitV()
    {
        applyDeltaVeeMultiDof(&m_splitV[0], 1);
    }
    void substractSplitV()
    {
        applyDeltaVeeMultiDof(&m_splitV[0], -1);
        
        for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
        {
            m_splitV[dof] = 0.f;
        }
    }
	void processDeltaVeeMultiDof2()
	{
		applyDeltaVeeMultiDof(&m_deltaV[0], 1);

		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		{
			m_deltaV[dof] = 0.f;
		}
	}

	void applyDeltaVeeMultiDof(const btScalar *delta_vee, btScalar multiplier)
	{
		//for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		//	printf("%.4f ", delta_vee[dof]*multiplier);
		//printf("\n");

		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		{
			m_realBuf[dof] += delta_vee[dof] * multiplier;
			btClamp(m_realBuf[dof], -m_maxCoordinateVelocity, m_maxCoordinateVelocity);
		}
	}

	// timestep the positions (given current velocities).
	void stepPositionsMultiDof(btScalar dt, btScalar *pq = 0, btScalar *pqd = 0);
    
    // predict the positions
    void predictPositionsMultiDof(btScalar dt);

	//
	// contacts
	//

	// This routine fills out a contact constraint jacobian for this body.
	// the 'normal' supplied must be -n for body1 or +n for body2 of the contact.
	// 'normal' & 'contact_point' are both given in world coordinates.

	void fillContactJacobianMultiDof(int link,
									 const btVector3 &contact_point,
									 const btVector3 &normal,
									 btScalar *jac,
									 btAlignedObjectArray<btScalar> &scratch_r,
									 btAlignedObjectArray<btVector3> &scratch_v,
									 btAlignedObjectArray<btMatrix3x3> &scratch_m) const { fillConstraintJacobianMultiDof(link, contact_point, btVector3(0, 0, 0), normal, jac, scratch_r, scratch_v, scratch_m); }

	//a more general version of fillContactJacobianMultiDof which does not assume..
	//.. that the constraint in question is contact or, to be more precise, constrains linear velocity only
	void fillConstraintJacobianMultiDof(int link,
										const btVector3 &contact_point,
										const btVector3 &normal_ang,
										const btVector3 &normal_lin,
										btScalar *jac,
										btAlignedObjectArray<btScalar> &scratch_r,
										btAlignedObjectArray<btVector3> &scratch_v,
										btAlignedObjectArray<btMatrix3x3> &scratch_m) const;

	//
	// sleeping
	//
	void setCanSleep(bool canSleep)
	{
		if (m_canWakeup)
		{
			m_canSleep = canSleep;
		}
	}

	bool getCanSleep() const
	{
		return m_canSleep;
	}

	bool getCanWakeup() const
	{
		return m_canWakeup;
	}
	
	void setCanWakeup(bool canWakeup) 
	{
		m_canWakeup = canWakeup;
	}
	bool isAwake() const 
	{ 
		return m_awake; 
	}
	void wakeUp();
	void goToSleep();
	void checkMotionAndSleepIfRequired(btScalar timestep);

	bool hasFixedBase() const;

	bool isBaseKinematic() const;

	bool isBaseStaticOrKinematic() const;

	// set the dynamic type in the base's collision flags.
	void setBaseDynamicType(int dynamicType);

	void setFixedBase(bool fixedBase)
	{
		m_fixedBase = fixedBase;
		if(m_fixedBase)
			setBaseDynamicType(btCollisionObject::CF_STATIC_OBJECT);
		else
			setBaseDynamicType(btCollisionObject::CF_DYNAMIC_OBJECT);
	}

	int getCompanionId() const
	{
		return m_companionId;
	}
	void setCompanionId(int id)
	{
		//printf("for %p setCompanionId(%d)\n",this, id);
		m_companionId = id;
	}

	void setNumLinks(int numLinks)  //careful: when changing the number of m_links, make sure to re-initialize or update existing m_links
	{
		m_links.resize(numLinks);
	}

	btScalar getLinearDamping() const
	{
		return m_linearDamping;
	}
	void setLinearDamping(btScalar damp)
	{
		m_linearDamping = damp;
	}
	btScalar getAngularDamping() const
	{
		return m_angularDamping;
	}
	void setAngularDamping(btScalar damp)
	{
		m_angularDamping = damp;
	}

	bool getUseGyroTerm() const
	{
		return m_useGyroTerm;
	}
	void setUseGyroTerm(bool useGyro)
	{
		m_useGyroTerm = useGyro;
	}
	btScalar getMaxCoordinateVelocity() const
	{
		return m_maxCoordinateVelocity;
	}
	void setMaxCoordinateVelocity(btScalar maxVel)
	{
		m_maxCoordinateVelocity = maxVel;
	}

	btScalar getMaxAppliedImpulse() const
	{
		return m_maxAppliedImpulse;
	}
	void setMaxAppliedImpulse(btScalar maxImp)
	{
		m_maxAppliedImpulse = maxImp;
	}
	void setHasSelfCollision(bool hasSelfCollision)
	{
		m_hasSelfCollision = hasSelfCollision;
	}
	bool hasSelfCollision() const
	{
		return m_hasSelfCollision;
	}

	void finalizeMultiDof();

	void useRK4Integration(bool use) { m_useRK4 = use; }
	bool isUsingRK4Integration() const { return m_useRK4; }
	void useGlobalVelocities(bool use) { m_useGlobalVelocities = use; }
	bool isUsingGlobalVelocities() const { return m_useGlobalVelocities; }

	bool isPosUpdated() const
	{
		return __posUpdated;
	}
	void setPosUpdated(bool updated)
	{
		__posUpdated = updated;
	}

	//internalNeedsJointFeedback is for internal use only
	bool internalNeedsJointFeedback() const
	{
		return m_internalNeedsJointFeedback;
	}
	void forwardKinematics(btAlignedObjectArray<btQuaternion>& world_to_local, btAlignedObjectArray<btVector3> & local_origin);

	void compTreeLinkVelocities(btVector3 * omega, btVector3 * vel) const;

	void updateCollisionObjectWorldTransforms(btAlignedObjectArray<btQuaternion> & world_to_local, btAlignedObjectArray<btVector3> & local_origin);
    void updateCollisionObjectInterpolationWorldTransforms(btAlignedObjectArray<btQuaternion> & world_to_local, btAlignedObjectArray<btVector3> & local_origin);

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char *serialize(void *dataBuffer, class btSerializer *serializer) const;

	const char *getBaseName() const
	{
		return m_baseName;
	}
	///memory of setBaseName needs to be manager by user
	void setBaseName(const char *name)
	{
		m_baseName = name;
	}

	///users can point to their objects, userPointer is not used by Bullet
	void *getUserPointer() const
	{
		return m_userObjectPointer;
	}

	int getUserIndex() const
	{
		return m_userIndex;
	}

	int getUserIndex2() const
	{
		return m_userIndex2;
	}
	///users can point to their objects, userPointer is not used by Bullet
	void setUserPointer(void *userPointer)
	{
		m_userObjectPointer = userPointer;
	}

	///users can point to their objects, userPointer is not used by Bullet
	void setUserIndex(int index)
	{
		m_userIndex = index;
	}

	void setUserIndex2(int index)
	{
		m_userIndex2 = index;
	}

	static void spatialTransform(const btMatrix3x3 &rotation_matrix,  // rotates vectors in 'from' frame to vectors in 'to' frame
		const btVector3 &displacement,     // vector from origin of 'from' frame to origin of 'to' frame, in 'to' coordinates
		const btVector3 &top_in,       // top part of input vector
		const btVector3 &bottom_in,    // bottom part of input vector
		btVector3 &top_out,         // top part of output vector
		btVector3 &bottom_out);      // bottom part of output vector

	void setLinkDynamicType(const int i, int type);

	bool isLinkStaticOrKinematic(const int i) const;

	bool isLinkKinematic(const int i) const;

	bool isLinkAndAllAncestorsStaticOrKinematic(const int i) const;

	bool isLinkAndAllAncestorsKinematic(const int i) const;

	void setSleepThreshold(btScalar sleepThreshold)
	{
		m_sleepEpsilon = sleepThreshold;
	}

	void setSleepTimeout(btScalar sleepTimeout)
	{
		this->m_sleepTimeout = sleepTimeout;
	}


private:
	btMultiBody(const btMultiBody &);     // not implemented
	void operator=(const btMultiBody &);  // not implemented

	void solveImatrix(const btVector3 &rhs_top, const btVector3 &rhs_bot, btScalar result[6]) const;
	void solveImatrix(const btSpatialForceVector &rhs, btSpatialMotionVector &result) const;

	void updateLinksDofOffsets()
	{
		int dofOffset = 0, cfgOffset = 0;
		for (int bidx = 0; bidx < m_links.size(); ++bidx)
		{
			m_links[bidx].m_dofOffset = dofOffset;
			m_links[bidx].m_cfgOffset = cfgOffset;
			dofOffset += m_links[bidx].m_dofCount;
			cfgOffset += m_links[bidx].m_posVarCount;
		}
	}

	void mulMatrix(const btScalar *pA, const btScalar *pB, int rowsA, int colsA, int rowsB, int colsB, btScalar *pC) const;

private:
	btMultiBodyLinkCollider *m_baseCollider;  //can be NULL
	const char *m_baseName;                   //memory needs to be manager by user!

	btVector3 m_basePos;      // position of COM of base (world frame)
    btVector3 m_basePos_interpolate;      // position of interpolated COM of base (world frame)
	btQuaternion m_baseQuat;  // rotates world points into base frame
    btQuaternion m_baseQuat_interpolate;  

	btScalar m_baseMass;      // mass of the base
	btVector3 m_baseInertia;  // inertia of the base (in local frame; diagonal)

	btVector3 m_baseForce;   // external force applied to base. World frame.
	btVector3 m_baseTorque;  // external torque applied to base. World frame.

	btVector3 m_baseConstraintForce;   // external force applied to base. World frame.
	btVector3 m_baseConstraintTorque;  // external torque applied to base. World frame.

	btAlignedObjectArray<btMultibodyLink> m_links;  // array of m_links, excluding the base. index from 0 to num_links-1.

	//
	// realBuf:
	//  offset         size            array
	//   0              6 + num_links   v (base_omega; base_vel; joint_vels)					MULTIDOF [sysdof x sysdof for D matrices (TOO MUCH!) + pos_delta which is sys-cfg sized]
	//   6+num_links    num_links       D
	//
	// vectorBuf:
	//  offset         size         array
	//   0              num_links    h_top
	//   num_links      num_links    h_bottom
	//
	// matrixBuf:
	//  offset         size         array
	//   0              num_links+1  rot_from_parent
	//
    btAlignedObjectArray<btScalar> m_splitV;
	btAlignedObjectArray<btScalar> m_deltaV;
	btAlignedObjectArray<btScalar> m_realBuf;
	btAlignedObjectArray<btVector3> m_vectorBuf;
	btAlignedObjectArray<btMatrix3x3> m_matrixBuf;

	btMatrix3x3 m_cachedInertiaTopLeft;
	btMatrix3x3 m_cachedInertiaTopRight;
	btMatrix3x3 m_cachedInertiaLowerLeft;
	btMatrix3x3 m_cachedInertiaLowerRight;
	bool m_cachedInertiaValid;

	bool m_fixedBase;

	// Sleep parameters.
	bool m_awake;
	bool m_canSleep;
	bool m_canWakeup;
	btScalar m_sleepTimer;
	btScalar m_sleepEpsilon;
	btScalar m_sleepTimeout;

	void *m_userObjectPointer;
	int m_userIndex2;
	int m_userIndex;

	int m_companionId;
	btScalar m_linearDamping;
	btScalar m_angularDamping;
	bool m_useGyroTerm;
	btScalar m_maxAppliedImpulse;
	btScalar m_maxCoordinateVelocity;
	bool m_hasSelfCollision;

	bool __posUpdated;
	int m_dofCount, m_posVarCnt;

	bool m_useRK4, m_useGlobalVelocities;
	//for global velocities, see 8.3.2B Proposed resolution in Jakub Stepien PhD Thesis
	//https://drive.google.com/file/d/0Bz3vEa19XOYGNWdZWGpMdUdqVmZ5ZVBOaEh4ZnpNaUxxZFNV/view?usp=sharing

	///the m_needsJointFeedback gets updated/computed during the stepVelocitiesMultiDof and it for internal usage only
	bool m_internalNeedsJointFeedback;

  //If enabled, calculate the velocity based on kinematic transform changes. Currently only implemented for the base.
	bool m_kinematic_calculate_velocity;
};

struct btMultiBodyLinkDoubleData
{
	btQuaternionDoubleData m_zeroRotParentToThis;
	btVector3DoubleData m_parentComToThisPivotOffset;
	btVector3DoubleData m_thisPivotToThisComOffset;
	btVector3DoubleData m_jointAxisTop[6];
	btVector3DoubleData m_jointAxisBottom[6];

	btVector3DoubleData m_linkInertia;  // inertia of the base (in local frame; diagonal)
	btVector3DoubleData m_absFrameTotVelocityTop;
	btVector3DoubleData m_absFrameTotVelocityBottom;
	btVector3DoubleData m_absFrameLocVelocityTop;
	btVector3DoubleData m_absFrameLocVelocityBottom;

	double m_linkMass;
	int m_parentIndex;
	int m_jointType;

	int m_dofCount;
	int m_posVarCount;
	double m_jointPos[7];
	double m_jointVel[6];
	double m_jointTorque[6];

	double m_jointDamping;
	double m_jointFriction;
	double m_jointLowerLimit;
	double m_jointUpperLimit;
	double m_jointMaxForce;
	double m_jointMaxVelocity;

	char *m_linkName;
	char *m_jointName;
	btCollisionObjectDoubleData *m_linkCollider;
	char *m_paddingPtr;
};

struct btMultiBodyLinkFloatData
{
	btQuaternionFloatData m_zeroRotParentToThis;
	btVector3FloatData m_parentComToThisPivotOffset;
	btVector3FloatData m_thisPivotToThisComOffset;
	btVector3FloatData m_jointAxisTop[6];
	btVector3FloatData m_jointAxisBottom[6];
	btVector3FloatData m_linkInertia;  // inertia of the base (in local frame; diagonal)
	btVector3FloatData m_absFrameTotVelocityTop;
	btVector3FloatData m_absFrameTotVelocityBottom;
	btVector3FloatData m_absFrameLocVelocityTop;
	btVector3FloatData m_absFrameLocVelocityBottom;

	int m_dofCount;
	float m_linkMass;
	int m_parentIndex;
	int m_jointType;

	float m_jointPos[7];
	float m_jointVel[6];
	float m_jointTorque[6];
	int m_posVarCount;
	float m_jointDamping;
	float m_jointFriction;
	float m_jointLowerLimit;
	float m_jointUpperLimit;
	float m_jointMaxForce;
	float m_jointMaxVelocity;

	char *m_linkName;
	char *m_jointName;
	btCollisionObjectFloatData *m_linkCollider;
	char *m_paddingPtr;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btMultiBodyDoubleData
{
	btVector3DoubleData m_baseWorldPosition;
	btQuaternionDoubleData m_baseWorldOrientation;
	btVector3DoubleData m_baseLinearVelocity;
	btVector3DoubleData m_baseAngularVelocity;
	btVector3DoubleData m_baseInertia;  // inertia of the base (in local frame; diagonal)
	double m_baseMass;
	int m_numLinks;
	char m_padding[4];

	char *m_baseName;
	btMultiBodyLinkDoubleData *m_links;
	btCollisionObjectDoubleData *m_baseCollider;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btMultiBodyFloatData
{
	btVector3FloatData m_baseWorldPosition;
	btQuaternionFloatData m_baseWorldOrientation;
	btVector3FloatData m_baseLinearVelocity;
	btVector3FloatData m_baseAngularVelocity;

	btVector3FloatData m_baseInertia;  // inertia of the base (in local frame; diagonal)
	float m_baseMass;
	int m_numLinks;

	char *m_baseName;
	btMultiBodyLinkFloatData *m_links;
	btCollisionObjectFloatData *m_baseCollider;
};

#endif
