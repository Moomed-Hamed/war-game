#ifndef BT_SOLVE_2LINEAR_CONSTRAINT_H
#define BT_SOLVE_2LINEAR_CONSTRAINT_H

#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btVector3.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"

class btRigidBody;

/// constraint class used for lateral tyre friction.
class btSolve2LinearConstraint
{
	btScalar m_tau;
	btScalar m_damping;

public:
	btSolve2LinearConstraint(btScalar tau, btScalar damping)
	{
		m_tau = tau;
		m_damping = damping;
	}
	//
	// solve unilateral constraint (equality, direct method)
	//
	void resolveUnilateralPairConstraint(
		btRigidBody* body1,
		btRigidBody* body2,

		const btMatrix3x3& world2A,
		const btMatrix3x3& world2B,

		const btVector3& invInertiaADiag,
		const btScalar invMassA,
		const btVector3& linvelA, const btVector3& angvelA,
		const btVector3& rel_posA1,
		const btVector3& invInertiaBDiag,
		const btScalar invMassB,
		const btVector3& linvelB, const btVector3& angvelB,
		const btVector3& rel_posA2,

		btScalar depthA, const btVector3& normalA,
		const btVector3& rel_posB1, const btVector3& rel_posB2,
		btScalar depthB, const btVector3& normalB,
		btScalar& imp0, btScalar& imp1)
	{
		(void)linvelA;
		(void)linvelB;
		(void)angvelB;
		(void)angvelA;

		imp0 = btScalar(0.);
		imp1 = btScalar(0.);

		btScalar len = btFabs(normalA.length()) - btScalar(1.);
		if (btFabs(len) >= SIMD_EPSILON)
			return;

		btAssert(len < SIMD_EPSILON);

		//this jacobian entry could be re-used for all iterations
		btJacobianEntry jacA(world2A, world2B, rel_posA1, rel_posA2, normalA, invInertiaADiag, invMassA,
							 invInertiaBDiag, invMassB);
		btJacobianEntry jacB(world2A, world2B, rel_posB1, rel_posB2, normalB, invInertiaADiag, invMassA,
							 invInertiaBDiag, invMassB);

		//const btScalar vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
		//const btScalar vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

		const btScalar vel0 = normalA.dot(body1->getVelocityInLocalPoint(rel_posA1) - body2->getVelocityInLocalPoint(rel_posA1));
		const btScalar vel1 = normalB.dot(body1->getVelocityInLocalPoint(rel_posB1) - body2->getVelocityInLocalPoint(rel_posB1));

		//	btScalar penetrationImpulse = (depth*contactTau*timeCorrection)  * massTerm;//jacDiagABInv
		btScalar massTerm = btScalar(1.) / (invMassA + invMassB);

		// calculate rhs (or error) terms
		const btScalar dv0 = depthA * m_tau * massTerm - vel0 * m_damping;
		const btScalar dv1 = depthB * m_tau * massTerm - vel1 * m_damping;

		// dC/dv * dv = -C

		// jacobian * impulse = -error
		//

		//impulse = jacobianInverse * -error

		// inverting 2x2 symmetric system (offdiagonal are equal!)
		//

		btScalar nonDiag = jacA.getNonDiagonal(jacB, invMassA, invMassB);
		btScalar invDet = btScalar(1.0) / (jacA.getDiagonal() * jacB.getDiagonal() - nonDiag * nonDiag);

		//imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
		//imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

		imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
		imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * -nonDiag * invDet;

		//[a b]								  [d -c]
		//[c d] inverse = (1 / determinant) * [-b a] where determinant is (ad - bc)

		//[jA nD] * [imp0] = [dv0]
		//[nD jB]   [imp1]   [dv1]
	}

	//
	// solving 2x2 lcp problem (inequality, direct solution )
	//
	void resolveBilateralPairConstraint(
		btRigidBody* body1,
		btRigidBody* body2,
		const btMatrix3x3& world2A,
		const btMatrix3x3& world2B,

		const btVector3& invInertiaADiag,
		const btScalar invMassA,
		const btVector3& linvelA, const btVector3& angvelA,
		const btVector3& rel_posA1,
		const btVector3& invInertiaBDiag,
		const btScalar invMassB,
		const btVector3& linvelB, const btVector3& angvelB,
		const btVector3& rel_posA2,

		btScalar depthA, const btVector3& normalA,
		const btVector3& rel_posB1, const btVector3& rel_posB2,
		btScalar depthB, const btVector3& normalB,
		btScalar& imp0, btScalar& imp1)
	{
		(void)linvelA;
		(void)linvelB;
		(void)angvelA;
		(void)angvelB;

		imp0 = btScalar(0.);
		imp1 = btScalar(0.);

		btScalar len = btFabs(normalA.length()) - btScalar(1.);
		if (btFabs(len) >= SIMD_EPSILON)
			return;

		btAssert(len < SIMD_EPSILON);

		//this jacobian entry could be re-used for all iterations
		btJacobianEntry jacA(world2A, world2B, rel_posA1, rel_posA2, normalA, invInertiaADiag, invMassA,
							 invInertiaBDiag, invMassB);
		btJacobianEntry jacB(world2A, world2B, rel_posB1, rel_posB2, normalB, invInertiaADiag, invMassA,
							 invInertiaBDiag, invMassB);

		//const btScalar vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
		//const btScalar vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

		const btScalar vel0 = normalA.dot(body1->getVelocityInLocalPoint(rel_posA1) - body2->getVelocityInLocalPoint(rel_posA1));
		const btScalar vel1 = normalB.dot(body1->getVelocityInLocalPoint(rel_posB1) - body2->getVelocityInLocalPoint(rel_posB1));

		// calculate rhs (or error) terms
		const btScalar dv0 = depthA * m_tau - vel0 * m_damping;
		const btScalar dv1 = depthB * m_tau - vel1 * m_damping;

		// dC/dv * dv = -C

		// jacobian * impulse = -error
		//

		//impulse = jacobianInverse * -error

		// inverting 2x2 symmetric system (offdiagonal are equal!)
		//

		btScalar nonDiag = jacA.getNonDiagonal(jacB, invMassA, invMassB);
		btScalar invDet = btScalar(1.0) / (jacA.getDiagonal() * jacB.getDiagonal() - nonDiag * nonDiag);

		//imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
		//imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

		imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
		imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * -nonDiag * invDet;

		//[a b]								  [d -c]
		//[c d] inverse = (1 / determinant) * [-b a] where determinant is (ad - bc)

		//[jA nD] * [imp0] = [dv0]
		//[nD jB]   [imp1]   [dv1]

		if (imp0 > btScalar(0.0))
		{
			if (imp1 > btScalar(0.0))
			{
				//both positive
			}
			else
			{
				imp1 = btScalar(0.);

				// now imp0>0 imp1<0
				imp0 = dv0 / jacA.getDiagonal();
				if (imp0 > btScalar(0.0))
				{
				}
				else
				{
					imp0 = btScalar(0.);
				}
			}
		}
		else
		{
			imp0 = btScalar(0.);

			imp1 = dv1 / jacB.getDiagonal();
			if (imp1 <= btScalar(0.0))
			{
				imp1 = btScalar(0.);
				// now imp0>0 imp1<0
				imp0 = dv0 / jacA.getDiagonal();
				if (imp0 > btScalar(0.0))
				{
				}
				else
				{
					imp0 = btScalar(0.);
				}
			}
			else
			{
			}
		}
	}
};

#endif  //BT_SOLVE_2LINEAR_CONSTRAINT_H
