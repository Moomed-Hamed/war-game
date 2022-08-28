#ifndef BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H
#define BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

extern btScalar gContactBreakingThreshold;

class btConvexPenetrationDepthSolver;

///The convex2dConvex2dAlgorithm collision algorithm support 2d collision detection for btConvex2dShape
///Currently it requires the btMinkowskiPenetrationDepthSolver, it has support for 2d penetration depth computation
class btConvex2dConvex2dAlgorithm : public btActivatingCollisionAlgorithm
{
	btSimplexSolverInterface* m_simplexSolver;
	btConvexPenetrationDepthSolver* m_pdSolver;

	bool m_ownManifold;
	btPersistentManifold* m_manifoldPtr;
	bool m_lowLevelOfDetail;

public:
	btConvex2dConvex2dAlgorithm(btPersistentManifold* mf, const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver, int /* numPerturbationIterations */, int /* minimumPointsPerturbationThreshold */)
		: btActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
		  m_simplexSolver(simplexSolver),
		  m_pdSolver(pdSolver),
		  m_ownManifold(false),
		  m_manifoldPtr(mf),
		  m_lowLevelOfDetail(false)
	{
		(void)body0Wrap;
		(void)body1Wrap;
	}

	virtual ~btConvex2dConvex2dAlgorithm()
	{
		if (m_ownManifold)
		{
			if (m_manifoldPtr)
				m_dispatcher->releaseManifold(m_manifoldPtr);
		}
	}

	// Convex-Convex collision algorithm
	virtual void processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		if (!m_manifoldPtr)
		{
			//swapped?
			m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
			m_ownManifold = true;
		}
		resultOut->setPersistentManifold(m_manifoldPtr);

		//comment-out next line to test multi-contact generation
		//resultOut->getPersistentManifold()->clearManifold();

		const btConvexShape* min0 = static_cast<const btConvexShape*>(body0Wrap->getCollisionShape());
		const btConvexShape* min1 = static_cast<const btConvexShape*>(body1Wrap->getCollisionShape());

		btVector3 normalOnB;
		btVector3 pointOnBWorld;

		{
			btGjkPairDetector::ClosestPointInput input;

			btGjkPairDetector gjkPairDetector(min0, min1, m_simplexSolver, m_pdSolver);
			//TODO: if (dispatchInfo.m_useContinuous)
			gjkPairDetector.setMinkowskiA(min0);
			gjkPairDetector.setMinkowskiB(min1);

			{
				input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactBreakingThreshold();
				input.m_maximumDistanceSquared *= input.m_maximumDistanceSquared;
			}

			input.m_transformA = body0Wrap->getWorldTransform();
			input.m_transformB = body1Wrap->getWorldTransform();

			gjkPairDetector.getClosestPoints(input, *resultOut, dispatchInfo.m_debugDraw);

			btVector3 v0, v1;
			btVector3 sepNormalWorldSpace;
		}

		if (m_ownManifold)
		{
			resultOut->refreshContactPoints();
		}
	}

	virtual btScalar calculateTimeOfImpact(btCollisionObject* col0, btCollisionObject* col1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
	{
		(void)resultOut;
		(void)dispatchInfo;
		///Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

		///Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
		///col0->m_worldTransform,
		btScalar resultFraction = btScalar(1.);

		btScalar squareMot0 = (col0->getInterpolationWorldTransform().getOrigin() - col0->getWorldTransform().getOrigin()).length2();
		btScalar squareMot1 = (col1->getInterpolationWorldTransform().getOrigin() - col1->getWorldTransform().getOrigin()).length2();

		if (squareMot0 < col0->getCcdSquareMotionThreshold() &&
			squareMot1 < col1->getCcdSquareMotionThreshold())
			return resultFraction;

		//An adhoc way of testing the Continuous Collision Detection algorithms
		//One object is approximated as a sphere, to simplify things
		//Starting in penetration should report no time of impact
		//For proper CCD, better accuracy and handling of 'allowed' penetration should be added
		//also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)

		/// Convex0 against sphere for Convex1
		{
			btConvexShape* convex0 = static_cast<btConvexShape*>(col0->getCollisionShape());

			btSphereShape sphere1(col1->getCcdSweptSphereRadius());  //todo: allow non-zero sphere sizes, for better approximation
			btConvexCast::CastResult result;
			btVoronoiSimplexSolver voronoiSimplex;
			//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
			///Simplification, one object is simplified as a sphere
			btGjkConvexCast ccd1(convex0, &sphere1, &voronoiSimplex);
			//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
			if (ccd1.calcTimeOfImpact(col0->getWorldTransform(), col0->getInterpolationWorldTransform(),
									  col1->getWorldTransform(), col1->getInterpolationWorldTransform(), result))
			{
				//store result.m_fraction in both bodies

				if (col0->getHitFraction() > result.m_fraction)
					col0->setHitFraction(result.m_fraction);

				if (col1->getHitFraction() > result.m_fraction)
					col1->setHitFraction(result.m_fraction);

				if (resultFraction > result.m_fraction)
					resultFraction = result.m_fraction;
			}
		}

		/// Sphere (for convex0) against Convex1
		{
			btConvexShape* convex1 = static_cast<btConvexShape*>(col1->getCollisionShape());

			btSphereShape sphere0(col0->getCcdSweptSphereRadius());  //todo: allow non-zero sphere sizes, for better approximation
			btConvexCast::CastResult result;
			btVoronoiSimplexSolver voronoiSimplex;
			//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
			///Simplification, one object is simplified as a sphere
			btGjkConvexCast ccd1(&sphere0, convex1, &voronoiSimplex);
			//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
			if (ccd1.calcTimeOfImpact(col0->getWorldTransform(), col0->getInterpolationWorldTransform(),
									  col1->getWorldTransform(), col1->getInterpolationWorldTransform(), result))
			{
				//store result.m_fraction in both bodies

				if (col0->getHitFraction() > result.m_fraction)
					col0->setHitFraction(result.m_fraction);

				if (col1->getHitFraction() > result.m_fraction)
					col1->setHitFraction(result.m_fraction);

				if (resultFraction > result.m_fraction)
					resultFraction = result.m_fraction;
			}
		}

		return resultFraction;
	};

	virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
	{
		///should we use m_ownManifold to avoid adding duplicates?
		if (m_manifoldPtr && m_ownManifold)
			manifoldArray.push_back(m_manifoldPtr);
	}

	void setLowLevelOfDetail(bool useLowLevel)
	{
		m_lowLevelOfDetail = useLowLevel;
	}

	const btPersistentManifold* getManifold()
	{
		return m_manifoldPtr;
	}

	struct CreateFunc : public btCollisionAlgorithmCreateFunc
	{
		btConvexPenetrationDepthSolver* m_pdSolver;
		btSimplexSolverInterface* m_simplexSolver;
		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;

		CreateFunc(btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver)
		{
			m_simplexSolver = simplexSolver;
			m_pdSolver = pdSolver;
		}

		virtual ~CreateFunc() {}

		virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btConvex2dConvex2dAlgorithm));
			return new (mem) btConvex2dConvex2dAlgorithm(ci.m_manifold, ci, body0Wrap, body1Wrap, m_simplexSolver, m_pdSolver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
		}
	};
};

#endif  //BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H
