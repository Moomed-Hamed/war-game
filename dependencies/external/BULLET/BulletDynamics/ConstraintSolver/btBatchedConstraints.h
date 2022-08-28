#ifndef BT_BATCHED_CONSTRAINTS_H
#define BT_BATCHED_CONSTRAINTS_H

#include "LinearMath/btThreads.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"
#include <string.h>  //for memset
#include <cmath>

struct btBatchedConstraints
{
	enum BatchingMethod
	{
		BATCHING_METHOD_SPATIAL_GRID_2D,
		BATCHING_METHOD_SPATIAL_GRID_3D,
		BATCHING_METHOD_COUNT
	};
	struct Range
	{
		int begin;
		int end;

		Range() : begin(0), end(0) {}
		Range(int _beg, int _end) : begin(_beg), end(_end) {}
	};

	btAlignedObjectArray<int> m_constraintIndices;
	btAlignedObjectArray<Range> m_batches;        // each batch is a range of indices in the m_constraintIndices array
	btAlignedObjectArray<Range> m_phases;         // each phase is range of indices in the m_batches array
	btAlignedObjectArray<char> m_phaseGrainSize;  // max grain size for each phase
	btAlignedObjectArray<int> m_phaseOrder;       // phases can be done in any order, so we can randomize the order here
	btIDebugDraw* m_debugDrawer;

	static bool s_debugDrawBatches;

	btBatchedConstraints() { m_debugDrawer = NULL; }
	void setup(btConstraintArray* constraints,
			   const btAlignedObjectArray<btSolverBody>& bodies,
			   BatchingMethod batchingMethod,
			   int minBatchSize,
			   int maxBatchSize,
			   btAlignedObjectArray<char>* scratchMemory);
	bool validate(btConstraintArray* constraints, const btAlignedObjectArray<btSolverBody>& bodies) const;
};

#endif  // BT_BATCHED_CONSTRAINTS_H
