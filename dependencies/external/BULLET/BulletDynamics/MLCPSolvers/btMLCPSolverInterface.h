// original version written by Erwin Coumans, October 2013

#ifndef BT_MLCP_SOLVER_INTERFACE_H
#define BT_MLCP_SOLVER_INTERFACE_H

#include "LinearMath/btMatrixX.h"

class btMLCPSolverInterface
{
public:
	virtual ~btMLCPSolverInterface()
	{
	}

	//return true is it solves the problem successfully
	virtual bool solveMLCP(const btMatrixXu& A, const btVectorXu& b, btVectorXu& x, const btVectorXu& lo, const btVectorXu& hi, const btAlignedObjectArray<int>& limitDependency, int numIterations, bool useSparsity = true) = 0;
};

#endif  //BT_MLCP_SOLVER_INTERFACE_H
