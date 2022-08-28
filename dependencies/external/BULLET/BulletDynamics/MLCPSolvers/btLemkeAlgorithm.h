/* Copyright (C) 2004-2013 MBSim Development Team

Code was converted for the Bullet Continuous Collision Detection and Physics Library

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//The original version is here
//https://code.google.com/p/mbsim-env/source/browse/trunk/kernel/mbsim/numerics/linear_complementarity_problem/lemke_algorithm.cc
//This file is re-distributed under the ZLib license, with permission of the original author (Kilian Grundl)

#ifndef BT_NUMERICS_LEMKE_ALGORITHM_H_
#define BT_NUMERICS_LEMKE_ALGORITHM_H_

#include "LinearMath/btMatrixX.h"

#include <vector>  //todo: replace by btAlignedObjectArray

static btScalar btMachEps()
{
	static bool calculated = false;
	static btScalar machEps = btScalar(1.);
	if (!calculated)
	{
		do
		{
			machEps /= btScalar(2.0);
			// If next epsilon yields 1, then break, because current
			// epsilon is the machine epsilon.
		} while ((btScalar)(1.0 + (machEps / btScalar(2.0))) != btScalar(1.0));
		//		printf( "\nCalculated Machine epsilon: %G\n", machEps );
		calculated = true;
	}
	return machEps;
}
static btScalar btEpsRoot()
{
	static btScalar epsroot = 0.;
	static bool alreadyCalculated = false;

	if (!alreadyCalculated)
	{
		epsroot = btSqrt(btMachEps());
		alreadyCalculated = true;
	}
	return epsroot;
}

class btLemkeAlgorithm
{
public:
	btLemkeAlgorithm(const btMatrixXu& M_, const btVectorXu& q_, const int& DEBUGLEVEL_ = 0) : DEBUGLEVEL(DEBUGLEVEL_)
	{
		setSystem(M_, q_);
	}

	/* GETTER / SETTER */
	/**
   * \brief return info of solution process
   */
	int getInfo()
	{
		return info;
	}

	/**
   * \brief get the number of steps until the solution was found
   */
	int getSteps(void)
	{
		return steps;
	}

	/**
   * \brief set system with Matrix M and vector q
   */
	void setSystem(const btMatrixXu& M_, const btVectorXu& q_)
	{
		m_M = M_;
		m_q = q_;
	}
	/***************************************************/

	/**
   * \brief solve algorithm adapted from : Fast Implementation of Lemke’s Algorithm for Rigid Body Contact Simulation (John E. Lloyd)
   */
	btVectorXu solve(unsigned int maxloops = 0)
	{
		steps = 0;

		int dim = m_q.size();

		btVectorXu solutionVector(2 * dim);
		solutionVector.setZero();

		btMatrixXu ident(dim, dim);
		ident.setIdentity();

		btMatrixXu mNeg = m_M.negative();

		btMatrixXu A(dim, 2 * dim + 2);
		//
		A.setSubMatrix(0, 0, dim - 1, dim - 1, ident);
		A.setSubMatrix(0, dim, dim - 1, 2 * dim - 1, mNeg);
		A.setSubMatrix(0, 2 * dim, dim - 1, 2 * dim, -1.f);
		A.setSubMatrix(0, 2 * dim + 1, dim - 1, 2 * dim + 1, m_q);

		btAlignedObjectArray<int> basis;
		//At first, all w-values are in the basis
		for (int i = 0; i < dim; i++)
			basis.push_back(i);

		int pivotRowIndex = -1;
		btScalar minValue = 1e30f;
		bool greaterZero = true;
		for (int i = 0; i < dim; i++)
		{
			btScalar v = A(i, 2 * dim + 1);
			if (v < minValue)
			{
				minValue = v;
				pivotRowIndex = i;
			}
			if (v < 0)
				greaterZero = false;
		}

		//  int pivotRowIndex = q_.minIndex();//minIndex(q_);     // first row is that with lowest q-value
		int z0Row = pivotRowIndex;    // remember the col of z0 for ending algorithm afterwards
		int pivotColIndex = 2 * dim;  // first col is that of z0

		if (!greaterZero)
		{
			if (maxloops == 0)
			{
				maxloops = 100;
				//        maxloops = UINT_MAX; //TODO: not a really nice way, problem is: maxloops should be 2^dim (=1<<dim), but this could exceed UINT_MAX and thus the result would be 0 and therefore the lemke algorithm wouldn't start but probably would find a solution within less then UINT_MAX steps. Therefore this constant is used as a upper border right now...
			}

			/*start looping*/
			for (steps = 0; steps < maxloops; steps++)
			{
				GaussJordanEliminationStep(A, pivotRowIndex, pivotColIndex, basis);

				int pivotColIndexOld = pivotColIndex;

				/*find new column index */
				if (basis[pivotRowIndex] < dim)  //if a w-value left the basis get in the correspondent z-value
					pivotColIndex = basis[pivotRowIndex] + dim;
				else
					//else do it the other way round and get in the corresponding w-value
					pivotColIndex = basis[pivotRowIndex] - dim;

				/*the column becomes part of the basis*/
				basis[pivotRowIndex] = pivotColIndexOld;

				pivotRowIndex = findLexicographicMinimum(A, pivotColIndex);

				if (z0Row == pivotRowIndex)
				{  //if z0 leaves the basis the solution is found --> one last elimination step is necessary
					GaussJordanEliminationStep(A, pivotRowIndex, pivotColIndex, basis);
					basis[pivotRowIndex] = pivotColIndex;  //update basis
					break;
				}
			}

			if (!validBasis(basis))
			{
				info = -1;

				return solutionVector;
			}
		}

		for (int i = 0; i < basis.size(); i++)
		{
			solutionVector[basis[i]] = A(i, 2 * dim + 1);  //q_[i];
		}

		info = 0;

		return solutionVector;
	}

	virtual ~btLemkeAlgorithm()
	{
	}

protected:
	int findLexicographicMinimum(const btMatrixXu& A, const int& pivotColIndex)
	{
		int RowIndex = 0;
		int dim = A.rows();
		btAlignedObjectArray<btVectorXu> Rows;
		for (int row = 0; row < dim; row++)
		{
			btVectorXu vec(dim + 1);
			vec.setZero();  //, INIT, 0.)
			Rows.push_back(vec);
			btScalar a = A(row, pivotColIndex);
			if (a > 0)
			{
				Rows[row][0] = A(row, 2 * dim + 1) / a;
				Rows[row][1] = A(row, 2 * dim) / a;
				for (int j = 2; j < dim + 1; j++)
					Rows[row][j] = A(row, j - 1) / a;
			}
		}

		for (int i = 0; i < Rows.size(); i++)
		{
			if (Rows[i].nrm2() > 0.)
			{
				int j = 0;
				for (; j < Rows.size(); j++)
				{
					if (i != j)
					{
						if (Rows[j].nrm2() > 0.)
						{
							btVectorXu test(dim + 1);
							for (int ii = 0; ii < dim + 1; ii++)
								test[ii] = Rows[j][ii] - Rows[i][ii];

							if (!LexicographicPositive(test))
								break;
						}
					}
				}

				if (j == Rows.size())
				{
					RowIndex += i;
					break;
				}
			}
		}

		return RowIndex;
	}
	bool LexicographicPositive(const btVectorXu& v)
	{
		int i = 0;

		while (i < v.size() - 1 && fabs(v[i]) < btMachEps())
			i++;
		if (v[i] > 0)
			return true;

		return false;
	}
	void GaussJordanEliminationStep(btMatrixXu& A, int pivotRowIndex, int pivotColumnIndex, const btAlignedObjectArray<int>& basis)
	{
		btScalar a = -1 / A(pivotRowIndex, pivotColumnIndex);

		for (int i = 0; i < A.rows(); i++)
		{
			if (i != pivotRowIndex)
			{
				for (int j = 0; j < A.cols(); j++)
				{
					if (j != pivotColumnIndex)
					{
						btScalar v = A(i, j);
						v += A(pivotRowIndex, j) * A(i, pivotColumnIndex) * a;
						A.setElem(i, j, v);
					}
				}
			}
		}

		for (int i = 0; i < A.cols(); i++)
		{
			A.mulElem(pivotRowIndex, i, -a);
		}

		for (int i = 0; i < A.rows(); i++)
		{
			if (i != pivotRowIndex)
			{
				A.setElem(i, pivotColumnIndex, 0);
			}
		}
	}
	bool greaterZero(const btVectorXu& vector)
	{
		bool isGreater = true;
		for (int i = 0; i < vector.size(); i++)
		{
			if (vector[i] < 0)
			{
				isGreater = false;
				break;
			}
		}

		return isGreater;
	}
	bool validBasis(const btAlignedObjectArray<int>& basis)
	{
		bool isValid = true;
		for (int i = 0; i < basis.size(); i++)
		{
			if (basis[i] >= basis.size() * 2)
			{  //then z0 is in the base
				isValid = false;
				break;
			}
		}

		return isValid;
	}

	btMatrixXu m_M;
	btVectorXu m_q;

	/**
   * \brief number of steps until the Lemke algorithm found a solution
   */
	unsigned int steps;

	/**
   * \brief define level of debug output
   */
	int DEBUGLEVEL;

	/**
   * \brief did the algorithm find a solution
   *
   * -1 : not successful
   *  0 : successful
   */
	int info;
};

#endif /* BT_NUMERICS_LEMKE_ALGORITHM_H_ */
