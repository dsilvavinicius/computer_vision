#ifndef RANSAC_H
#define RANSAC_H

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <vector>
#include <map>
#include <utility>
#include <functional>
#include <Eigen/Dense>

#include "PanoramaDlt.h"
#include "CameraMatrixDlt.h"

using namespace std;
using namespace Eigen;

namespace math
{	
	/** Suppose a linear system defined by restrictions generated by a sample with s elements in a large set S. The RANSAC
	 * algorithm ensures that the solution of the system defined by the final sample with s elements will have big
	 * concordance with all elements in S. In other words, the final solution of the system will have a small number of
	 * outliers in S. */
	template < typename Correspondence, typename Solver >
	class Ransac
	{
	public:
		/** Initializes RANSAC.
		 * @param set is the set of elements from which the algorithm will acquire the samples.
		 * @param evaluator is a function to evaluate the linear system for a sample in S, returning the system solution.
		 * @param nElementsPerSample is the number of elements in a sample. The sample defines the linear system.
		 * @param epsilon is the initial probability of outliers in samples.
		 */
		Ransac( shared_ptr< vector< Correspondence > > set, int nElementsPerSample, double epsilon = 0.99 );
		
		Ransac( vector< Correspondence >& set, int nElementsPerSample, double epsilon = 0.99 );
		
		/** Computes the result of the linear system.
		 * @param evaluator evaluates the vector of samples and returns the system solution.
		 * @param scorer scores the solution computed by evaluator, returning a new epsilon.
		 */
		MatrixXd compute();
		
		shared_ptr< Solver > getSolver();
	
	protected:
		/** Method to create the solvers for RANSAC iterations. Default implementation just calls the solver constructor
		 * with the current sample as parameter. */
		virtual Solver createSolver( vector< Correspondence >& sample );
		
		/** Gets the number of iteration, give the number of elements in sample and the probability of having outliers. */
		static double getIterNumber( int numElements, float outlierProb );
		
		/** The samples. */
		shared_ptr< vector< Correspondence > > m_set;
		
		/** The solver of the best iteration of the RANSAC loop. */
		shared_ptr< Solver > m_bestSolver;
		
		/** Probability of getting outliers. */
		double m_epsilon;
		
		/** Number of samples per iteration. */
		int m_nElementsPerSample;
		
		/** Estimated number of iterations to guarantee a sample without outliers. */
		double m_nIter;
	};
	
	template< typename Correspondence, typename Solver >
	double Ransac< Correspondence, Solver >::getIterNumber( int numElements, float outlierProb )
	{
		//return Ransac< T >::m_iterTable[ numElements ].lower_bound( outlierProb )->second;
		double result = log( 1. - 0.99 ) / log( 1. - pow( 1. - outlierProb, 4. ) );
		//cout << "Iter number calculation: " << result << endl << "Epsilon: " << outlierProb << endl << endl;
		return result;
	}
	
	template< typename Correspondence, typename Solver >
	Ransac< Correspondence, Solver >::Ransac( vector< Correspondence >& set, int nElementsPerSample, double epsilon)
	: Ransac( make_shared< vector< Correspondence > >( set ), nElementsPerSample, epsilon )
	{}
	
	template< typename Correspondence, typename Solver >
	Ransac< Correspondence, Solver >::Ransac( shared_ptr< vector< Correspondence > > set, int nElementsPerSample,
																double epsilon ) :
		m_set( set ),
		m_nElementsPerSample( nElementsPerSample ),
		m_epsilon( epsilon )
	{
		m_nIter = getIterNumber( m_nElementsPerSample, m_epsilon );
		srand( time( NULL ) );
	}
	
	template< typename Correspondence, typename Solver >
	Solver Ransac< Correspondence, Solver >::createSolver( vector< Correspondence >& sample )
	{
		return Solver( sample );
	}
	
	/** Dummy specialization. This code will be overriden by ReconstructionRansac. What a shame! */
	template <>
	CameraMatrixDlt Ransac< Correspondence, CameraMatrixDlt >::createSolver( vector< Correspondence >& sample );
	
	template< typename Correspondence, typename Solver >
	MatrixXd Ransac< Correspondence, Solver >::compute()
	{
		cout << "Starting max iters: " << m_nIter << endl << endl;
		for( double iter = 0; iter < m_nIter && iter < 20000.; ++iter ) 
		{
			cout << "Current epsilon: " << m_epsilon << endl << "Current max iters: " << m_nIter << endl
				 << "Current iter: " << iter << endl << endl;
			
			vector< Correspondence > sample;
			
			for( int i = 0; i < m_nElementsPerSample; ++i )
			{
				sample.push_back( ( *m_set )[ rand() % m_set->size() ] );
			}
			
			Solver solver = createSolver( sample );
			MatrixXd systemSol = solver.solve();
			
			int currInliers = solver.scoreSolution( m_set );
			double newEpsilon = 1 - ((double) currInliers /  m_set->size() );
			cout << "Iter score: " << newEpsilon << endl << endl;
			
			if( newEpsilon < m_epsilon )
			{
				cout << "Better result. Inliers: " << currInliers << " of " << m_set->size() << ", Prev epsilon: "
					 << m_epsilon << ", new Epsilon: " << newEpsilon << ", Prev iter: " << m_nIter;
				
				m_epsilon = newEpsilon;
				m_nIter = getIterNumber( m_nElementsPerSample, m_epsilon );
				
				cout << ", New Iter: " << m_nIter << ", Current Iter: " << iter << endl << endl;
				m_bestSolver = make_shared< Solver >( solver );
			}
		}
		
		return m_bestSolver->getSolution();
	}
	
	template< typename Correspondence, typename Solver >
	shared_ptr< Solver > Ransac< Correspondence, Solver >::getSolver()
	{
		return m_bestSolver;
	}
}

#endif