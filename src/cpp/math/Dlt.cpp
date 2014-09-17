#include "Dlt.h"
#include <iostream>

using namespace std;

namespace math
{
	Dlt::Dlt( vector< Correspondence > sample )
	: DltBase( sample )
	{}
	
	MatrixXd Dlt::createLinearSystem()
	{
		int sampleSize = m_sample->size();
		MatrixXd A( 2 * sampleSize, 9 );
		for( int i = 0; i < sampleSize; ++i)
		{
			Block< MatrixXd > block = A.block(i * 2, 0, 2, 9);
			VectorXd v0 = (*m_sample)[i].first;
			VectorXd v1 = (*m_sample)[i].second;
			
			block 	<< 0.			, 0. 			, 0. 			, -v1[2] * v0[0] , -v1[2] * v0[1] 	, -v1[2] * v0[2] , v1[1] * v0[0]  , v1[1] * v0[1]  , v1[1] * v0[2],
					v1[2] * v0[0]	, v1[2] * v0[1] , v1[2] * v0[2] , 0. 			 , 0. 				, 0.			 , -v1[0] * v0[0] , -v1[0] * v0[1] , -v1[0] * v0[2];
		}
		
		return A;
	}
	
	int Dlt::scoreSolution( vector< Correspondence > correspondences )
	{
		// First, calculates the distance variance.
		vector< double > distances;
		for( Correspondence correspondence : correspondences )
		{
			VectorXd transformed = ( *m_resultH ) * correspondence.first;
			transformed = transformed / transformed[ 2 ];
			
			VectorXd original = ( *m_resultH ).inverse() * correspondence.second;
			original = original / original[ 2 ];
			
			VectorXd diffTransformed = transformed - correspondence.second;
			VectorXd diffOriginal = original - correspondence.first;
			
			double distance = diffTransformed.norm() + diffOriginal.norm();
			
			distances.push_back( distance );
		}
		int numPoints = distances.size();
		
		double threshold = 2.;
		
		// Third, returns the percentage of outliers in the correspondence set.
		int inliers = 0;
		for( int i = 0; i < distances.size(); ++i)
		{
			double distance = distances[ i ]; 
			//cout << "Variance: " << variance << endl << "threshold: " << threshold << endl
			//	 << "distance: " << distance << endl << endl;
			if( distance <= threshold )
			{
				++inliers;
			}
		}
		
		//cout << "Homography: " << endl << *m_resultH << endl << endl
		//	 << "Inliers: " << inliers << " of " << numPoints << endl << endl;
		
		return inliers;
	}
}