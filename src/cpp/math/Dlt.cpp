#include "Dlt.h"
#include <iostream>

using namespace std;

namespace math
{
	ostream& operator<<( ostream& out, const Correspondence& correspondence )
	{
		out << "Correspondence: " << endl << correspondence.first << endl << ", " << endl << correspondence.second << endl;
		return out;
	}
	
	ostream& operator<<( ostream& out, const vector< Correspondence >& correspondences )
	{
		for( Correspondence correspondence : correspondences )
		{
			out << correspondence << endl;
		}
		return out;
	}
	
	Dlt::Dlt( vector< Correspondence > sample ) :
		m_sample( make_shared< vector< Correspondence > >( sample ) )
	{}
	
	MatrixXd Dlt::solve()
	{
		if( m_resultH != nullptr )
		{
			return *m_resultH;
		}
		else
		{
			//cout << "Correspondences before normalization: " << endl << *m_sample << endl << endl;
			normalize();
			//cout << "Correspondences after normalization: " << endl << *m_sample << endl;
			
			MatrixXd A( 8, 9 );
			for( int i = 0; i < m_sample->size(); ++i)
			{
				Block< MatrixXd > block = A.block(i * 2, 0, 2, 9);
				VectorXd v0 = (*m_sample)[i].first;
				VectorXd v1 = (*m_sample)[i].second;
				
				block 	<< 0.			, 0. 			, 0. 			, -v1[2] * v0[0] , -v1[2] * v0[1] 	, -v1[2] * v0[2] , v1[1] * v0[0]  , v1[1] * v0[1]  , v1[1] * v0[2],
						v1[2] * v0[0]	, v1[2] * v0[1] , v1[2] * v0[2] , 0. 			 , 0. 				, 0.			 , -v1[0] * v0[0] , -v1[0] * v0[1] , -v1[0] * v0[2];
			}
			
			cout << "Linear System matrix: " << endl << A << endl << endl;
			
			JacobiSVD<MatrixXd> svd(A, ComputeThinV);
			
			cout << "SVD V matrix: " << endl << svd.matrixV() << endl << endl;
			
			VectorXd hCol = svd.matrixV().col(7);
			
			m_resultH = make_shared< MatrixXd >(3, 3);
			(*m_resultH) << hCol[0], hCol[1], hCol[2],
							hCol[3], hCol[4], hCol[5],
							hCol[6], hCol[7], hCol[8];
			
			//cout << "H before denormalization: " << endl << *m_resultH << endl;
			denormalize();
			//cout << "H after denormalization: " << endl << *m_resultH << endl;
			
			return *m_resultH;
		}
	}
	
	double Dlt::scoreSolution( vector< Correspondence > correspondences )
	{
		// First, calculates the distance variance.
		double distanceSum = 0.;
		vector< double > distances;
		for( Correspondence correspondence : correspondences )
		{
			VectorXd transformed = ( *m_resultH ) * correspondence.first;
			transformed = transformed / transformed[ 2 ];
			double distance = ( transformed - correspondence.second ).norm();
			distances.push_back( distance );
			distanceSum += distance;
		}
		int numPoints = distances.size();
		
		double meanDistance = distanceSum / numPoints;
		double sumSquaredDiff = 0;
		for( double dist : distances)
		{
			double squaredDiff = pow(dist - meanDistance, 2);
			sumSquaredDiff += squaredDiff;
		}
		
		double variance = sumSquaredDiff / numPoints;
		
		// Second, calculates the distance threshold to consider transformed points as inliers.
		double threshold = sqrt( 5.99 * variance );
		
		// Third, returns the percentage of outliers in the correspondence set.
		int nOutliers = 0;
		for( double distance : distances )
		{
			if( distance > threshold )
			{
				++nOutliers;
			}
		}
		
		cout << "Homography: " << endl << *m_resultH << endl << endl
			 << "Outliers: " << nOutliers << " of " << numPoints << endl << endl;
		
		return nOutliers / numPoints;
	}
	
	void Dlt::normalize()
	{
		VectorXd vectorSum0(3); vectorSum0[0] = 0; vectorSum0[1] = 0; vectorSum0[2] = 1;
		VectorXd vectorSum1 = vectorSum0;
		
		for( Correspondence correspondence : *m_sample )
		{
			VectorXd p0 = correspondence.first;
			VectorXd p1 = correspondence.second;
			
			vectorSum0 += p0;
			vectorSum1 += p1;
		}		
		
		VectorXd centroid0 = vectorSum0 / m_sample->size();
		VectorXd centroid1 = vectorSum1 / m_sample->size();
		
		double distanceSum0 = 0;
		double distanceSum1 = 0;
		for( Correspondence correspondence : *m_sample )
		{
			VectorXd p0 = correspondence.first - centroid0;
			VectorXd p1 = correspondence.second - centroid1;
			distanceSum0 += p0.norm();
			distanceSum1 += p1.norm();
		}
		
		double scale0 = 1.414213562 / ( distanceSum0 / m_sample->size() );
		double scale1 = 1.414213562 / ( distanceSum1 / m_sample->size() );
		
		m_S0Normalizer = make_shared< MatrixXd >(3, 3);
		(*m_S0Normalizer) << scale0	, 0.	, -centroid0[0],
							 0.		, scale0, -centroid0[1],
							 0.		, 0.	, 1;
							 
		m_S1Normalizer = make_shared< MatrixXd >(3, 3);
		(*m_S1Normalizer) << scale1	, 0.	, -centroid1[0],
							 0.		, scale1, -centroid1[1],
							 0.		, 0.	, 1;
		
		cout << "S0 normalizer: " << endl << *m_S0Normalizer << endl << endl
			 << "S1 normalizer: " << endl << *m_S1Normalizer << endl << endl;
		
		for( int i = 0; i < m_sample->size(); ++i )
		{
			(*m_sample)[i].first = (*m_S0Normalizer) * (*m_sample)[i].first;
			(*m_sample)[i].second = (*m_S1Normalizer) * (*m_sample)[i].second;
		}
	}
		
	void Dlt::denormalize()
	{
		*m_resultH = m_S1Normalizer->inverse() * (*m_resultH) * (*m_S0Normalizer);
	}
}