#include "Dlt.h"

namespace math
{
	Dlt::Dlt( vector< Correspondence > sample ) :
		m_sample( make_shared< vector< Correspondence > >(sample) )
	{}
	
	MatrixXd Dlt::solve()
	{
		if( m_resultH != nullptr )
		{
			return *m_resultH;
		}
		else
		{
			normalize();
			
			MatrixXd A( 8, 9 );
			for( int i = 0; i < m_sample->size(); ++i)
			{
				Block< MatrixXd > block = A.block(i * 2, 0, 2, 9);
				VectorXd v0 = (*m_sample)[i].first;
				VectorXd v1 = (*m_sample)[i].second;
				
				block 	<< 0. , 0. , 0. , -v1[2] * v0[0] , -v1[2] * v0[1] , -v1[2] * v0[2] , v1[1] * v0[0] , v1[1] * v0[1] , v1[1] * v0[2],
						v1[2] * v0[0] , v1[2] * v0[1] , v1[2] * v0[2] , 0. , 0. , 0. , -v1[0] * v0[0] , -v1[0] * v0[1] , -v1[0] * v0[2];
			}
			
			JacobiSVD<MatrixXd> svd(A, ComputeThinV);
			VectorXd hCol = svd.matrixV().col(7);
			
			m_resultH = make_shared< MatrixXd >(3, 3);
			(*m_resultH) << hCol[0], hCol[1], hCol[2],
							hCol[3], hCol[4], hCol[5],
							hCol[6], hCol[7], hCol[8];
			
			denormalize();
		}
	}
	
	double Dlt::scoreSolution( vector< Correspondence > correspondences )
	{
		// First, calculates the distance variance.
		double distanceSum = 0.;
		vector< double > distances;
		for( Correspondence correspondence : correspondences )
		{
			double distance = ( correspondence.first * (*m_resultH) - correspondence.second ).squaredNorm();
			distances.push_back( distance );
			distanceSum += distance;
		}
		
		double meanDistance = distanceSum / correspondences.size();
		double sumSquaredDiff = 0;
		for( double dist : distances)
		{
			double squaredDiff = pow(dist - meanDistance, 2);
			sumSquaredDiff += squaredDiff;
		}
		double variance = sumSquaredDiff / distances.size();
		
		// Second, calculates the distance threshold to consider transformed points as inliers.
		double sqrThreshold = 3.84 * variance;
		
		// Third, returns the percentage of outliers in the correspondence set.
		int nOutliers = 0;
		for( double distance : distances )
		{
			if( distance > sqrThreshold )
			{
				++nOutliers;
			}
		}
		
		return nOutliers / distances.size();
	}
	
	void Dlt::normalize()
	{
		VectorXd vectorSum0(2); vectorSum0[0] = 0; vectorSum0[1] = 0;
		VectorXd vectorSum1 = vectorSum0;
		double sqrDistanceSum0 = 0;
		double sqrDistanceSum1 = 0;
		
		for( Correspondence correspondence : *m_sample )
		{
			VectorXd p0 = correspondence.first;
			VectorXd p1 = correspondence.second;
			vectorSum0 += p0;
			vectorSum1 += p1;
			sqrDistanceSum0 += p0.squaredNorm();
			sqrDistanceSum1 += p1.squaredNorm();
		}
		
		VectorXd centroid0 = vectorSum0 / m_sample->size();
		VectorXd centroid1 = vectorSum1 / m_sample->size();
		double scale0 = 0.5 * sqrDistanceSum0 / m_sample->size();
		double scale1 = 0.5 * sqrDistanceSum1 / m_sample->size();
		
		m_S0Normalizer = make_shared< MatrixXd >(3, 3);
		(*m_S0Normalizer) << scale0	, 0.	, -centroid0[0],
							 0.		, scale0, -centroid0[1],
							 0.		, 0.	, 1;
		
		m_S1Normalizer = make_shared< MatrixXd >(3, 3);
		(*m_S0Normalizer) << scale1	, 0.	, -centroid1[0],
							 0.		, scale1, -centroid1[1],
							 0.		, 0.	, 1;
		
		for( int i = 0; i < m_sample->size(); ++i )
		{
			(*m_sample)[i].first = (*m_sample)[i].first * (*m_S0Normalizer);
			(*m_sample)[i].second = (*m_sample)[i].second * (*m_S1Normalizer);
		}
	}
		
	void Dlt::denormalize()
	{
		*m_resultH = m_S1Normalizer->inverse() * (*m_resultH) * (*m_S0Normalizer);
	}
}