#include "DltBase.h"
#include <iostream>

using namespace std;

namespace math
{
	DltBase::DltBase( vector< Correspondence > sample ) :
		m_sample( make_shared< vector< Correspondence > >( sample ) )
	{}
	
	void DltBase::applyRestrictions() { ; }
	
	MatrixXd DltBase::solve()
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
			
			MatrixXd A = createLinearSystem();
			
			//cout << "Linear System matrix: " << endl << A << endl << endl;
			
			JacobiSVD<MatrixXd> svd(A, ComputeThinV);
			
			//cout << "SVD V matrix: " << endl << svd.matrixV() << endl << endl;
			
			VectorXd hCol = svd.matrixV().col(7);
			
			m_resultH = make_shared< MatrixXd >(3, 3);
			(*m_resultH) << hCol[0], hCol[1], hCol[2],
							hCol[3], hCol[4], hCol[5],
							hCol[6], hCol[7], hCol[8];
			
			applyRestrictions();
			
			//cout << "H before denormalization: " << endl << *m_resultH << endl;
			denormalize();
			//cout << "H after denormalization: " << endl << *m_resultH << endl;
			
			return *m_resultH;
		}
	}
	
	void DltBase::normalize()
	{
		VectorXd vectorSum0(2); vectorSum0[0] = 0; vectorSum0[1] = 0;
		VectorXd vectorSum1 = vectorSum0;
		
		for( Correspondence correspondence : *m_sample )
		{
			VectorXd p0 = correspondence.first;
			VectorXd p1 = correspondence.second;
			
			vectorSum0[ 0 ] += p0[ 0 ]; vectorSum0[ 1 ] += p0[ 1 ];
			vectorSum1[ 0 ] += p1[ 0 ]; vectorSum1[ 1 ] += p1[ 1 ];
		}		
		
		VectorXd centroid0 = vectorSum0 / m_sample->size();
		VectorXd centroid1 = vectorSum1 / m_sample->size();
		
		double distanceSum0 = 0;
		double distanceSum1 = 0;
		for( Correspondence correspondence : *m_sample )
		{
			VectorXd p0( 2 );
			p0[ 0 ] = correspondence.first[ 0 ] - centroid0[ 0 ];
			p0[ 1 ] = correspondence.first[ 1 ] - centroid0[ 1 ];
			
			VectorXd p1( 2 );
			p1[ 0 ] = correspondence.second[ 0 ] - centroid1[ 0 ];
			p1[ 1 ] = correspondence.second[ 1 ] - centroid1[ 1 ];
			
			distanceSum0 += p0.norm();
			distanceSum1 += p1.norm();
		}
		
		double scale0 = 1.414213562 / ( distanceSum0 / m_sample->size() );
		double scale1 = 1.414213562 / ( distanceSum1 / m_sample->size() );
		
		m_S0Normalizer = make_shared< MatrixXd >(3, 3);
		(*m_S0Normalizer) << scale0	, 0.	, -centroid0[ 0 ],
							 0.		, scale0, -centroid0[ 1 ],
							 0.		, 0.	, 1;
							 
		m_S1Normalizer = make_shared< MatrixXd >(3, 3);
		(*m_S1Normalizer) << scale1	, 0.	, -centroid1[ 0 ],
							 0.		, scale1, -centroid1[ 1 ],
							 0.		, 0.	, 1;
		
		//cout << "S0 normalizer: " << endl << *m_S0Normalizer << endl << endl
		//	 << "S1 normalizer: " << endl << *m_S1Normalizer << endl << endl;
		
		for( int i = 0; i < m_sample->size(); ++i )
		{
			VectorXd p0 = (*m_S0Normalizer) * (*m_sample)[ i ].first;
			p0 = p0 / p0[ 2 ];
			(*m_sample)[ i ].first = p0;
			
			VectorXd p1 = (*m_S1Normalizer) * (*m_sample)[ i ].second;
			p1 = p1 / p1[ 2 ];
			(*m_sample)[ i ].second = p1;
		}
	}
		
	void DltBase::denormalize()
	{
		*m_resultH = m_S1Normalizer->inverse() * (*m_resultH) * (*m_S0Normalizer);
	}
}