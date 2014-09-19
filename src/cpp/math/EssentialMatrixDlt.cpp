#include "EssentialMatrixDlt.h"
#include "TriangulationDlt.h"

namespace math
{
	EssentialMatrixDlt::EssentialMatrixDlt( vector< Correspondence >& correspondences, shared_ptr< MatrixXd > K0,
											shared_ptr< MatrixXd > K1 )
	: DltBase( correspondences ),
	m_K0( K0 ),
	m_K1( K1 )
	{}
	
	MatrixXd EssentialMatrixDlt::createLinearSystem() const
	{
		int sampleSize = m_sample->size();
		MatrixXd A( sampleSize, 9 );
		
		for( int i = 0; i < sampleSize; ++i)
		{
			Block< MatrixXd > block = A.block(i, 0, 1, 9);
			VectorXd v0 = (*m_sample)[i].first;
			VectorXd v1 = (*m_sample)[i].second;
			
			block 	<< v1[ 0 ] * v0[ 0 ], v1[ 0 ] * v0[ 1 ], v1[0], v1[ 1 ] * v0[ 0 ], v1[ 1 ] * v0[ 1 ], v1[ 1 ],
					   v0[ 0 ], v0[ 1 ], 1.;
		}
		
		return A;
	}
	
	void EssentialMatrixDlt::applyRestrictions()
	{
		JacobiSVD< MatrixXd > Fsvd( *m_resultH, ComputeFullU | ComputeFullV );
		VectorXd singularValues = Fsvd.singularValues();
		DiagonalMatrix< double, 3, 3 > D( singularValues[ 0 ], singularValues[ 1 ], 0. );
		
		*m_resultH = Fsvd.matrixU() * D * Fsvd.matrixV().transpose();
	}
	
	bool EssentialMatrixDlt::checkP1( const MatrixXd& P0, const MatrixXd& P1 ) const
	{
		TriangulationDlt dlt( *m_sample, P0, P1 );
		dlt.solve();
		VectorXd point3D = dlt.getPoint3D();
		
		MatrixXd KR1 = P1.block( 0, 0, 3, 3 ); // Calibration and rotation part.
		HouseholderQR< MatrixXd > qr( KR1 );
		MatrixXd R = qr.householderQ();
		
		VectorXd zRotated = R.col( 2 );
		return point3D[2] > 0. && point3D.dot( zRotated ) > 0.;
	}
	
	MatrixXd EssentialMatrixDlt::computeP( MatrixXd& E )
	{
		JacobiSVD< MatrixXd > svd( E, ComputeFullU | ComputeFullV );
		MatrixXd U = svd.matrixU();
		MatrixXd Vt = svd.matrixV().transpose();
		MatrixXd W( 3 , 3);
		W << 0., -1., 0.,
			 1., 0., 0.,
			 0., 0., 1.;
		VectorXd u3 = U.col(2);
		
		m_P0 = make_shared< MatrixXd >( 3, 4 ); // Just K0: no translation or rotation.
		*m_P0 << ( *m_K0 )( 0, 0 ), ( *m_K0 )( 0, 1 ), ( *m_K0 )( 0, 2 ), 0.,
				 ( *m_K0 )( 1, 0 ), ( *m_K0 )( 1, 1 ), ( *m_K0 )( 1, 2 ), 0.,
				 ( *m_K0 )( 2, 0 ), ( *m_K0 )( 2, 1 ), ( *m_K0 )( 2, 2 ), 0.;
		
		MatrixXd P1noT = U * W * Vt; // P without translation part.
		
		MatrixXd P1( 3, 4 );
		P1 << P1noT( 0, 0 ), P1noT( 0, 1 ), P1noT( 0, 2 ), u3[ 0 ],
			  P1noT( 1, 0 ), P1noT( 1, 1 ), P1noT( 1, 2 ), u3[ 1 ],
			  P1noT( 2, 0 ), P1noT( 2, 1 ), P1noT( 2, 2 ), u3[ 2 ];
		
		if( checkP1( *m_P0, P1 ) )
		{
			*m_resultH = P1;
			return *m_resultH;
		}
		
		P1.block( 0, 3, 3, 1 ) << -u3[ 0 ], -u3[ 1 ], -u3[ 2 ];
		if( checkP1( *m_P0, P1 ) )
		{
			*m_resultH = P1;
			return *m_resultH;
		}
		
		P1noT = U * W.transpose() * Vt;
		P1 << P1noT( 0, 0 ), P1noT( 0, 1 ), P1noT( 0, 2 ), u3[ 0 ],
			  P1noT( 1, 0 ), P1noT( 1, 1 ), P1noT( 1, 2 ), u3[ 1 ],
			  P1noT( 2, 0 ), P1noT( 2, 1 ), P1noT( 2, 2 ), u3[ 2 ];
		if( checkP1( *m_P0, P1 ) )
		{
			*m_resultH = P1;
			return *m_resultH;
		}
		
		P1.block( 0, 3, 3, 1 ) << -u3[ 0 ], -u3[ 1 ], -u3[ 2 ];
		if( checkP1( *m_P0, P1 ) )
		{
			*m_resultH = P1;
			return *m_resultH;
		}
		
		throw runtime_error( "None of the results for P1 was accepted!" );
	}
	
	void EssentialMatrixDlt::onDenormalizationEnd()
	{
		MatrixXd E = ( m_K1->transpose() ) * ( *m_resultH ) * ( *m_K0 );
		*m_resultH = computeP( E );
	}
	
	int EssentialMatrixDlt::scoreSolution( shared_ptr< vector< Correspondence > > allCorrespondences )
	{
		m_points3D = make_shared< vector< VectorXd > >();
		
		vector< double > distances;
		for( Correspondence correspondence : *allCorrespondences )
		{
			vector< Correspondence > pair;
			pair.push_back( correspondence );
			
			TriangulationDlt dlt( pair, *m_P0, *m_resultH );
			dlt.solve();
			VectorXd point3D = dlt.getPoint3D();
			m_points3D->push_back( point3D );
			
			VectorXd v0Expected = ( *m_P0 ) * point3D;
			v0Expected = v0Expected / v0Expected[ 2 ];
			VectorXd v1Expected = ( *m_resultH ) * point3D;
			v1Expected = v1Expected / v1Expected[ 2 ];
			
			VectorXd v0Diff = v0Expected - correspondence.first;
			VectorXd v1Diff = v1Expected - correspondence.second;
			
			double distance = v0Diff.norm() + v1Diff.norm();
			
			distances.push_back( distance );
		}
		
		double threshold = 2.;
		
		// Third, returns the percentage of outliers in the correspondence set.
		int inliers = 0;
		for( int i = 0; i < distances.size(); ++i)
		{
			double distance = distances[ i ]; 
			
			if( distance <= threshold )
			{
				++inliers;
			}
		}
		
		//cout << "Homography: " << endl << *m_resultH << endl << endl
		//	 << "Inliers: " << inliers << " of " << numPoints << endl << endl;
		
		return inliers;
	}
	
	MatrixXd EssentialMatrixDlt::getP0() const { return *m_P0; }
	
	shared_ptr< vector< VectorXd > > EssentialMatrixDlt::getPoints3D() const { return m_points3D; }
}