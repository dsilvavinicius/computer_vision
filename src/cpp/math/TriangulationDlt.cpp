#include "TriangulationDlt.h"

namespace math
{
	TriangulationDlt::TriangulationDlt( vector< Correspondence>& correspondence, const MatrixXd& P0, const MatrixXd& P1)
	: DltBase( correspondence ),
	m_P0( P0 ),
	m_P1( P1 )
	{
		if( correspondence.size() != 1 ) {
			stringstream ss;
			ss << "The triangulation needs one correspondence pair only. Actual size is " << correspondence.size();
			throw logic_error( ss.str() );
		}
	}
	
	void TriangulationDlt::buildAndApplyNormalizers( const VectorXd& centroid0, const double& scale0,
													 const VectorXd& centroid1, const double& scale1 )
	{
		/*m_S0Normalizer = make_shared< MatrixXd >( 4, 4);
		( *m_S0Normalizer ) << scale0	, 0.	, 0		, -centroid0[ 0 ],
							   0.		, scale0, 0		, -centroid0[ 1 ],
							   0.		, 0.	, scale0, -centroid0[ 2 ],
							   0.		, 0.	, 0		, 1.;
							 
		m_S1Normalizer = make_shared< MatrixXd >( 4, 4 );
		( *m_S1Normalizer ) << scale1	, 0.	, 0		, -centroid1[ 0 ],
							   0.		, scale1, 0		, -centroid1[ 1 ],
							   0.		, 0.	, scale1, -centroid1[ 2 ],
							   0.		, 0.	, 0		, 1.;
		
		//cout << "S0 normalizer: " << endl << *m_S0Normalizer << endl << endl
		//	 << "S1 normalizer: " << endl << *m_S1Normalizer << endl << endl;

		VectorXd p0 = (*m_S0Normalizer) * (*m_sample)[ 0 ].first;
		p0 = p0 / p0[ 3 ];
		( *m_sample )[ 0 ].first = p0;
			
		VectorXd p1 = (*m_S1Normalizer) * (*m_sample)[ 0 ].second;
		p1 = p1 / p1[ 3 ];
		( *m_sample )[ 0 ].second = p1;*/
	}
	
	void TriangulationDlt::normalize()
	{
		/*VectorXd v0 = ( *m_sample )[ 0 ].first;
		VectorXd v1 = ( *m_sample )[ 0 ].second;
		
		VectorXd centroid0( 4 );
		centroid0[ 0 ] = v0[ 0 ]; centroid0[ 1 ] = v0[ 1 ]; centroid0[ 2 ] = v0[ 2 ]; centroid0[ 1 ] = 1.;
		VectorXd centroid1( 4 );
		centroid1[ 0 ] = v1[ 0 ]; centroid1[ 1 ] = v1[ 1 ]; centroid1[ 2 ] = v0[ 2 ]; centroid1[ 1 ] = 1.;
		
		double scale0 = 1.414213562 / v0.norm();
		double scale1 = 1.414213562 / v1.norm();
		
		buildAndApplyNormalizers( centroid0, scale0, centroid1, scale1);*/
	}

	
	MatrixXd TriangulationDlt::createLinearSystem() const
	{
		MatrixXd A( 4, 4 );
		
		VectorXd v0 = (*m_sample)[0].first;
		VectorXd v1 = (*m_sample)[0].second;
		
		VectorXd p0r0 = m_P0.row( 0 );
		VectorXd p0r1 = m_P0.row( 1 );
		VectorXd p0r2 = m_P0.row( 2 );
		
		VectorXd p1r0 = m_P1.row( 0 );
		VectorXd p1r1 = m_P1.row( 1 );
		VectorXd p1r2 = m_P1.row( 2 );
		
		//=====================================================
		// MAYBE THE FOLLOWING VECTORS SHOULDN'T BE NORMALIZED.
		//=====================================================
		VectorXd v0xTimesP0R2MinusP0R0 = v0[ 0 ] * p0r2 - p0r0;
		v0xTimesP0R2MinusP0R0.normalize();
		
		VectorXd v0yTimesP0R2MinusP0R1 = v0[ 1 ] * p0r2 - p0r1;
		v0yTimesP0R2MinusP0R1.normalize();
		
		VectorXd v1xTimesP0R2MinusP0R0 = v1[ 0 ] * p1r2 - p1r0;
		v1xTimesP0R2MinusP0R0.normalize();
		
		VectorXd v1yTimesP0R2MinusP0R1 = v1[ 1 ] * p1r2 - p1r1;
		v1yTimesP0R2MinusP0R1.normalize();
		
		A << v0xTimesP0R2MinusP0R0[ 0 ], v0xTimesP0R2MinusP0R0[ 1 ], v0xTimesP0R2MinusP0R0[ 2 ], v0xTimesP0R2MinusP0R0[ 3 ],
			 v0yTimesP0R2MinusP0R1[ 0 ], v0yTimesP0R2MinusP0R1[ 1 ], v0yTimesP0R2MinusP0R1[ 2 ], v0yTimesP0R2MinusP0R1[ 3 ],
			 v1xTimesP0R2MinusP0R0[ 0 ], v1xTimesP0R2MinusP0R0[ 1 ], v1xTimesP0R2MinusP0R0[ 2 ], v1xTimesP0R2MinusP0R0[ 3 ],
			 v1yTimesP0R2MinusP0R1[ 0 ], v1yTimesP0R2MinusP0R1[ 1 ], v1yTimesP0R2MinusP0R1[ 2 ], v1yTimesP0R2MinusP0R1[ 3 ];
		
		return A;
	}
	
	MatrixXd TriangulationDlt::buildSolutionMatrix( VectorXd& solution ) const
	{
		MatrixXd formatedSolution( 4, 1 );
		formatedSolution << solution[ 0 ], solution[ 1 ], solution[ 2 ], solution[ 3 ];
		
		return formatedSolution;
	}
	
	void TriangulationDlt::denormalize() {}
	
	void TriangulationDlt::onDenormalizationEnd()
	{
		( *m_resultH ) = ( *m_resultH ) / ( *m_resultH )( 3, 0 );
	}
	
	VectorXd TriangulationDlt::getPoint3D()
	{
		if( m_resultH == nullptr )
		{
			solve();
		}
		
		VectorXd point3D( 4 );
		point3D[0] = ( *m_resultH )( 0, 0 ); point3D[1] = ( *m_resultH )( 1, 0 ); point3D[2] = ( *m_resultH )( 2, 0 );
		point3D[3] = ( *m_resultH )( 3, 0 );
		return point3D;
	}
	
	int TriangulationDlt::scoreSolution( shared_ptr< vector< Correspondence > > allCorrespondences )
	{
		throw logic_error( "ScoreSolution is undefined for TriangulationDlt." );
	}
}