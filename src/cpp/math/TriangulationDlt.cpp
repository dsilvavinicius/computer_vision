#include "TriangulationDlt.h"

namespace math
{
	TriangulationDlt::TriangulationDlt( vector< Correspondence>& correspondence, MatrixXd& P0, MatrixXd& P1)
	: DltBase( correspondence ),
	m_P0( P0 ),
	m_P1( P1 )
	{}
	
	MatrixXd TriangulationDlt::createLinearSystem() const
	{
		MatrixXd A( 4, 4 );
		
		VectorXd v0 = (*m_sample)[0].first;
		VectorXd v1 = (*m_sample)[0].second;
		
		VectorXd p0r0 = m_P0.row( 0 );
		VectorXd p0r1 = m_P0.row( 1 );
		VectorXd p0r2 = m_P0.row( 2 );
		VectorXd p0r3 = m_P0.row( 3 );
		
		VectorXd p1r0 = m_P1.row( 0 );
		VectorXd p1r1 = m_P1.row( 1 );
		VectorXd p1r2 = m_P1.row( 2 );
		VectorXd p1r3 = m_P1.row( 3 );
		
		VectorXd v0xTimesP0R3MinusP0R1 = v0[ 0 ] * p0r3 - p0r1;
		VectorXd v0yTimesP0R3MinusP0R2 = v0[ 1 ] * p0r3 - p0r2;
		VectorXd v1xTimesP0R3MinusP0R1 = v1[ 0 ] * p1r3 - p1r1;
		VectorXd v1yTimesP0R3MinusP0R2 = v1[ 1 ] * p1r3 - p1r2;
		
		A << v0xTimesP0R3MinusP0R1[ 0 ], v0xTimesP0R3MinusP0R1[ 1 ], v0xTimesP0R3MinusP0R1[ 2 ], v0xTimesP0R3MinusP0R1[ 3 ],
			 v0yTimesP0R3MinusP0R2[ 0 ], v0yTimesP0R3MinusP0R2[ 1 ], v0yTimesP0R3MinusP0R2[ 2 ], v0yTimesP0R3MinusP0R2[ 3 ],
			 v1xTimesP0R3MinusP0R1[ 0 ], v1xTimesP0R3MinusP0R1[ 1 ], v1xTimesP0R3MinusP0R1[ 2 ], v1xTimesP0R3MinusP0R1[ 3 ],
			 v1yTimesP0R3MinusP0R2[ 0 ], v1yTimesP0R3MinusP0R2[ 1 ], v1yTimesP0R3MinusP0R2[ 2 ], v1yTimesP0R3MinusP0R2[ 3 ];
		
		return A;
	}
	
	void TriangulationDlt::onDenormalizationEnd()
	{
		
	}
	
	int TriangulationDlt::scoreSolution( shared_ptr< vector< Correspondence > > allCorrespondences ) {}
}