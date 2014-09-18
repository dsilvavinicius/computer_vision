#include "ReconstructionController.h"
#include <Ransac.h>
#include <TriangulationDlt.h>
#include <ReconstructionRansac.h>

namespace model
{
	ReconstructionController::ReconstructionController( vector< Correspondence >& correspondences, MatrixXd& K0, MatrixXd& K1 )
	: m_correspondences( make_shared< vector< Correspondence > >( correspondences ) )
	{
		if( correspondences.size() < 8 ) { throw logic_error("8 correspondences needed to compute essential matrix."); }
		m_ransac = make_shared< ReconstructionRansac >( make_shared< vector< Correspondence > >( correspondences ), K0, K1 );
	}

	vector< VectorXd > ReconstructionController::reconstruct()
	{
		MatrixXd E = m_ransac->compute();
		MatrixXd P = computeP( E );
	}
	
	MatrixXd ReconstructionController::computeP( MatrixXd& E )
	{
		JacobiSVD< MatrixXd > svd( E, ComputeFullU | ComputeFullV );
		MatrixXd U = svd.matrixU();
		MatrixXd Vt = svd.matrixV().transpose();
		MatrixXd W( 3 , 3);
		W << 0., -1., 0.,
			 1., 0., 0.,
			 0., 0., 1.;
		VectorXd u3 = U.col(2);
		
		MatrixXd PnoT = U * W * Vt; // P without translation part.
		
		MatrixXd P( 3, 4 );
		P << PnoT( 0, 0 ), PnoT( 0, 1 ), PnoT( 0, 2 ), u3[ 0 ],
			 PnoT( 1, 0 ), PnoT( 1, 1 ), PnoT( 1, 2 ), u3[ 1 ],
			 PnoT( 2, 0 ), PnoT( 2, 1 ), PnoT( 2, 2 ), u3[ 2 ];
		
		/*TriangulationDlt dlt(  );
		if()
		{
		}*/
	}
}