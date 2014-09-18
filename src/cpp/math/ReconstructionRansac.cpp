#include "ReconstructionRansac.h"
#include "EssentialMatrixDlt.h"

namespace math
{
	ReconstructionRansac::ReconstructionRansac( shared_ptr< vector< Correspondence > > correspondences, MatrixXd& K0,
												MatrixXd& K1 )
	: Ransac< Correspondence, EssentialMatrixDlt >( correspondences, 8 ),
	m_K0( make_shared< MatrixXd >( K0 ) ),
	m_K1( make_shared< MatrixXd >( K1 ) )
	{}
	
	EssentialMatrixDlt ReconstructionRansac::createSolver( vector< Correspondence >& sample )
	{
		return EssentialMatrixDlt( sample, m_K0, m_K1 );
	}
}