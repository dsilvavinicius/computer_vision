#include "ReconstructionRansac.h"
#include "CameraMatrixDlt.h"

namespace math
{
	ReconstructionRansac::ReconstructionRansac( shared_ptr< vector< Correspondence > > correspondences, MatrixXd& K0,
												MatrixXd& K1 )
	: Ransac< Correspondence, CameraMatrixDlt >( correspondences, 8 ),
	m_K0( make_shared< MatrixXd >( K0 ) ),
	m_K1( make_shared< MatrixXd >( K1 ) )
	{}
	
	CameraMatrixDlt ReconstructionRansac::createSolver( vector< Correspondence >& sample )
	{
		return CameraMatrixDlt( sample, m_K0, m_K1 );
	}
}