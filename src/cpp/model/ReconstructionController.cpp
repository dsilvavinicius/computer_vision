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
}