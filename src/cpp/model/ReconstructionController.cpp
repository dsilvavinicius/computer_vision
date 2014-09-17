#include "ReconstructionController.h"

namespace model
{
	ReconstructionController::ReconstructionController( vector< Correspondence >& correspondences, MatrixXd& K0, MatrixXd& K1 )
	{
		if( correspondences.size() < 8 ) { throw logic_error("8 correspondences needed to compute essential matrix."); }
		m_correspondences = normalize(correspondences, K0, K1);
	}

	vector< Correspondence > normalize( vector< Correspondence >& correspondences, MatrixXd& K0, MatrixXd& K1 )
	{
		
	}

	vector< VectorXd > ReconstructionController::reconstruct()
	{
		//Ransac< Correspondence,  >
		//MatrixXd E = ;
	}
}