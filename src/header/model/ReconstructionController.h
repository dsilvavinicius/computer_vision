#ifndef RECONSTRUCTION_CONTROLLER_H
#define RECONSTRUCTION_CONTROLLER_H

#include <vector>
#include <Eigen/Dense>
#include "Correspondence.h"
#include <EssentialMatrixDlt.h>
#include <ReconstructionRansac.h>

using namespace Eigen;
using namespace std;
using namespace math;

namespace model
{
	
	class ReconstructionController
	{
	public:
		/** Build the reconstructor of 3d points, given the correspondences of projected images and the camera calibration
		* matrices for each image. */
		ReconstructionController( vector< Correspondence >& correspondences, MatrixXd& K0, MatrixXd& K1 );
		
		/** Reconstructs the 3d points. */
		shared_ptr< vector< VectorXd > > reconstruct();
	private:
		shared_ptr< vector< Correspondence > > m_correspondences;
		shared_ptr< ReconstructionRansac > m_ransac;
	};
}

#endif