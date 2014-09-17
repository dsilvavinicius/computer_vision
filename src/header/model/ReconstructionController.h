#ifndef RECONSTRUCTION_CONTROLLER_H
#define RECONSTRUCTION_CONTROLLER_H

#include <vector>
#include <Eigen/Dense>
#include "Correspondence.h"

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
		
		vector< Correspondence > normalize( vector< Correspondence >& correspondences, MatrixXd& K0, MatrixXd& K1 );
		
		/** Reconstructs the 3d points. */
		vector< VectorXd > reconstruct();
	private:
		/** Normalized correspondences. */
		vector< Correspondence > m_correspondences;
	};
}

#endif