#ifndef RECONSTRUCTION_RANSAC_H
#define RECONSTRUCTION_RANSAC_H

#include "Ransac.h"
#include "CameraMatrixDlt.h"

namespace math
{
	class ReconstructionRansac : public Ransac< Correspondence, CameraMatrixDlt >
	{
	public:
		ReconstructionRansac( shared_ptr< vector< Correspondence > > correspondences, MatrixXd& K0, MatrixXd& K1  );
	protected:
		virtual CameraMatrixDlt createSolver( vector< Correspondence >& sample );
	private:
		shared_ptr< MatrixXd > m_K0;
		shared_ptr< MatrixXd > m_K1;
	};
}

#endif