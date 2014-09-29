#ifndef RECONSTRUCTION_CONTROLLER_H
#define RECONSTRUCTION_CONTROLLER_H

#include <vector>
#include <Eigen/Dense>
#include "Correspondence.h"
#include <CameraMatrixDlt.h>
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
		
		static vector< MatrixXd > readCalibrationMatrices( vector< string >& camFileNames );
		
		vector< map< int, VectorXd > > readPointCorrespondence( const vector< string >& pointFileNames,
																const string& correspondenceFileName );
		
		vector< Correspondence > restrictPointCorrespondencesToImgs( const vector< map< int, VectorXd > >& pointMap,
																	 const int& imgIdx0, const int& imgIdx1 );
		
		/** Reads the file with line correspondences. */
		static vector< map< int, Line > > readLineCorrespondence( const vector< string >& lineFileNames,
																  const string& correspondenceFileName );
		
		/** Converts the line correspondence vector to a point correspondence vector. */
		static vector< Correspondence > lineCorrespToPointCorresp( const vector< map< int, Line > >& lineCorresp, const int& imgIdx0,
																   const int& imgIdx1 );
		
		/** Reads the line correspondences and converts them to point correspondences. */
		static vector< Correspondence > readLineCorrespConvertPointCorresp( const vector< string >& lineFileNames,
																			const string& correspondenceFileName,
																			const int& imgIdx0, const int& imgIdx1 );
		
		/** Normalizes the correspondences in respect of camera calibration. */
		static vector< Correspondence > normalizeWithCamCalibration( const vector< Correspondence >& correspondences,
																	 const MatrixXd& K0, const MatrixXd& K1 );
		
		/** Reconstructs the 3d points. */
		shared_ptr< vector< VectorXd > > reconstruct();
		
		shared_ptr< ReconstructionRansac > getRansac();
	private:
		shared_ptr< vector< Correspondence > > m_correspondences;
		shared_ptr< ReconstructionRansac > m_ransac;
	};
}

#endif