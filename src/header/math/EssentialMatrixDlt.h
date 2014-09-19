#ifndef RECONSTRUCTION_DLT_H
#define RECONSTRUCTION_DLT_H
#include "DltBase.h"

namespace math
{
	/** DLT to calculate the camera matrices of two images, given correspondence points in the images and the camera
	 * calibration matrices. Camera 0 is assumed to be at origin and not rotated. */
	class EssentialMatrixDlt : public DltBase
	{
	public:
		/** Builds this DLT, given the correspondence vector and camera calibration matrices. */
		EssentialMatrixDlt( vector< Correspondence >& correspondences, shared_ptr< MatrixXd > K0, shared_ptr< MatrixXd > K1 );
		
		int scoreSolution( shared_ptr< vector< Correspondence > > allCorrespondences ) const;
		
		/** Calculates the essential matrix after fundamental matrix normalization. */
		void onDenormalizationEnd();
	
	protected:
		/** Creates the linear system for the fundamental matrix computation (the essential matrix is computed after
		 * denormalization of the fundamental matrix result). */
		MatrixXd createLinearSystem() const;
		
		/** Applies the fundamental matrix restrictions. */
		void applyRestrictions();
	private:
		/** Computes camera's P' matrix, checking all the 4 possible solution and choosing the one which reconstructs points
		 * in front of both cameras. */
		MatrixXd computeP( MatrixXd& E );
		
		/** The camera matrix P1 have 4 possible solutions when calculated from the camera Essential Matrix. The correct
		 * solution will be in front of both cameras (represented by P0 and P1). This method checks if one of these solution
		 * is the correct. */
		bool checkP1( const MatrixXd& P0, const MatrixXd& P1 ) const;
		
		/** Calibration matrix for camera 0. */
		shared_ptr< MatrixXd > m_K0;
		
		/** Calibration matrix for camera 1. */
		shared_ptr< MatrixXd > m_K1;
		
		/** Solution matrix for camera 0. */
		shared_ptr< MatrixXd > m_P0;
	};
}

#endif