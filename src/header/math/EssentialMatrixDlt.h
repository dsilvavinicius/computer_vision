#ifndef RECONSTRUCTION_DLT_H
#define RECONSTRUCTION_DLT_H
#include "DltBase.h"

namespace math
{
	/** DLT to calculate the essential matrix. */
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
		shared_ptr< MatrixXd > m_K0;
		shared_ptr< MatrixXd > m_K1;
	};
}

#endif