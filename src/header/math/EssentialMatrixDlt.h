#ifndef RECONSTRUCTION_DLT_H
#define RECONSTRUCTION_DLT_H
#include "DltBase.h"

namespace math
{
	/** DLT to calculate the essential matrix. */
	class EssentialMatrixDlt : public DltBase
	{
	public:
		int scoreSolution( vector< Correspondence > allCorrespondences );
		
		/** Calculates the essential matrix after fundamental matrix normalization. */
		void onDenormalizationEnd();
	
	protected:
		/** Creates the linear system for the fundamental matrix computation (the essential matrix is computed after
		 * denormalization of the fundamental matrix result). */
		MatrixXd createLinearSystem();
		
		/** Applies the fundamental matrix restrictions. */
		void applyRestrictions();
	};
}

#endif