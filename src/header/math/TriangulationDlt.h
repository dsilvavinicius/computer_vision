#ifndef TRIANGULATION_DLT_H
#define TRIANGULATION_DLT_H

#include "DltBase.h"

namespace math
{
	/** DLT to reconstruct a 3D point, given a correspondence in two images generated by cameras with matrices P0 and P1. */
	class TriangulationDlt : public DltBase
	{
	public:
		TriangulationDlt( vector< Correspondence>& correspondence, MatrixXd& P0, MatrixXd& P1);
		
		int scoreSolution( shared_ptr< vector< Correspondence > > allCorrespondences );
	
	protected:
		MatrixXd createLinearSystem() const;
		
		/** Saves the final 3D point. */
		void onDenormalizationEnd();
	
	private:
		/** Camera matrix P0. */
		MatrixXd m_P0;
		
		/** Camera matrix P1. */
		MatrixXd m_P1;
		
		/** Final reconstructed 3D point. */
		VectorXd m_3DPoint;
	};
}

#endif