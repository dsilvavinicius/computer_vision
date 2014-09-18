#include "EssentialMatrixDlt.h"

namespace math
{
	MatrixXd EssentialMatrixDlt::createLinearSystem()
	{
		int sampleSize = m_sample->size();
		MatrixXd A( sampleSize, 9 );
		
		for( int i = 0; i < sampleSize; ++i)
		{
			Block< MatrixXd > block = A.block(i, 0, 1, 9);
			VectorXd v0 = (*m_sample)[i].first;
			VectorXd v1 = (*m_sample)[i].second;
			
			block 	<< v1[ 0 ] * v0[ 0 ], v1[ 0 ] * v0[ 1 ], v1[0], v1[ 1 ] * v0[ 0 ], v1[ 1 ] * v0[ 1 ], v1[ 1 ],
					   v0[ 0 ], v0[ 1 ], 1.;
		}
		
		return A;
	}
	
	void EssentialMatrixDlt::applyRestrictions()
	{
		JacobiSVD< MatrixXd > Fsvd( *m_resultH, ComputeFullU | ComputeFullV );
		VectorXd singularValues = Fsvd.singularValues();
		DiagonalMatrix< double, 3, 3 > D( singularValues[ 0 ], singularValues[ 1 ], 0. );
		
		*m_resultH = Fsvd.matrixU() * D * Fsvd.matrixV().transpose();
	}
	
	void EssentialMatrixDlt::onDenormalizationEnd()
	{
		
	}
	
	int EssentialMatrixDlt::scoreSolution( vector< Correspondence > allCorrespondences )
	{
		
	}
}