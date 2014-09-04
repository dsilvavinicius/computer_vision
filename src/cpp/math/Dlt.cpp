#include "Dlt.h"

namespace math
{
	Dlt::Dlt() {}
	
	VectorXd Dlt::compute( vector< Correspondence > correlations )
	{
		MatrixXd A( 8, 9 );
		for( int i = 0; i < correlations.size(); ++i)
		{
			Block< MatrixXd > block = A.block(i * 2, 0, 2, 9);
			VectorXd v0 = correlations[i].first;
			VectorXd v1 = correlations[i].second;
			
			block 	<< 0. , 0. , 0. , -v1[2] * v0[0] , -v1[2] * v0[1] , -v1[2] * v0[2] , v1[1] * v0[0] , v1[1] * v0[1] , v1[1] * v0[2],
					v1[2] * v0[0] , v1[2] * v0[1] , v1[2] * v0[2] , 0. , 0. , 0. , -v1[0] * v0[0] , -v1[0] * v0[1] , -v1[0] * v0[2];
		}
		
		JacobiSVD<MatrixXd> svd(A, ComputeThinV);
		return svd.matrixV().col(7);
	}
}