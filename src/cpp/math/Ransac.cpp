#include "Ransac.h"

namespace math
{
	template <>
	CameraMatrixDlt Ransac< Correspondence, CameraMatrixDlt >::createSolver( vector< Correspondence >& sample )
	{
		return CameraMatrixDlt( sample, make_shared< MatrixXd >(1, 1), make_shared< MatrixXd >(1, 1) );
	}
}