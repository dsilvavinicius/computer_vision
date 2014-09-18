#include "Ransac.h"

namespace math
{
	template <>
	EssentialMatrixDlt Ransac< Correspondence, EssentialMatrixDlt >::createSolver( vector< Correspondence >& sample )
	{
		return EssentialMatrixDlt( sample, make_shared< MatrixXd >(1, 1), make_shared< MatrixXd >(1, 1) );
	}
}