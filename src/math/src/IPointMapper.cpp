#include "IPointMapper.h"

namespace math
{
	shared_ptr<MatrixXd> IPointMapper::getTransformation()
	{
		return m_transformation;
	}
}