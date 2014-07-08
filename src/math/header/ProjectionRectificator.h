#ifndef PROJECTION_RECTIFICATOR_H
#define PROJECTION_RECTIFICATOR_H

#include "IPointMapper.h"

using namespace std;
using namespace Eigen;

namespace math
{
	class ProjectionRectificator : public IPointMapper
	{
	public:
		ProjectionRectificator(vector<pair<VectorXd, VectorXd>> correlations);
		virtual shared_ptr<MatrixXd> buildTransformation(vector<pair<VectorXd, VectorXd>> correlations);
		virtual shared_ptr<MatrixXd> getTransformation();
	};
}

#endif // PROJECTION_RECTIFICATOR_H
