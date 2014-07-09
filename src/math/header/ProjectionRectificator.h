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
		ProjectionRectificator(vector<pair<VectorXd, VectorXd>> const correlations);
		shared_ptr<MatrixXd> buildTransformation(vector<pair<VectorXd, VectorXd>> const correlations);
		shared_ptr<MatrixXd> getTransformation();
	private:
		/** Checks if the built transformation makes sense. */
		void sanityCheck(vector<pair<VectorXd, VectorXd>> const correlations, double error);
	};
}

#endif // PROJECTION_RECTIFICATOR_H
