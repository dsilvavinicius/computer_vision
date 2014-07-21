#ifndef PROJECTION_RECTIFICATOR_H
#define PROJECTION_RECTIFICATOR_H

#include "IPointMapper.h"

using namespace std;
using namespace Eigen;

namespace math
{
	// TODO: Change the name of this class to SimilarityRectificator, since it is leading image from projection
	// space to similarity space.
	class ProjectionRectificator : public IPointMapper
	{
	public:
		ProjectionRectificator(vector<pair<VectorXd, VectorXd>> const correlations);
		shared_ptr<MatrixXd> buildTransformation(vector<pair<VectorXd, VectorXd>> const correlations);
	private:
		/** Checks if the built transformation makes sense. */
		void sanityCheck(vector<pair<VectorXd, VectorXd>> const correlations, double error);
	};
}

#endif // PROJECTION_RECTIFICATOR_H
