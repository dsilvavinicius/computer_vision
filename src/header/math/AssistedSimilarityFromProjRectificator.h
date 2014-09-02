#ifndef PROJECTION_RECTIFICATOR_H
#define PROJECTION_RECTIFICATOR_H

#include "IPointMapper.h"

using namespace std;
using namespace Eigen;

namespace math
{
	// TODO: Change the name of this class to SimilarityRectificator, since it is leading image from projection
	// space to similarity space.
	class AssistedSimilarityFromProjRectificator : public IPointMapper
	{
	public:
		AssistedSimilarityFromProjRectificator(const vector<pair<VectorXd, VectorXd>>& correlations);
		shared_ptr<MatrixXd> buildTransformation(const vector<pair<VectorXd, VectorXd>>& correlations);
	private:
		/** Checks if the built transformation makes sense. */
		void sanityCheck(const vector<pair<VectorXd, VectorXd>>& correlations, double error);
	};
}

#endif // PROJECTION_RECTIFICATOR_H
