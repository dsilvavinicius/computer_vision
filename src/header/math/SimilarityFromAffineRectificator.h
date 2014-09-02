#ifndef SIMILARITY_FROM_AFFINE_RECTIFICATOR_H
#define SIMILARITY_FROM_AFFINE_RECTIFICATOR_H

#include "IPointMapper.h"

namespace math
{
	/** Rectifies an image in affine space to similarity space. */
	class SimilarityFromAffineRectificator : public IPointMapper
	{
	public:
		SimilarityFromAffineRectificator(const vector<pair<VectorXd, VectorXd>>& orthoLines);
		shared_ptr<MatrixXd> buildTransformation(const vector<pair<VectorXd, VectorXd>>& orthoLines);
		void sanityCheck(const vector<pair<VectorXd, VectorXd>>& orthoLines, double error);
	};
}

#endif // SIMILARITY_FROM_AFFINE_RECTIFICATOR_H
