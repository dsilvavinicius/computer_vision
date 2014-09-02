#ifndef SIMILARITY_FROM_PROJ_RECTIFICATOR_H
#define SIMILARITY_FROM_PROJ_RECTIFICATOR_H

#include "IPointMapper.h"

namespace math
{
	/** Rectifies an image in projection space to similarity space. Needs 5 pairs of lines that should be orthogonal in similarity space. */
	class SimilarityFromProjRectificator : public IPointMapper
	{
	public:
		SimilarityFromProjRectificator(const vector<pair<VectorXd, VectorXd>>& orthoLines);
		
		/** Given 5 lines in projection space that should be orthogonal in similarity space, rectifies the image to similarity space. */
		shared_ptr<MatrixXd> buildTransformation(const vector<pair<VectorXd, VectorXd>>& orthoLines);
		
		void sanityCheck(const vector<pair<VectorXd, VectorXd>>& orthoLines, double error);
	};
}

#endif // SIMILARITY_FROM_PROJ_RECTIFICATOR_H
