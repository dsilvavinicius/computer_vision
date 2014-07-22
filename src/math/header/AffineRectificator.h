#ifndef AFFINE_RECTIFICATOR_H
#define AFFINE_RECTIFICATOR_H

#include "IPointMapper.h"

namespace math
{
	/** Builds the affine rectification transformation, given two pairs of lines in projection space that
	 * should be parallel in affine space. */
	class AffineRectificator : public IPointMapper
	{
	public:
		AffineRectificator(vector<pair<VectorXd, VectorXd>> parallelLines);
		shared_ptr<MatrixXd> buildTransformation(const vector<pair<VectorXd, VectorXd>>& parallelLines);
	private:
		void sanityCheck(vector<pair<VectorXd, VectorXd>> parallelLines, Vector3d lineAtInf, double error);
	};
}

#endif // AFFINE_RECTIFICATOR_H
