#ifndef IPOINT_MAPPER_H
#define IPOINT_MAPPER_H

#include <memory>
#include <vector>
#include <utility>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace math
{
	// TODO: Change the name of this interface to ICorrelationMapper, since implementors can correlate points and
	// other entities.
	
	/** Interface for point mappers. Implementators should interpret a given set of point correlations from a defined space P0
	 * to another space P1, generating a transformation to map the points from P0 to P1. */
	class IPointMapper
	{
	public:
		/** Given a vector with point correlations from space P0 to P1, generates a transformation mapping points from P0 to
		 * P1. */
		virtual shared_ptr<MatrixXd> buildTransformation(const vector<pair<VectorXd, VectorXd>>& correlations) = 0;
		
		/** Gets the transformation generated by buildTransformation() */
		virtual shared_ptr<MatrixXd> getTransformation();
		
		/** Helper to convert VectorXd to Vector3d. Use when there is need of cross products, which are only
		 * defined for Vector3d. */
		static vector<pair<Vector3d, Vector3d>> toVector3d(const vector<pair<VectorXd, VectorXd>>& correlations);
	protected:
		shared_ptr<MatrixXd> m_transformation;
	};
}

#endif // IPOINT_MAPPER_H
