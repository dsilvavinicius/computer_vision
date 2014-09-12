#ifndef DLT_H
#define DLT_H

#include <memory>
#include <utility>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace math
{
	using Correspondence = pair< VectorXd, VectorXd >;
	ostream& operator<<( ostream& out, const Correspondence& correspondence );
	ostream& operator<<( ostream& out, const vector<Correspondence>& correspondences );
	
	/** Implements the algorithm Direct Linear Transformation, to solve overdetermined linear systems in the form Ax = 0,
	 * generated by correlations of points. This implementation normalizes the system before computing the results and
	 * denormalizes it afterwards.
	 */
	class Dlt
	{
	public:
		/** Inits this Dlt, given the sample of correspondences. */
		Dlt( vector< Correspondence > sample );
		
		/** Computes the results of the generated linear system the system is normalized before computing results and
		 * denormalized afterwards. */
		MatrixXd solve();
		
		/** Scores the solution of this DLT instance with the number of inliers in the given set of
		 * correspondences. */
		int scoreSolution( vector< Correspondence > allCorrespondences);
	private:
		/** Normalizes the points in the sets S0 and S1 which are the sets being corresponded, resulting in one normalization
		 * matrix for each set. The normalization puts the centroid of the point set at origin and scales the space as the
		 * mean distance from origin is sqrt(2). */
		void normalize();
		
		/** Denormalizes homography H that maps points from first set S0 to second set S1. */
		void denormalize();
		
		shared_ptr< MatrixXd > m_S0Normalizer;
		shared_ptr< MatrixXd > m_S1Normalizer;
		shared_ptr< MatrixXd > m_resultH;
		shared_ptr< vector< Correspondence > > m_sample;
	};
}

#endif