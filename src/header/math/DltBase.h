#ifndef DLT_BASE_H
#define DLT_BASE_H

#include <memory>
#include <Eigen/Dense>
#include "Correspondence.h"

using namespace std;
using namespace Eigen;

namespace math
{
	/** Base class of implementers of the Direct Linear Transformation algorithm, to solve overdetermined linear systems in
	 * the form Ax = 0, generated by correlations of points. This implementation normalizes the system before computing the
	 * results and denormalizes it afterwards. The virtual exported API let modification on parts of the algorithm.
	 */
	class DltBase
	{
	public:
		/** Inits this Dlt, given the sample of correspondences. */
		DltBase( vector< Correspondence > sample );
		
		/** Computes the results of the generated linear system the system is normalized before computing results and
		 * denormalized afterwards. */
		MatrixXd solve();
		
		/** Scores the solution of this DLT instance with the number of inliers in the given set of
		 * correspondences. */
		virtual int scoreSolution( vector< Correspondence > allCorrespondences ) = 0;
	
	protected:
		/** Creates the linear system to solve. */
		virtual MatrixXd createLinearSystem() = 0;
		
		/** Applies restrions on the linear system result before denormalization. At the time this method is called m_resultH
		 * already has the current normalized solution of the linear system. Default implementation does nothing. */
		virtual void applyRestrictions();
		
		/** Called after the denormalization is done (m_resultH will have the denormalized result at this time). Default
		 * implementation does nothing. */
		virtual void onDenormalizationEnd();
	
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