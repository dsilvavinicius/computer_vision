#include "Dlt.h"

#include <gtest/gtest.h>

namespace math
{
	namespace test
	{
        class DltTest : public ::testing::Test
		{
		protected:
			void SetUp() {}
		};

		/** Tests Dlt by creating a transform H and correlating generated points with transformed points. Test pass if the Dlt
		 * solution is the inverse of H. */
		TEST_F(DltTest, solving)
		{
			// The transformation is a rotation around z axis, a translation and an anisotropic scale.
			MatrixXd H(3, 3);
			double rotationAngle = 3.1421;
			double scaleX = 2;
			double scaleY = 3;
			double translationX = 5;
			double translationY = 10;
			H << cos( rotationAngle ) * scaleX, -sin( rotationAngle ) * scaleY, translationX,
				 sin( rotationAngle ) * scaleX, cos( rotationAngle ) * scaleY, translationY,
				 0., 0., 1.;
			VectorXd p0(3); p0[0] = 23; p0[1] = 45; p0[2] = 1;
			VectorXd p1(3); p1[0] = 10; p1[1] = 139; p1[2] = 1;
			VectorXd p2(3); p2[0] = 85; p2[1] = 29; p2[2] = 1;
			VectorXd p3(3); p3[0] = 526; p3[1] = 640; p3[2] = 1;
			
			vector< Correspondence > correspondences;
			correspondences.push_back( Correspondence( p0, H * p0 ) );
			correspondences.push_back( Correspondence( p1, H * p1 ) );
			correspondences.push_back( Correspondence( p2, H * p2 ) );
			correspondences.push_back( Correspondence( p3, H * p3 ) );
			
			Dlt dlt( correspondences );
			MatrixXd dltSolution = dlt.solve();
			MatrixXd inverseH = H.inverse();
			
			cout << "Dlt solution: " << endl << dltSolution << endl
				 << "Inverse H: " << endl << inverseH << endl;
				 
			cout << "dltSolution * H * p0: " << endl << dltSolution * H * p0;
			
			ASSERT_TRUE( p0.isApprox( dltSolution * H * p0 ) );
			ASSERT_TRUE( p1.isApprox( dltSolution * H * p1 ) );
			ASSERT_TRUE( p2.isApprox( dltSolution * H * p2 ) );
			ASSERT_TRUE( p3.isApprox( dltSolution * H * p3 ) );
		}
	}
}