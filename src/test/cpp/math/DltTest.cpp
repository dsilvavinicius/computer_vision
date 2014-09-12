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
		TEST_F( DltTest, DISABLED_SimpleTransform )
		{
			// The transformation is a rotation around z axis, a translation and an anisotropic scale.
			MatrixXd H(3, 3);
			double rotationAngle = 3.14159265 * 0.25; // 45 degrees.
			double scaleX = 2;
			double scaleY = 3;
			double translationX = 5;
			double translationY = 10;
			H << cos( rotationAngle ) * scaleX, -sin( rotationAngle ) * scaleY, translationX,
				 sin( rotationAngle ) * scaleX, cos( rotationAngle ) * scaleY, translationY,
				 0., 0., 1.;
			VectorXd p0i0(3); p0i0[0] = 23; p0i0[1] = 45; p0i0[2] = 1;
			VectorXd p0i1 = H * p0i0;
			VectorXd p1i0(3); p1i0[0] = 10; p1i0[1] = 139; p1i0[2] = 1;
			VectorXd p1i1 = H * p1i0;
			VectorXd p2i0(3); p2i0[0] = 85; p2i0[1] = 29; p2i0[2] = 1;
			VectorXd p2i1 = H * p2i0;
			VectorXd p3i0(3); p3i0[0] = 526; p3i0[1] = 640; p3i0[2] = 1;
			VectorXd p3i1 = H * p3i0;
			
			vector< Correspondence > correspondences;
			correspondences.push_back( Correspondence( p0i0, p0i1 ) );
			correspondences.push_back( Correspondence( p1i0, p1i1 ) );
			correspondences.push_back( Correspondence( p2i0, p2i1 ) );
			correspondences.push_back( Correspondence( p3i0, p3i1 ) );
			
			Dlt dlt( correspondences );
			MatrixXd dltSolution = dlt.solve();
			
			VectorXd transfP0 = dltSolution * p0i0; transfP0 = transfP0 / transfP0[2];
			VectorXd transfP1 = dltSolution * p1i0; transfP1 = transfP1 / transfP1[2];
			VectorXd transfP2 = dltSolution * p2i0; transfP2 = transfP2 / transfP2[2];
			VectorXd transfP3 = dltSolution * p3i0; transfP3 = transfP3 / transfP3[2];
			
			cout << "Dlt solution: " << endl << dltSolution << endl << endl
				 << "Original H: " << endl << H << endl << endl
				 << "p0':" << endl << p0i1 << endl << endl << " transfP0: " << endl << transfP0 << endl << endl
				 << "p1':" << endl << p1i1 << endl << endl << " transfP1: " << endl << transfP1 << endl << endl
				 << "p2':" << endl << p2i1 << endl << endl << " transfP2: " << endl << transfP2 << endl << endl
				 << "p3':" << endl << p3i1 << endl << endl << " transfP3: " << endl << transfP3 << endl;
				 
			ASSERT_TRUE( ( p0i1 ).isApprox( transfP0 ) );
			ASSERT_TRUE( ( p1i1 ).isApprox( transfP1 ) );
			ASSERT_TRUE( ( p2i1 ).isApprox( transfP2 ) );
			ASSERT_TRUE( ( p3i1 ).isApprox( transfP3 ) );
		}
		
		TEST_F( DltTest, DISABLED_RealExample )
		{
			VectorXd p0i0(3); p0i0[0] = 139.346; p0i0[1] = 77.1379; p0i0[2] = 1;
			VectorXd p0i1(3); p0i1[0] = 262.767; p0i1[1] = 161.243; p0i1[2] = 1;
			
			VectorXd p1i0(3); p1i0[0] = 170.035; p1i0[1] = 188.698; p1i0[2] = 1;
			VectorXd p1i1(3); p1i1[0] = 248.832; p1i1[1] = 167.962; p1i1[2] = 1;
			
			VectorXd p2i0(3); p2i0[0] = 351; p2i0[1] = 79; p2i0[2] = 1;
			VectorXd p2i1(3); p2i1[0] = 163.814; p2i1[1] = 174.182; p2i1[2] = 1;
			
			VectorXd p3i0(3); p3i0[0] = 203.213; p3i0[1] = 105.754; p3i0[2] = 1;
			VectorXd p3i1(3); p3i1[0] = 118.195; p3i1[1] = 248.832; p3i1[2] = 1;
			
			vector< Correspondence > correspondences;
			correspondences.push_back( Correspondence( p0i0, p0i1 ) );
			correspondences.push_back( Correspondence( p1i0, p1i1 ) );
			correspondences.push_back( Correspondence( p2i0, p2i1 ) );
			correspondences.push_back( Correspondence( p3i0, p3i1 ) );
			
			Dlt dlt( correspondences );
			MatrixXd dltSolution = dlt.solve();
			
			VectorXd transfP0 = dltSolution * p0i0; transfP0 = transfP0 / transfP0[2];
			VectorXd transfP1 = dltSolution * p1i0; transfP1 = transfP1 / transfP1[2];
			VectorXd transfP2 = dltSolution * p2i0; transfP2 = transfP2 / transfP2[2];
			VectorXd transfP3 = dltSolution * p3i0; transfP3 = transfP3 / transfP3[2];
			
			cout << "Dlt solution: " << endl << dltSolution << endl << endl
				 << "p0':" << endl << p0i1 << endl << endl << " transfP0: " << endl << transfP0 << endl << endl
				 << "p1':" << endl << p1i1 << endl << endl << " transfP1: " << endl << transfP1 << endl << endl
				 << "p2':" << endl << p2i1 << endl << endl << " transfP2: " << endl << transfP2 << endl << endl
				 << "p3':" << endl << p3i1 << endl << endl << " transfP3: " << endl << transfP3 << endl;
				 
			ASSERT_TRUE( ( p0i1 ).isApprox( transfP0 ) );
			ASSERT_TRUE( ( p1i1 ).isApprox( transfP1 ) );
			ASSERT_TRUE( ( p2i1 ).isApprox( transfP2 ) );
			ASSERT_TRUE( ( p3i1 ).isApprox( transfP3 ) );
		}
	}
}