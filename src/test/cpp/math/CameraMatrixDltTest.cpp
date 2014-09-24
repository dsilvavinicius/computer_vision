#include "CameraMatrixDlt.h"
#include "TriangulationDlt.h"

#include <gtest/gtest.h>

using namespace math;

extern "C" int g_argc;
extern "C" char** g_argv;

namespace math
{
	namespace test
	{
        class CameraMatrixDltTest : public ::testing::Test
		{
		protected:
			void SetUp()
			{
				m_K0 = MatrixXd( 3, 3 );
				m_K0 << 468.2, 91.2, 300.0,
					0., 427.2, 200.0,
					0., 0., 1.;
				
				m_P0 = MatrixXd::Zero( 3, 4 ); // No rotation, camera center at origin.
				m_P0.block( 0, 0, 3, 3 ) = m_K0;
					
				m_K1 = MatrixXd( 3, 3 );
				m_K1 << 652.2, 5.4, 105.0,
					0., 805.2, 351.0,
					0., 0., 1.;
				
				double angleRad = 3.14159265359 * 0.25; // 45 degrees.
				MatrixXd R1( 3, 3 ); // Rotation around x axis.
				R1 << 1., 0., 0.,
					0., cos( angleRad ), -sin( angleRad ),
					0., sin( angleRad ), cos( angleRad );
				
				m_P1 = MatrixXd( 3, 4 );
				m_P1.block( 0, 0, 3, 3 ) = m_K1 * R1;
				m_P1.block( 0, 3, 3, 1 ) << 10., 15., 5.; // Cam translation
				
				srand( 1 );
			}
			
			MatrixXd m_K0;
			MatrixXd m_K1;
			MatrixXd m_P0;
			MatrixXd m_P1;
		};
		
		TEST_F( CameraMatrixDltTest, DISABLED_Triangulation )
		{
			for( int i = 0; i < 1000; ++i )
			{
				VectorXd point3D( 4 ); point3D << rand() % 100 + 1, rand() % 100 + 1, rand() % 100 + 1, 1.;
				
				VectorXd p0 = m_P0 * point3D;
				VectorXd p1 = m_P1 * point3D;
				
				p0 = p0 / p0[ 2 ];
				p1 = p1 / p1[ 2 ];
				
				vector< Correspondence > correspondence( 1 );
				correspondence[ 0 ] = Correspondence( p0, p1 );
				
				TriangulationDlt dlt( correspondence, m_P0, m_P1 );
				dlt.solve();
				
				VectorXd triangulatedPoint = dlt.getPoint3D();
				/*cout << "Correspondence: " << endl << p0 << endl << p1 << endl << endl
					 << "Expected 3d point:" << endl << point3D << endl << endl
					 << "Triangulated point:" << endl << triangulatedPoint << endl << endl;*/
				
				ASSERT_TRUE( point3D.isApprox( triangulatedPoint ) );
			}
		}
		
		TEST_F( CameraMatrixDltTest, Solving )
		{	
			int numPoints = 8;
			vector< VectorXd > points3D( numPoints );
			vector< Correspondence > correspondences( numPoints );
			for( int i = 0; i < numPoints; ++i )
			{
				VectorXd point3D( 4 ); point3D << rand() % 100, rand() % 100, rand() % 100, 1.;
				
				points3D[ i ] = point3D;
				
				VectorXd p0 = m_P0 * point3D; p0 = p0 / p0[ 2 ];
				VectorXd p1 = m_P1 * point3D; p1 = p1 / p1[ 2 ];
				
				cout << "Generated test point: " << endl << point3D << endl << endl
					 << "Img0: " << endl << p0 << endl << endl
					 << "Img1: " << endl << p1 << endl << endl;
				
				correspondences[ i ] = Correspondence( p0, p1 );
			}
			
			shared_ptr< MatrixXd > pK0 = make_shared< MatrixXd >( m_K0 );
			shared_ptr< MatrixXd > pK1 = make_shared< MatrixXd >( m_K1 );
			CameraMatrixDlt dlt( correspondences, pK0, pK1 );
			
			MatrixXd resultP1 = dlt.solve();
			MatrixXd resultP0 = dlt.getP0();
			
			cout << "Expected P0: " << endl << m_P0 << endl << endl
				 << "Result P0: " << endl << resultP0 << endl << endl
				 << "Expected P1: " << endl << m_P1 << endl << endl
				 << "Result P1: " << endl << resultP1 << endl << endl;
			
			ASSERT_TRUE( resultP1.isApprox( m_P0 ) );
			ASSERT_TRUE( resultP0.isApprox( m_P1 ) );
		}
	}
}