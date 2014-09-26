#include "CameraMatrixDlt.h"
#include "TriangulationDlt.h"
#include "ReconstructionController.h"

#include <gtest/gtest.h>

using namespace math;
using namespace model;

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
				m_K0 = MatrixXd::Identity( 3, 3 );
				/*m_K0 << 468.2, 91.2, 300.0,
					0., 427.2, 200.0,
					0., 0., 1.;*/
				
				m_P0 = MatrixXd::Identity( 3, 4 ); // No rotation, camera center at origin.
				//m_P0.block( 0, 0, 3, 3 ) = m_K0;
					
				m_K1 = MatrixXd( 3, 3 );
				m_K1 << 652.2, 5.4, 105.0,
					0., 805.2, 351.0,
					0., 0., 1.;
				
				double angleRad = 3.14159265359 * 0.25; // 45 degrees.
				MatrixXd R1( 3, 3 ); // Rotation around x axis.
				R1 << 1., 0., 0.,
					0., cos( angleRad ), -sin( angleRad ),
					0., sin( angleRad ), cos( angleRad );
				
				m_P1 = MatrixXd::Identity( 3, 4 );
				m_P1.block( 0, 0, 3, 3 ) = R1;
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
		
		TEST_F( CameraMatrixDltTest, DISABLED_Solving )
		{
			// Luiz dataset.
			/*VectorXd p00( 3 ); p00 << 99.000000, 288.000000, 1.;
			VectorXd p01( 3 ); p01 << 104.000000, 308.000000, 1.;
			VectorXd p10( 3 ); p10 << 154.000000, 226.000000, 1.;
			VectorXd p11( 3 ); p11 << 116.000000, 248.000000, 1.;
			VectorXd p20( 3 ); p20 << 214.000000, 387.000000, 1.;
			VectorXd p21( 3 ); p21 << 199.000000, 394.000000, 1.;
			VectorXd p30( 3 ); p30 << 268.000000, 275.000000, 1.;
			VectorXd p31( 3 ); p31 << 230.000000, 285.000000, 1.;
			
			VectorXd p40( 3 ); p40 << 632.000000, 368.000000, 1.;
			VectorXd p41( 3 ); p41 << 657.000000, 352.000000, 1.;
			VectorXd p50( 3 ); p50 << 684.000000, 257.000000, 1.;
			VectorXd p51( 3 ); p51 << 710.000000, 230.000000, 1.;
			VectorXd p60( 3 ); p60 << 110.000000, 46.000000, 1.;
			VectorXd p61( 3 ); p61 << 56.000000, 82.000000, 1.;
			VectorXd p70( 3 ); p70 << 39.000000, 193.000000, 1.;
			VectorXd p71( 3 ); p71 << 56.000000, 223.000000, 1.;

			vector< Correspondence > correspondences( 8 );
			correspondences[ 0 ] = Correspondence( p00, p01 );
			correspondences[ 1 ] = Correspondence( p10, p11 );
			correspondences[ 2 ] = Correspondence( p20, p21 );
			correspondences[ 3 ] = Correspondence( p30, p31 );
			correspondences[ 4 ] = Correspondence( p40, p41 );
			correspondences[ 5 ] = Correspondence( p50, p51 );
			correspondences[ 6 ] = Correspondence( p60, p61 );
			correspondences[ 7 ] = Correspondence( p70, p71 );*/
			
			int numTotalImgs = 10;
			vector< string > lineFileNames( numTotalImgs );
			
			for( int i = 0; i < numTotalImgs; ++i )
			{
				stringstream lineSS;
				lineSS << "../../../src/images/house/correspondences/house.00" << i << ".lines";
				lineFileNames[ i ] =  lineSS.str();
			}
			
			string correspondenceFileName = "../../../src/images/house/correspondences/house.nview-lines";
			
			vector< Correspondence > correspondences = ReconstructionController::readLineCorrespConvertPointCorresp(
				lineFileNames, correspondenceFileName, 0, 1 );
			
			int numUsedImgs = 2;
			vector< string > camMatrixFileNames( numUsedImgs );
			for( int i = 0; i < numUsedImgs; ++i )
			{
				stringstream ss;
				ss << "../../../src/images/house/3D/house.00" << i << ".P";
				camMatrixFileNames[ i ] =  ss.str();
			}
			vector< MatrixXd > Ks = ReconstructionController::readCalibrationMatrices( camMatrixFileNames );
			shared_ptr< MatrixXd > pK0 = make_shared< MatrixXd >( Ks[ 0 ] );
			shared_ptr< MatrixXd > pK1 = make_shared< MatrixXd >( Ks[ 1 ] );
			
			cout << "=============== Camera matrices ====================" << endl << endl
				 << "K:" << endl << *pK0 << endl << endl << "K':" << endl << *pK1 << endl << endl
				 << "=============== Camera matrices end ====================" << endl << endl;
			
			vector< Correspondence > normalizedCorrespondences = ReconstructionController::normalizeWithCamCalibration(
				correspondences, *pK0, *pK1);
			
			CameraMatrixDlt dlt( normalizedCorrespondences, pK0, pK1 );
			MatrixXd resultP1 = dlt.solve();
			MatrixXd resultP0 = dlt.getP0();
			
				
			/*
			int numPoints = 8;
			vector< VectorXd > points3D( numPoints );
			vector< Correspondence > correspondences( numPoints );
			
			cout << "=========== Data generation ==============" << endl << endl;
			for( int i = 0; i < numPoints; ++i )
			{
				VectorXd point3D( 4 ); point3D << rand() % 100, rand() % 100, rand() % 100, 1.;
				
				points3D[ i ] = point3D;
				
				VectorXd p0 = m_P0 * point3D; p0 = p0 / p0[ 2 ];
				VectorXd p1 = m_P1 * point3D; p1 = p1 / p1[ 2 ];
				
				cout << "Generated test point: " << endl << point3D << endl << endl
					 << "Normalized Img0: " << endl << p0 << endl << endl
					 << "Normalized Img1: " << endl << p1 << endl << endl;
				
				correspondences[ i ] = Correspondence( p0, p1 );
			}
			cout << "=========== Data generation End ==============" << endl << endl;
			
			shared_ptr< MatrixXd > pK0 = make_shared< MatrixXd >( m_K0 );
			shared_ptr< MatrixXd > pK1 = make_shared< MatrixXd >( m_K1 );
			CameraMatrixDlt dlt( correspondences, pK0, pK1 );
			
			MatrixXd resultP1 = dlt.solve();
			MatrixXd resultP0 = dlt.getP0();
			
			cout << "=========== Results ==============" << endl << endl
				 << "Expected P0: " << endl << m_P0 << endl << endl
				 << "Result P0: " << endl << resultP0 << endl << endl
				 << "Expected P1: " << endl << m_P1 << endl << endl
				 << "Result P1: " << endl << resultP1 << endl << endl
				 << "=========== Results End ==============" << endl << endl;
			
			ASSERT_TRUE( resultP1.isApprox( m_P0 ) );
			ASSERT_TRUE( resultP0.isApprox( m_P1 ) );*/
		}
	}
}