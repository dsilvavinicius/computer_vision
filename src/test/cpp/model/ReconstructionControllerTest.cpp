#include "ReconstructionController.h"

#include <gtest/gtest.h>
#include <highgui.h>

extern "C" int g_argc;
extern "C" char** g_argv;

using namespace cv;

namespace model
{
	namespace test
	{
        class ReconstructionControllerTest : public ::testing::Test
		{
		protected:
			void SetUp()
			{
				m_numImgs = 10;
				m_imgs = vector< Mat >( m_numImgs );
				vector< string > lineFileNames( m_numImgs );
				
				for( int i = 0; i < m_numImgs; ++i )
				{
					stringstream lineSS;
					lineSS << "../../../src/images/house/correspondences/house.00" << i << ".lines";
					lineFileNames[ i ] =  lineSS.str();
					
					stringstream imgSS;
					imgSS << "../../../src/images/house/images/house.00" << i << ".pgm";
					m_imgs[ i ] = imread( imgSS.str() );
				}
				
				string correspondenceFileName = "../../../src/images/house/correspondences/house.nview-lines";
				
				m_lineCorrespondences = ReconstructionController::readLineCorrespondence( lineFileNames,
																						  correspondenceFileName );
			}
			
			int m_numImgs;
			vector< Mat > m_imgs;
			vector< map< int, Line > > m_lineCorrespondences;
		};

		/** Tests the line reading for reconstruction. Checks the first and last correspondences of the first line and the
		 * first correspondence of the last line */
		TEST_F( ReconstructionControllerTest, DISABLED_LineReading )
		{
			VectorXd expected( 3 ); expected << 330.21, 283.428, 1.;
			ASSERT_TRUE( m_lineCorrespondences[ 0 ][ 0 ].first.isApprox( expected, 1.0e-6 ) );
			expected << 594.949, 219.914, 1.;
			ASSERT_TRUE( m_lineCorrespondences[ 0 ][ 0 ].second.isApprox( expected, 1.0e-6 ) );
			
			expected << 312.82, 275.179, 1.;
			ASSERT_TRUE( m_lineCorrespondences[ 0 ][ 9 ].first.isApprox( expected, 1.0e-6 ) );
			expected << 349.38, 311.619, 1.;
			ASSERT_TRUE( m_lineCorrespondences[ 0 ][ 9 ].second.isApprox( expected, 1.0e-6 ) );
			
			expected << 582.008, 411.538, 1.;
			ASSERT_TRUE( m_lineCorrespondences[ 29 ][ 0 ].first.isApprox( expected, 1.0e-6 ) );
			expected << 656.567, 422.65, 1.;
			ASSERT_TRUE( m_lineCorrespondences[ 29 ][ 0 ].second.isApprox( expected, 1.0e-6 ) );
			
			// Rendering correspondences.
			for( map< int, Line > correspondences : m_lineCorrespondences )
			{
				for( int i = 0; i < m_numImgs; ++i )
				{
					auto it = correspondences.find( i );
					if( it != correspondences.end() )
					{
						Line line = it->second;
						Point p0( line.first[0], line.first[1] );
						Point p1( line.second[0], line.second[1] );
						
						cv::line( m_imgs[i], p0, p1, Scalar( 0, 0, 255 ), 2, 8 );
					}
				}
			}
			
			for( int i = 0; i < m_numImgs; ++i )
			{
				Mat img = m_imgs[ i ];
				stringstream ss;
				ss << "House " << i;
				imshow( ss.str(), img );
			}
			waitKey();
		}
		
		TEST_F( ReconstructionControllerTest, DISABLED_Reconstruction )
		{
			vector< string > camMatrixFileNames( m_numImgs );
			for( int i = 0; i < m_numImgs; ++i )
			{
				stringstream ss;
				ss << "../../../src/images/house/3D/house.00" << i << ".P";
				camMatrixFileNames[ i ] =  ss.str();
			}
			vector< MatrixXd > Ks = ReconstructionController::readCalibrationMatrices( camMatrixFileNames );
			
			MatrixXd K0 = Ks[ 0 ];
			MatrixXd K1 = Ks[ 1 ];
			
			cout << "K0: " << endl << K0 << endl << endl
				 << "K1: " << endl << K1 << endl << endl;
			
			vector< Correspondence > correspondences = ReconstructionController::lineCorrespToPointCorresp( m_lineCorrespondences, 0 , 1 );
			vector< Correspondence > normalized = ReconstructionController::normalizeWithCamCalibration( correspondences, K0, K1 );
			
			ReconstructionController controller( normalized, K0, K1 );
			shared_ptr< vector< VectorXd > > points3D = controller.reconstruct();
			
			shared_ptr< CameraMatrixDlt > solver = controller.getRansac()->getSolver();
			MatrixXd P0 = solver->getP0();
			MatrixXd P1 = solver->getSolution();
			
			cout << "================= Reconstruction results ===================" << endl << endl
				 << "Final camera matrices:" << endl << endl << "P0: " << P0 << endl << endl << "P1: " << P1 << endl << endl
				 << "Reconstructed points:" << endl << endl;
			for( VectorXd point3D : *points3D )
			{
				cout << point3D << endl << endl;
			}
			
			cout << "================= Reconstruction results end ===================" << endl << endl;
		}
	}
}