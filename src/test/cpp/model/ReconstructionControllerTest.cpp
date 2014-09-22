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
			void SetUp() {}
		};

		/** Tests the line reading for reconstruction. Checks the first and last correspondences of the first line and the
		 * first correspondence of the last line */
		TEST_F( ReconstructionControllerTest, LineReading )
		{
			vector< string > lineFileNames( 10 );
			for( int i = 0; i < 10; ++i )
			{
				stringstream ss;
				ss << "../../../src/images/house/correspondences/house.00" << i << ".lines";
				
				lineFileNames[ i ] =  ss.str();
			}
			
			string correspondenceFileName = "../../../src/images/house/correspondences/house.nview-lines";
			
			vector< map< int, Line > > lineCorrespondences = ReconstructionController::readLineCorrespondence(
				lineFileNames, correspondenceFileName );
			
			VectorXd expected( 3 ); expected << 330.21, 283.428, 1.;
			ASSERT_TRUE( lineCorrespondences[ 0 ][ 0 ].first.isApprox( expected, 1.0e-6 ) );
			expected << 594.949, 219.914, 1.;
			ASSERT_TRUE( lineCorrespondences[ 0 ][ 0 ].second.isApprox( expected, 1.0e-6 ) );
			
			expected << 312.82, 275.179, 1.;
			ASSERT_TRUE( lineCorrespondences[ 0 ][ 9 ].first.isApprox( expected, 1.0e-6 ) );
			expected << 349.38, 311.619, 1.;
			ASSERT_TRUE( lineCorrespondences[ 0 ][ 9 ].second.isApprox( expected, 1.0e-6 ) );
			
			expected << 582.008, 411.538, 1.;
			ASSERT_TRUE( lineCorrespondences[ 29 ][ 0 ].first.isApprox( expected, 1.0e-6 ) );
			expected << 656.567, 422.65, 1.;
			ASSERT_TRUE( lineCorrespondences[ 29 ][ 0 ].second.isApprox( expected, 1.0e-6 ) );
			
			// Rendering correspondences.
			int numImgs = 3;
			vector< Mat > imgs( numImgs );
			imgs[0] = imread( "../../../src/images/house/images/house.000.pgm" );
			imgs[1] = imread( "../../../src/images/house/images/house.001.pgm" );
			imgs[2] = imread( "../../../src/images/house/images/house.002.pgm" );
			
			for( map< int, Line > correspondences : lineCorrespondences )
			{
				for( int i = 0; i < numImgs; ++i )
				{
					Line line = correspondences[ i ];
					Point p0( line.first[0], line.first[1] );
					Point p1( line.second[0], line.second[1] );
					
					cv::line( imgs[i], p0, p1, Scalar( 0, 0, 255 ), 2, 8 );
				}
			}
			
			for( int i = 0; i < numImgs; ++i )
			{
				Mat img = imgs[ i ];
				stringstream ss;
				ss << "House " << i;
				imshow( ss.str(), img );
			}
			waitKey();
		}
	}
}