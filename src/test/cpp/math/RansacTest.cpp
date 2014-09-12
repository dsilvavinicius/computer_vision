#include "PanoramaController.h"
#include "Ransac.h"
#include "Dlt.h"

#include <gtest/gtest.h>
#include <QApplication>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace math;

extern "C" int g_argc;
extern "C" char** g_argv;

using namespace cv;
using namespace model;

namespace math
{
	namespace test
	{
        class RansacTest : public ::testing::Test
		{
		protected:
			void SetUp() {}
		};

		TEST_F(RansacTest, Test)
		{
			QApplication app(g_argc, g_argv);
			QPixmap centerImg("../../../src/images/panorama/panorama1.JPG");
			QPixmap currentImg("../../../src/images/panorama/panorama2.JPG");
			Q_ASSERT(!centerImg.isNull() && !currentImg.isNull());
			
			vector< Correspondence > bestCorrespondences;
			vector< Correspondence > correspondences = PanoramaController::match(currentImg, centerImg, &bestCorrespondences);
			Ransac< Correspondence > ransac( bestCorrespondences, 4, 0.99 );
			MatrixXd H = ransac.compute();
			
			for( Correspondence correspondence : bestCorrespondences )
			{
				VectorXd p0 = correspondence.first;
				VectorXd p1 = correspondence.second;
				VectorXd transfP0 = H * p0;
				transfP0 = transfP0 / transfP0[ 2 ];
				
				cout << "p0: " << endl << p0 << endl << endl
					 << "p1: " << endl << p1 << endl << endl
					 << "H * p0: " << endl << transfP0 << endl << endl;
				
				//ASSERT_TRUE( p1.isApprox( transfP0 ) );
			}
			
			// Qt uses the transpose of the usual transformation representation.
			/*QTransform qH(
				H(0, 0), H(1, 0), H(2, 0),
				H(0, 1), H(1, 1), H(2, 1),
				H(0, 2), H(1, 2), H(2, 2)
			);*/
			
			Mat currentMat = PanoramaController::QPixmapToCvMat( currentImg );
			Mat centerMat = PanoramaController::QPixmapToCvMat( centerImg );
			Mat cvH;
			cvH.create( 3, 3, CV_32FC1 );
			cvH.at< float >( 0, 0 ) = H( 0, 0 ); cvH.at< float >( 0, 1 ) = H( 0, 1 ); cvH.at< float >( 0, 2 ) = H( 0, 2 );
			cvH.at< float >( 1, 0 ) = H( 1, 0 ); cvH.at< float >( 1, 1 ) = H( 1, 1 ); cvH.at< float >( 1, 2 ) = H( 1, 2 );
			cvH.at< float >( 2, 0 ) = H( 2, 0 ); cvH.at< float >( 2, 1 ) = H( 2, 1 ); cvH.at< float >( 2, 2 ) = H( 2, 2 );
			
			// Computing offset to avoid cropping in the transformed image.
			vector< Point2d > origPoints;
			origPoints.push_back( Point2d( 0., 0. ) );
			origPoints.push_back( Point2d( 0., currentMat.size[ 1 ] - 1 ) );
			origPoints.push_back( Point2d( currentMat.size[ 0 ] - 1, 0. ) );
			origPoints.push_back( Point2d( currentMat.size[ 0 ] - 1, currentMat.size[ 1 ] - 1 ) );
			
			vector< Point2d > transformedPoints;
			perspectiveTransform(origPoints, transformedPoints, cvH);
			Point2d minCoords( 9999, 9999 );
			Point2d maxCoords( -1, -1 );
			for( Point2d point : transformedPoints )
			{
				minCoords.x = min( minCoords.x, point.x );
				minCoords.y = min( minCoords.y, point.y );
				maxCoords.x = max( maxCoords.x, point.x );
				maxCoords.y = max( maxCoords.y, point.y );
			}
			Mat translation;
			translation.create( 3, 3, CV_32FC1 );
			translation.at< float >( 0, 0 ) = 1.; translation.at< float >( 0, 1 ) = 0.; translation.at< float >( 0, 2 ) = -minCoords.x;
			translation.at< float >( 1, 0 ) = 0.; translation.at< float >( 1, 1 ) = 1.; translation.at< float >( 1, 2 ) = -minCoords.y;
			translation.at< float >( 2, 0 ) = 0.; translation.at< float >( 2, 1 ) = 0.; translation.at< float >( 2, 2 ) = 1.;
			
			Mat trueTransform = translation * cvH;
			Mat currentTransformed;
			warpPerspective( currentMat, currentTransformed, trueTransform, Size(1000, 1000), CV_INTER_AREA );
			
			Mat centerTransformed;
			warpPerspective( centerMat, centerTransformed, translation, Size(1000, 1000), CV_INTER_AREA );
			
			imshow( "Current", currentMat );
			imshow( "Center", centerMat );
			imshow( "Current Transformed", currentTransformed );
			imshow( "Center Transformed", centerTransformed );
			
			waitKey();
			
			Mat panorama;
			addWeighted( centerTransformed, 1., currentTransformed, 1., 0.0, panorama);
			imshow( "Panorama", panorama );
			waitKey();
		}
	}
}