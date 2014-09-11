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
			vector< Correspondence > correspondences = PanoramaController::match(centerImg, currentImg, &bestCorrespondences);
			Ransac< Correspondence > ransac( bestCorrespondences, 4, 0.5 );
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
			Mat transformed;
			Mat cvH;
			cvH.create( 3, 3, CV_32FC1 );
			cvH.at< float >( 0, 0 ) = H( 0, 0 ); cvH.at< float >( 0, 1 ) = H( 0, 1 ); cvH.at< float >( 0, 2 ) = H( 0, 2 );
			cvH.at< float >( 1, 0 ) = H( 1, 0 ); cvH.at< float >( 1, 1 ) = H( 1, 1 ); cvH.at< float >( 1, 2 ) = H( 1, 2 );
			cvH.at< float >( 2, 0 ) = H( 2, 0 ); cvH.at< float >( 2, 1 ) = H( 2, 1 ); cvH.at< float >( 2, 2 ) = H( 2, 2 );
			
			warpPerspective( centerMat, transformed, cvH, Size(1000, 1000) , CV_INTER_AREA);
			
			imshow( "Center", centerMat );
			
			imshow( "Current", currentMat );
			
			imshow( "Transfored", transformed );
			waitKey();
			
			//Mat panorama;
			
			//addWeighted( centerMat, 0.5, transformed, 0.5, 0.0, panorama);
			//imshow( "Panorama", panorama );
			//waitKey();
		}
	}
}