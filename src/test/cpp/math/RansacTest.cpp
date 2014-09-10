#include "PanoramaController.h"
#include "Ransac.h"
#include "Dlt.h"

#include <gtest/gtest.h>
#include <QApplication>
#include <highgui.h>

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
			Ransac< Correspondence > ransac( correspondences, 4, 0.5 );
			MatrixXd H = ransac.compute();
			
			// Qt uses the transpose of the usual transformation representation.
			QTransform qH(
				H(0, 0), H(1, 0), H(2, 0),
				H(0, 1), H(1, 1), H(2, 1),
				H(0, 2), H(1, 2), H(2, 2)
			);
			
			Mat currentMat = PanoramaController::QPixmapToCvMat( currentImg.transformed(qH) );
			Mat centerMat = PanoramaController::QPixmapToCvMat( centerImg );
			Mat panorama;
			
			addWeighted( centerMat, 0.5, currentMat, 0.5, 0.0, panorama);
			imshow( "Panorama", panorama );
			
			for( Correspondence correspondence : bestCorrespondences )
			{
				VectorXd p0 = correspondence.first;
				VectorXd p1 = correspondence.second;
				VectorXd transfP0 = H * p0;
				transfP0 = transfP0 / transfP0[ 2 ];
				
				cout << "p0: " << endl << p0 << endl << endl
					 << "p1: " << endl << p1 << endl << endl
					 << "H * p0: " << endl << transfP0 << endl << endl;
				
				ASSERT_TRUE( p1.isApprox( transfP0 ) );
			}
		}
	}
}