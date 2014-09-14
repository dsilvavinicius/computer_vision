#include "PanoramaController.h"
#include "Ransac.h"
#include "Dlt.h"

#include <gtest/gtest.h>
#include <QApplication>
#include <qgraphicsitem.h>
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
			Mat panoramaImg = imread("../../../src/images/panorama_yosemite/yosemite1.jpg");
			Mat lastImg = imread("../../../src/images/panorama_yosemite/yosemite1.jpg");
			Mat currentImg = imread("../../../src/images/panorama_yosemite/yosemite2.jpg");
			
			Mat panoramaHomography = Mat::eye( 3, 3, CV_64F ); // Identity, at first.
			PanoramaController::map( lastImg, currentImg, panoramaImg, panoramaHomography);
		}
	}
}