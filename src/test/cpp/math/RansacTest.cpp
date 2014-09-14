#include "PanoramaController.h"
#include "Ransac.h"
#include "Dlt.h"

#include <gtest/gtest.h>
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
		
		TEST_F(RansacTest, DISABLED_Test)
		{
			Mat panoramaImg = imread( "../../../src/images/panorama_yosemite/yosemite1.jpg" );
			Mat lastImg = imread( "../../../src/images/panorama_yosemite/yosemite1.jpg" );
			Mat currentImg = imread( "../../../src/images/panorama_yosemite/yosemite2.jpg" );
			
			Mat panoramaHomography = Mat::eye( 3, 3, CV_64F ); // Identity, at first.
			Mat additionalTranslation = Mat::eye( 3, 3, CV_64F );
			PanoramaController::map( lastImg, currentImg, panoramaImg, panoramaHomography, additionalTranslation, true, 0.5,
									 0.5);
		}
	}
}