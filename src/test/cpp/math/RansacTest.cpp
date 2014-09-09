#include "PanoramaController.h"
#include "Ransac.h"
#include "Dlt.h"

#include <gtest/gtest.h>
#include <QApplication>

using namespace math;

extern "C" int g_argc;
extern "C" char** g_argv;

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
			/*QApplication app(g_argc, g_argv);
			QPixmap centerImg("../../../src/images/panorama/panorama1.JPG");
			QPixmap currentImg("../../../src/images/panorama/panorama2.JPG");
			Q_ASSERT(!centerImg.isNull() && !currentImg.isNull());
			
			vector< Correspondence > correspondences = PanoramaController::match(centerImg, currentImg);
			Ransac< Correspondence > ransac( correspondences, 4, 1. );
			ransac.compute();*/
		}
	}
}