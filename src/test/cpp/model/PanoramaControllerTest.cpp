#include "PanoramaController.h"

#include <gtest/gtest.h>
#include <QString>
#include <QApplication>

extern "C" int g_argc;
extern "C" char** g_argv;

namespace model
{
	namespace test
	{
        class PanoramaControllerTest : public ::testing::Test
		{
		protected:
			void SetUp() {}
		};

		TEST_F(PanoramaControllerTest, map)
		{
			/*QApplication app(g_argc, g_argv);
			
			QPixmap centerImg("../../../src/images/panorama/panorama1.JPG");
			QPixmap currentImg("../../../src/images/panorama/panorama2.JPG");
			Q_ASSERT(!centerImg.isNull() && !currentImg.isNull());
			
			PanoramaController::map(centerImg, currentImg);*/
		}
	}
}
