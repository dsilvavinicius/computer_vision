#include "PanoramaController.h"

#include <gtest/gtest.h>
#include <highgui.h>

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
			vector< Mat > images;
			for( int i = 1; i < 5; ++i )
			{
				stringstream fileName;
				fileName << "../../../src/images/panorama_yosemite/yosemite" << i << ".jpg";
				images.push_back( imread( fileName.str() ) );
			}
			
			PanoramaController::computePanorama( images );
		}
	}
}
