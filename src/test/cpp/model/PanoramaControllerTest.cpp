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

		TEST_F( PanoramaControllerTest, FromLeft )
		{
			vector< Mat > images;
			for( int i = 1; i < 5; ++i )
			{
				stringstream fileName;
				fileName << "../../../src/images/panorama_yosemite/yosemite" << i << ".jpg";
				images.push_back( imread( fileName.str() ) );
			}
			
			Mat panorama = PanoramaController::computePanorama( images );
			imwrite( "output/PanoramaYosemiteFromLeft.jpg", panorama );
		}
		
		TEST_F( PanoramaControllerTest, FromCenter )
		{
			vector< Mat > images;
			for( int i = 1; i < 5; ++i )
			{
				stringstream fileName;
				fileName << "../../../src/images/panorama_yosemite/yosemite" << i << ".jpg";
				images.push_back( imread( fileName.str() ) );
			}
			
			Mat panorama = PanoramaController::computePanoramaFromCenter( images );
			imwrite( "output/PanoramaYosemiteFromCenter.jpg", panorama );
		}
	}
}
