#include "PanoramaController.h"
#include "Ransac.h"
#include "Dlt.h"

#include <gtest/gtest.h>

using namespace math;

namespace model
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
			QPixmap centerImg("../../../src/images/panorama/panorama1.JPG");
			QPixmap currentImg("../../../src/images/panorama/panorama2.JPG");
			Q_ASSERT(!centerImg.isNull() && !currentImg.isNull());
			
			vector< Correspondence > correspondences = PanoramaController::match(centerImg, currentImg);
			Ransac< Correspondence > ransac( correspondences, Dlt::compute , 4, 0.5 );
			ransac.compute();
		}
	}
}