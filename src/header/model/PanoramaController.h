#ifndef PANORAMA_CONTROLLER_H

#include <QPixmap>
#include <vector>
#include <opencv2/core/core.hpp>
#include "PanoramaDlt.h"

namespace cv {
struct DMatch;}

using namespace std;
using namespace cv;
using namespace math;

namespace model
{
	class PanoramaController
	{
	public:
		/** Computes a panorama from given images. The image vector is assumed to be rotation-wise ordered. */
		static Mat computePanorama( vector< Mat >& images );
		
		/** Computes a panorama from given images, using the center image as reference. The image vector is assumed to be
		 * rotation-wise ordered. */
		static Mat computePanoramaFromCenter( vector< Mat >& images );
		
		/** Computes the correspondences of img0 and img1.
		 * @param outBetterMatches is an optional vector that will output the best computed matches. The vector returned by
		 * the methods has all computed matches (even outliers).
		 * @returns all computed matches (even outliers). */
		static vector< Correspondence > match( const Mat& img0, const Mat& img1,
											   vector< Correspondence >* outBetterMatches = nullptr );
		
		/** Transforms the image bounding box, returning the minimum transformed coordinates, maximum transformed coordinates
		 * and the transformed image final size. */
		static void transformBoundingBox( const Mat& transform, const Mat& image, Point2d& out_transfMinCoords,
										  Point2d& out_transfMaxCoords, Size& out_finalSize );
		
		/** Maps currentImg to the panorama space in two steps. First maps it to lastImg space, for which the homography to
		 * panorama space is previously know. Then uses this already-known panoramaHomography to do the final map to
		 * panorama space. The alpha paramters are used just for blending the final panorama image. */
		static void map( const Mat& lastImg, const Mat& currentImg, Mat& in_out_panorama, Mat& in_out_panoramaHomography,
						 Mat& in_out_additionalTranslation, const bool& isClockWise );
		
		/** Converts a QImage to OpenCV Mat. */
		static cv::Mat QImageToCvMat( const QImage &inImage, bool inCloneImageData = true );
		
		/** Converts a QPixmap to OpenCV Mat. */
		static cv::Mat QPixmapToCvMat( const QPixmap &inPixmap, bool inCloneImageData = true );
	private:
		PanoramaController();
	};
}

#endif