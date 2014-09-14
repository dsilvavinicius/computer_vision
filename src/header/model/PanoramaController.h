#ifndef PANORAMA_CONTROLLER_H

#include <QPixmap>
#include <vector>
#include <opencv2/core/core.hpp>
#include "Dlt.h"

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
		/** Computes a panorama from given images. All images will be mapped to centerImg space, in the order of the indices in orderIndices.
		 * @param centerImg is the center image, which will define the space of the panorama final image.
		 * @param images are the other images, which will be mapped to centerImg space.
		 * @param orderIndices are the indices of the images in images. The order of the elements in orderIndices indicates the order in which
		 * the map will be done.
		 */
		static QPixmap computePanorama( QPixmap& centerImg, vector< QPixmap& >& images, vector< int >& orderIndices );
		
		/** Computes the correspondences of img0 and img1.
		 * @param outBetterMatches is an optional vector that will output the best computed matches. The vector returned by
		 * the methods has all computed matches (even outliers).
		 * @returns all computed matches (even outliers). */
		static vector< Correspondence > match( Mat& img0, Mat& img1, vector< Correspondence >* outBetterMatches = nullptr );
		
		/** Transforms the image bounding box, returning the minimum transformed coordinates, maximum transformed coordinates
		 * and the transformed image final size. */
		static void transformBoundingBox( Mat& transform, Mat& image, Point2d& out_transfMinCoords,
										  Point2d& out_transfMaxCoords, Size& out_finalSize );
		
		/** Maps currentImg to the panorama space in two steps. First maps it to lastImg space, for which the homography to
		 * panorama space is previously know. Then uses this already-known panoramaHomography to do the final map to
		 * panorama space. */
		static void map( Mat& lastImg, Mat& currentImg, Mat& in_out_panorama, Mat& in_out_panoramaHomography );
		
		/** Converts a QImage to OpenCV Mat. */
		static cv::Mat QImageToCvMat( const QImage &inImage, bool inCloneImageData = true );
		
		/** Converts a QPixmap to OpenCV Mat. */
		static cv::Mat QPixmapToCvMat( const QPixmap &inPixmap, bool inCloneImageData = true );
	private:
		PanoramaController();
	};
}

#endif