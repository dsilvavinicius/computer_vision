#include "PanoramaController.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QRgb>
#include <QDebug>

using namespace cv;
using namespace std;

namespace model
{
	QPixmap PanoramaController::computePanorama( QPixmap& centerImg, vector< QPixmap& >& images,
												vector< int >& orderIndices )
	{
		
	}

	// If inImage exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inImage's
   // data with the cv::Mat directly
   // NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
   cv::Mat PanoramaController::QImageToCvMat( const QImage &inImage, bool inCloneImageData )
   {
      switch ( inImage.format() )
      {
         // 8-bit, 4 channel
         case QImage::Format_RGB32:
         {
            cv::Mat mat( inImage.height(), inImage.width(), CV_8UC4, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );
			cv::cvtColor(mat, mat, CV_BGRA2BGR);
            return (inCloneImageData ? mat.clone() : mat);
         }
 
         // 8-bit, 3 channel
         case QImage::Format_RGB888:
         {
            if ( !inCloneImageData )
               qWarning() << "ASM::QImageToCvMat() - Conversion requires cloning since we use a temporary QImage";
 
            QImage swapped = inImage.rgbSwapped();
 
            return cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine() ).clone();
         }
 
         // 8-bit, 1 channel
         case QImage::Format_Indexed8:
         {
            cv::Mat  mat( inImage.height(), inImage.width(), CV_8UC1, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );
 
            return (inCloneImageData ? mat.clone() : mat);
         }
 
         default:
            qWarning() << "ASM::QImageToCvMat() - QImage format not handled in switch:" << inImage.format();
            break;
      }
 
      return cv::Mat();
   }
 
   // If inPixmap exists for the lifetime of the resulting cv::Mat, pass false to inCloneImageData to share inPixmap's data
   // with the cv::Mat directly
   //    NOTE: Format_RGB888 is an exception since we need to use a local QImage and thus must clone the data regardless
   cv::Mat PanoramaController::QPixmapToCvMat( const QPixmap &inPixmap, bool inCloneImageData )
   {
      return QImageToCvMat( inPixmap.toImage(), inCloneImageData );
   }
	
	void PanoramaController::map( QPixmap& qtPanorama, QPixmap& qtCurrentImg )
	{
		if( !qtPanorama || !qtCurrentImg )
		{
			throw logic_error("Cannot map from or to a null image.");
		}
		QSize imgSize = qtPanorama.size();
		
		Mat panoramaImg = QPixmapToCvMat( qtPanorama );
		Mat currentImg = QPixmapToCvMat( qtCurrentImg );
		
		CV_Assert(!panoramaImg.empty() && !currentImg.empty());
		
		ORB detector;
		vector< KeyPoint > panoramaKeyPoints, currentKeyPoints;
		detector.detect( panoramaImg, panoramaKeyPoints );
		detector.detect( currentImg, currentKeyPoints );

		//
		/*Mat imgPanoramaPoints; Mat imgCurrentPoints;
		drawKeypoints( panoramaImg, panoramaKeyPoints, imgPanoramaPoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		drawKeypoints( currentImg, currentKeyPoints, imgCurrentPoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

		//-- Show detected (drawn) keypoints
		imshow("Keypoints 1", imgPanoramaPoints );
		imshow("Keypoints 2", imgCurrentPoints );

		waitKey(0);*/
		//
		
		OrbDescriptorExtractor extractor;
		Mat panoramaDescriptors, currentDescriptors;
		extractor.compute( panoramaImg, panoramaKeyPoints, panoramaDescriptors );
		extractor.compute( currentImg, currentKeyPoints, currentDescriptors );

		BFMatcher matcher( NORM_HAMMING );
		std::vector< DMatch > matches;
		matcher.match( panoramaDescriptors, currentDescriptors, matches );
		
		// drawing the results
		namedWindow( "matches", 1 );
		Mat imgMatches;
		drawMatches( panoramaImg, panoramaKeyPoints, currentImg, currentKeyPoints, matches, imgMatches );
		imshow( "matches", imgMatches );
		waitKey( 0 );
	}
}