#include "PanoramaController.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QRgb>
#include <QDebug>

using namespace cv;
using namespace std;
using namespace math;

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

	vector< Correspondence > PanoramaController::match(QPixmap& qtImg0, QPixmap& qtImg1,
													   vector< Correspondence >* outBetterMatches)
	{
		if( !qtImg0 || !qtImg1 )
		{
			throw logic_error("Cannot map from or to a null image.");
		}
		QSize imgSize = qtImg0.size();
		
		Mat img0 = QPixmapToCvMat( qtImg0 );
		Mat img1 = QPixmapToCvMat( qtImg1 );
		
		CV_Assert(!img0.empty() && !img1.empty());
		
		ORB detector;
		vector< KeyPoint > img0KeyPoints, img1KeyPoints;
		detector.detect( img0, img0KeyPoints );
		detector.detect( img1, img1KeyPoints );
		
		OrbDescriptorExtractor extractor;
		Mat img0Descriptors, img1Descriptors;
		extractor.compute( img0, img0KeyPoints, img0Descriptors );
		extractor.compute( img1, img1KeyPoints, img1Descriptors );

		BFMatcher matcher( NORM_HAMMING );
		std::vector< DMatch > matches;
		matcher.match( img0Descriptors, img1Descriptors, matches );
		
		vector< DMatch > matchesToRender;
		if( outBetterMatches != nullptr )
		{
			double max_dist = 0; double min_dist = 100;
			
			//-- Quick calculation of max and min distances between keypoints
			for( int i = 0; i < img0Descriptors.rows; i++ )
			{ 
				double dist = matches[i].distance;
				if( dist < min_dist ) min_dist = dist;
				if( dist > max_dist ) max_dist = dist;
			}

			//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
			//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
			//-- small)
			//-- PS.- radiusMatch can also be used here.
			for( int i = 0; i < img0Descriptors.rows; i++ )
			{
				if( matches[i].distance <= max(2 * min_dist, 0.02) )
				{
					DMatch match = matches[i];
					matchesToRender.push_back( match );
					
					Point2f cvP0 = img0KeyPoints[match.queryIdx].pt;
					Point2f cvP1 = img1KeyPoints[match.trainIdx].pt;
					VectorXd p0(3); p0[0] = cvP0.x; p0[1] = cvP0.y; p0[2] = 1;
					VectorXd p1(3); p1[0] = cvP1.x; p1[1] = cvP1.y; p1[2] = 1;
			
					outBetterMatches->push_back( Correspondence( p0, p1 ) );
				}
			}
		}
		else
		{
			matchesToRender = matches;
		}
		
		// Drawing the results
		namedWindow( "matches", 1 );
		Mat imgMatches;
		drawMatches( img0, img0KeyPoints, img1, img1KeyPoints, matchesToRender, imgMatches );
		imshow( "matches", imgMatches );
		waitKey( 0 );
		
		vector< Correspondence > correspondences;
		for( vector< DMatch >::iterator iter = matches.begin(); iter != matches.end(); ++iter )
		{
			Point2f cvP0 = img0KeyPoints[iter->queryIdx].pt;
			Point2f cvP1 = img1KeyPoints[iter->trainIdx].pt;
			VectorXd p0(3); p0[0] = cvP0.x; p0[1] = cvP0.y; p0[2] = 1;
			VectorXd p1(3); p1[0] = cvP1.x; p1[1] = cvP1.y; p1[2] = 1;
			
			correspondences.push_back( Correspondence( p0, p1 ) );
		}
		
		return correspondences;
	}
   
	void PanoramaController::map( QPixmap& qtPanorama, QPixmap& qtCurrentImg )
	{
		match(qtPanorama, qtCurrentImg);
	}
}