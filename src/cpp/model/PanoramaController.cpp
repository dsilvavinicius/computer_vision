#include "PanoramaController.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QRgb>
#include <QDebug>

#include "Ransac.h"

using namespace cv;
using namespace std;
using namespace math;

namespace model
{
	Mat PanoramaController::computePanorama( vector< Mat >& images )
	{
		Mat currentImg = images[ 0 ];
		cout << "Starting from photo 0" << endl << endl;
		Mat lastImg = currentImg;
		Mat panoramaImg = currentImg;
		Mat panoramaHomography = Mat::eye( 3, 3, CV_64F ); // Identity, at first.
		Mat additionalTranslation = Mat::eye( 3, 3, CV_64F );
		
		for( int i = 1; i < images.size(); ++i )
		{
			cout << "Adding photo " << i << endl << endl;
			
			currentImg = images[ i ];
			PanoramaController::map( lastImg, currentImg, panoramaImg, panoramaHomography, additionalTranslation, true );
			lastImg = currentImg;
		}
		
		return panoramaImg;
	}
	
	Mat PanoramaController::computePanoramaFromCenter( vector< Mat >& images )
	{
		int centerIdx = images.size() / 2;
		cout << "Center photo " << centerIdx << endl << endl;
		Mat currentImg = images[ centerIdx ];
		Mat lastImg = currentImg;
		Mat panoramaImg = currentImg;
		Mat panoramaHomography = Mat::eye( 3, 3, CV_64F ); // Identity, at first.
		Mat additionalTranslation = Mat::eye( 3, 3, CV_64F );
		
		for( int i = centerIdx + 1; i < images.size(); ++i )
		{
			cout << "Adding photo " << i << endl << endl;
			
			currentImg = images[ i ];
			PanoramaController::map( lastImg, currentImg, panoramaImg, panoramaHomography, additionalTranslation, true );
			lastImg = currentImg;
		}
		
		Mat translationFromClockWise = additionalTranslation.clone();

		panoramaHomography = translationFromClockWise.clone();
		lastImg = images[ centerIdx ];
		for( int i = centerIdx - 1; i > -1; --i )
		{
			cout << "Adding photo " << i << endl << endl;
			
			currentImg = images[ i ];
			
			PanoramaController::map( lastImg, currentImg, panoramaImg, panoramaHomography, additionalTranslation, false );
			lastImg = currentImg;
		}
		
		return panoramaImg;
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

	vector< Correspondence > PanoramaController::match( const Mat& img0, const Mat& img1, vector< Correspondence >* outBetterMatches )
	{
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
				if( matches[i].distance <= 50/*max(2 * min_dist, 0.02)*/ )
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
		/*namedWindow( "matches", 1 );
		Mat imgMatches;
		drawMatches( img0, img0KeyPoints, img1, img1KeyPoints, matchesToRender, imgMatches );
		imshow( "matches", imgMatches );
		waitKey( 0 );
		destroyAllWindows();*/
		
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
	
	void PanoramaController::transformBoundingBox( const Mat& transform, const Mat& image, Point2d& transfMinCoords,
												   Point2d& transfMaxCoords, Size& finalSize )
	{
		// Computing offset to avoid cropping in the transformed image.
		vector< Point2d > origPoints;
		origPoints.push_back( Point2d( 0., 0. ) );
		origPoints.push_back( Point2d( image.size().width - 1, 0. ) );
		origPoints.push_back( Point2d( image.size().width - 1, image.size().height - 1 ) );
		origPoints.push_back( Point2d( 0., image.size().width - 1 ) );
		
		cout << "CurrentImg box: " << endl;
		for( Point2d point : origPoints ) { cout << point << endl; }
		cout << endl;
		
		vector< Point2d > transformedPoints;
		perspectiveTransform(origPoints, transformedPoints, transform);
		
		transfMinCoords.x = 9999; transfMinCoords.y = 9999;
		transfMaxCoords.x = -1; transfMaxCoords.y = -1;
		
		for( int i = 0; i < transformedPoints.size(); ++i )
		{
			Point2d original = origPoints[ i ];
			Point2d transformed = transformedPoints[ i ];
			
			transfMinCoords.x = min( transfMinCoords.x, transformed.x );
			transfMinCoords.y = min( transfMinCoords.y, transformed.y );
			
			transfMaxCoords.x = max( transfMaxCoords.x, transformed.x );
			transfMaxCoords.y = max( transfMaxCoords.y, transformed.y );
		}
		
		finalSize.width = transfMaxCoords.x;
		finalSize.height = transfMaxCoords.y;
	}
	
	void PanoramaController::map( const Mat& lastImg, const Mat& currentImg, Mat& panorama, Mat& panoramaHomography,
								  Mat& additionalTranslation, const bool& isClockWise )
	{
		vector< Correspondence > bestCorrespondences;
		vector< Correspondence > correspondences = PanoramaController::match(currentImg, lastImg, &bestCorrespondences);
		Ransac< Correspondence, Dlt > ransac( bestCorrespondences, 4, 0.99 );
		MatrixXd H = ransac.compute();
		
		Mat cvH;
		cvH.create( 3, 3, CV_64FC1 );
		cvH.at< double >( 0, 0 ) = H( 0, 0 ); cvH.at< double >( 0, 1 ) = H( 0, 1 ); cvH.at< double >( 0, 2 ) = H( 0, 2 );
		cvH.at< double >( 1, 0 ) = H( 1, 0 ); cvH.at< double >( 1, 1 ) = H( 1, 1 ); cvH.at< double >( 1, 2 ) = H( 1, 2 );
		cvH.at< double >( 2, 0 ) = H( 2, 0 ); cvH.at< double >( 2, 1 ) = H( 2, 1 ); cvH.at< double >( 2, 2 ) = H( 2, 2 );
		
		cout << "Ransac Homography: " << endl << cvH << endl << endl; 
		
		panoramaHomography = panoramaHomography * cvH;
		
		Size currentTransformedSize;
		Point2d currentToPanoramaROIOrigin;
		Point2d currentToPanoramaROIMaxCoords;
		PanoramaController::transformBoundingBox(panoramaHomography, currentImg, currentToPanoramaROIOrigin,
												 currentToPanoramaROIMaxCoords, currentTransformedSize);
		
		// Taking into account the translation to avoid cropping pixels transformed to negative positions.
		Mat translation;
		translation.create( 3, 3, CV_64FC1 );
		translation.at< double >( 0, 0 ) = 1.; translation.at< double >( 0, 1 ) = 0.; translation.at< double >( 0, 2 ) = -min( currentToPanoramaROIOrigin.x, 0. );
		translation.at< double >( 1, 0 ) = 0.; translation.at< double >( 1, 1 ) = 1.; translation.at< double >( 1, 2 ) = -min( currentToPanoramaROIOrigin.y, 0. );
		translation.at< double >( 2, 0 ) = 0.; translation.at< double >( 2, 1 ) = 0.; translation.at< double >( 2, 2 ) = 1.;
		
		panoramaHomography = translation * panoramaHomography;
		additionalTranslation = translation * additionalTranslation;
		
		PanoramaController::transformBoundingBox(panoramaHomography, currentImg, currentToPanoramaROIOrigin,
												 currentToPanoramaROIMaxCoords, currentTransformedSize);
		
		Size panoramaTranslatedSize;
		Point2d panoramaTranslatedROIOrigin;
		Point2d panoramaTranslatedROIMaxCoords;
		PanoramaController::transformBoundingBox(translation, panorama, panoramaTranslatedROIOrigin,
												 panoramaTranslatedROIMaxCoords, panoramaTranslatedSize);
		
		Size finalSize = isClockWise ? currentTransformedSize : panoramaTranslatedSize;
		
		Size currentToPanoramaROISize( min( currentToPanoramaROIMaxCoords.x - currentToPanoramaROIOrigin.x,
											finalSize.width - currentToPanoramaROIOrigin.x ),
									   min( currentToPanoramaROIMaxCoords.y - currentToPanoramaROIOrigin.y, 
											finalSize.height - currentToPanoramaROIOrigin.y ) );
		
		cout << "Current to panorama size: " << currentTransformedSize << endl << endl
			 << "Panorama size: " << panoramaTranslatedSize << endl << endl
			 << "Final size:" << finalSize << endl << endl;
		
		Mat currentToPanorama;
		warpPerspective( currentImg, currentToPanorama, panoramaHomography, finalSize );
		
		Mat panoramaTransformed;
		warpPerspective( panorama, panoramaTransformed, translation, finalSize );
		
		Rect roiPanorama( currentToPanoramaROIOrigin, currentToPanoramaROISize );
		
		cout  << "Current to Panorama ROI: " << roiPanorama << endl << endl
			 << "Current to panorama size: " << currentToPanorama.size() << endl << endl
			 << "Panorama size: " << panoramaTransformed.size() << endl << endl;
		
		Mat mask;
		inRange( currentToPanorama( roiPanorama ), cv::Scalar(0., 0., 0.), cv::Scalar(0., 0., 0.), mask );
		bitwise_not( mask, mask );
		
		currentToPanorama( roiPanorama ).copyTo( panoramaTransformed( roiPanorama ), mask );
		
		imshow( "Panorama Transformed", panoramaTransformed );
		waitKey();
		
		panorama = panoramaTransformed;
	}
}