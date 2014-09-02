#include "PanoramaController.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

QPixmap computePanorama(QPixmap* centerImg, vector< QPixmap* >& images, vector< int >& orderIndices)
{
	
}

void map(QPixmap* qtPanorama, QPixmap* qtCenterImg, QPixmap* qtCurrentImg)
{
	if (!qtPanorama || !qtCenterImg || !qtCurrentImg)
	{
		throw logic_error("Cannot map from or to a null image.");
	}
	QSize imgSize = qtPanorama->size();
	
	Mat centerImg( imgSize.width(), imgSize.height(), CV_32FC2 );
	Mat currentImg( imgSize.width(), imgSize.height(), CV_32FC2 );

	if( !centerImg.data || !currentImg.data )
	{
		throw logic_error("Problem generating OpenCV images.");
	}

	SurfFeatureDetector detector;
	vector<KeyPoint> centerKeyPoints, currentKeyPoints;
	detector.detect( centerImg, centerImg );
	detector.detect( currentImg, currentKeyPoints );

	SurfDescriptorExtractor extractor;
	Mat centerDescriptors, currentDescriptors;
	extractor.compute( centerImg, centerKeyPoints, centerDescriptors );
	extractor.compute( currentImg, currentKeyPoints, currentDescriptors );

	//-- Step 3: Matching descriptor vectors with a brute force matcher
	BFMatcher matcher(NORM_L2);
	std::vector< DMatch > matches;
	matcher.match( centerDescriptors, currentDescriptors, matches );
	
	// drawing the results
	namedWindow("matches", 1);
	Mat imgMatches;
	drawMatches(centerImg, centerKeyPoints, currentImg, currentKeyPoints, matches, imgMatches);
	imshow("matches", imgMatches);
	waitKey(0);
}