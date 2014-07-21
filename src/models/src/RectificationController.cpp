#include <Eigen/Dense>
#include "ProjectionRectificator.h"
#include "AffineRectificator.h"
#include "RectificationController.h"

using namespace Eigen;
using namespace math;

namespace models
{
	RectificationController::RectificationController() {}

	QPixmap RectificationController::rectify(ClickableLabel* projectedImageLabel, const QSize& POISize,
											 bool pointOfInterestFlag)
	{	
		// First, create pairs of point correlations between projection and world space.
		CircularList<SelectedPixel*>* selectedPixelsProj = projectedImageLabel->getSelectedPixels();
		int numSelectedPixels = selectedPixelsProj->size();
		
		vector<QPoint> selectedPixelsWorld(numSelectedPixels);
		selectedPixelsWorld[0] = QPoint(0, 0);
		selectedPixelsWorld[1] = selectedPixelsWorld[0] + QPoint(0, POISize.height());
		selectedPixelsWorld[2] = selectedPixelsWorld[0] + QPoint(POISize.width(), POISize.height());
		selectedPixelsWorld[3] = selectedPixelsWorld[0] + QPoint(POISize.width(), 0);
		
		vector<pair<VectorXd, VectorXd>> correlationPoints(numSelectedPixels);
		
		for (int i = 0; i < numSelectedPixels; ++i)
		{
			QPoint qProjectedPoint = (*selectedPixelsProj)[i]->getPos();
			QPoint qWorldPoint = selectedPixelsWorld[i];
			
			VectorXd projectedPoint(3);
			projectedPoint << qProjectedPoint.x(), qProjectedPoint.y(), 1.;
			VectorXd worldPoint(3);
			worldPoint << qWorldPoint.x(), qWorldPoint.y(), 1.;
			
			pair<VectorXd, VectorXd> correlationPair(projectedPoint, worldPoint);
			correlationPoints[i] = correlationPair;
		}
		
		// Second, define the projection to world transformation.
		ProjectionRectificator rectificator = ProjectionRectificator(correlationPoints);
		MatrixXd projToWorld = *rectificator.getTransformation();
		
		// Qt uses the transpose of the usual transformation representation.
		QTransform qProjToWorld(
			projToWorld(0, 0), projToWorld(1, 0), projToWorld(2, 0),
			projToWorld(0, 1), projToWorld(1, 1), projToWorld(2, 1),
			projToWorld(0, 2), projToWorld(1, 2), projToWorld(2, 2)
		);
		
		QPixmap rectifiedPixmap = projectedImageLabel->pixmap()->transformed(qProjToWorld, Qt::SmoothTransformation);
		
		if (pointOfInterestFlag)
		{
			return selectPointOfInterest(rectifiedPixmap, qProjToWorld, projectedImageLabel->pixmap()->size(), (*selectedPixelsProj)[0]->getPos(), POISize);
		}
		else
		{
			return rectifiedPixmap;
		}
	}
	
	QPixmap RectificationController::selectPointOfInterest(const QPixmap& rectifiedImage, const QTransform& rectification, const QSize& originalImageSize,
														   const QPoint& POIOrigin, const QSize& POISize)
	{
		// Get the true transformation used in Qt, since Qt automaticaly scales the image to bound all transformed pixels.
		QTransform trueRectification = QPixmap::trueMatrix(rectification, originalImageSize.width(), originalImageSize.height());
		QPoint rectifiedOrigin = trueRectification.map(POIOrigin);
		return rectifiedImage.copy(rectifiedOrigin.x(), rectifiedOrigin.y(), POISize.width(), POISize.height());
	}
	
	QPixmap RectificationController::toAffine(ClickableLabel* projectedImageLabel)
	{
		CircularList<SelectedPixel*>* points = projectedImageLabel->getSelectedPixels();
		if (points->size() != 8)
		{
			throw logic_error("8 points that define two pairs of parallel lines in affine space should be"
				"selected in the label"
			);
		}
		
		vector<pair<VectorXd, VectorXd>> parallelPairs;
		VectorXd linePair[2];
			
		for (int j = 0; j < 2; ++j) // Parallel line pair creation.
		{
			VectorXd line;
			for (int i = 0; i < 2; ++i) // Line creation.
			{
				QPoint p0 = (*points)[j*4 + i*2]->getPos();
				QPoint p1 = (*points)[j*4 + i*2 + 1]->getPos();
				double m = (double)(p1.y() - p0.y()) / (double)(p1.x() - p0.x()); // Line slope.
				double b = m * p0.x() + p0.y(); // Line y-intercept.
				VectorXd line(3);
				line << m, b, 1.;
				linePair[i] = line;
			}
			parallelPairs.push_back( pair<VectorXd, VectorXd>(linePair[0], linePair[1]) );
		}
		
		AffineRectificator rectificator = AffineRectificator(parallelPairs);
		MatrixXd projToAffine = *rectificator.getTransformation();
		
		// Qt uses the transpose of the usual transformation representation.
		QTransform qProjToAffine(
			projToAffine(0, 0), projToAffine(1, 0), projToAffine(2, 0),
			projToAffine(0, 1), projToAffine(1, 1), projToAffine(2, 1),
			projToAffine(0, 2), projToAffine(1, 2), projToAffine(2, 2)
		);
		
		return projectedImageLabel->pixmap()->transformed(qProjToAffine, Qt::SmoothTransformation);
		
	}
}