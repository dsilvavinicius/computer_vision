#include <Eigen/Dense>
#include "ProjectionRectificator.h"
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
}