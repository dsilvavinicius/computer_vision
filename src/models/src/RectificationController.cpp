#include <Eigen/Dense>
#include "ProjectionRectificator.h"
#include "RectificationController.h"

using namespace Eigen;
using namespace math;

namespace models
{
	RectificationController::RectificationController() {}

	QPixmap RectificationController::rectify(ClickableLabel* projectedImageLabel, ClickableLabel* worldImageLabel, bool pointOfInterestFlag)
	{	
		// First, create pairs of point correlations between projection and world space.
		CircularList<SelectedPixel*>* selectedPixelsProj = projectedImageLabel->getSelectedPixels();
		CircularList<SelectedPixel*>* selectedPixelsWorld = worldImageLabel->getSelectedPixels();
		
		int numSelectedPixels = selectedPixelsProj->size();
		vector<pair<VectorXd, VectorXd>> correlationPoints(numSelectedPixels);
		
		// Point of interest coordinates.
		int positiveInf = 9999999;
		int negativeInf = -1;
		QPoint minPOICoords(positiveInf, positiveInf);
		QPoint maxPOICoords(negativeInf, negativeInf);
		
		for (int i = 0; i < numSelectedPixels; ++i)
		{
			QPoint qProjectedPoint = (*selectedPixelsProj)[i]->getPos();
			QPoint qWorldPoint = (*selectedPixelsWorld)[i]->getPos();
			
			VectorXd projectedPoint(3);
			projectedPoint << qProjectedPoint.x(), qProjectedPoint.y(), 1.;
			VectorXd worldPoint(3);
			worldPoint << qWorldPoint.x(), qWorldPoint.y(), 1.;
			
			pair<VectorXd, VectorXd> correlationPair(projectedPoint, worldPoint);
			correlationPoints[i] = correlationPair;
			
			// Update point of interest coordinates.
			if (qWorldPoint.x() < minPOICoords.x())
			{
				minPOICoords.setX(qWorldPoint.x());
			}
			if (qWorldPoint.y() < minPOICoords.y())
			{
				minPOICoords.setY(qWorldPoint.y());
			}
			if (qWorldPoint.x() > maxPOICoords.x())
			{
				maxPOICoords.setX(qWorldPoint.x());
			}
			if (qWorldPoint.y() > maxPOICoords.y())
			{
				maxPOICoords.setY(qWorldPoint.y());
			}
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
			return selectPointOfInterest(rectifiedPixmap, minPOICoords, maxPOICoords, worldImageLabel->pixmap()->size());
		}
		else
		{
			return rectifiedPixmap;
		}
	}
	
	QPixmap RectificationController::selectPointOfInterest(const QPixmap& rectifiedImage, const QPoint& minPOICoords,
														   const QPoint& maxPOICoords, const QSize& worldImageSize)
	{
		QSize rectifiedImageSize = rectifiedImage.size();
		
		// Left-upper corner
		QPoint POIOrigin(
			((float)minPOICoords.x() / (float)worldImageSize.width()) * rectifiedImageSize.width(),
			((float)minPOICoords.y() / (float)worldImageSize.height()) * rectifiedImageSize.height()
		);
		
		QSize POISize(
			((float)(maxPOICoords.x() - minPOICoords.x()) / (float)worldImageSize.width()) * rectifiedImageSize.width(),
			((float)(maxPOICoords.y() - minPOICoords.y()) / (float)worldImageSize.height()) * rectifiedImageSize.height()
		);
		
		return rectifiedImage.copy(POIOrigin.x(), POIOrigin.y(), POISize.width(), POISize.height());
	}
}