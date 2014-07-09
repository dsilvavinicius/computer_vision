#include <Eigen/Dense>
#include "ProjectionRectificator.h"
#include "RectificationController.h"

using namespace Eigen;
using namespace math;

namespace models
{
	RectificationController::RectificationController() {}

	QPixmap RectificationController::rectify(ClickableLabel* projectedImageLabel, ClickableLabel* worldImageLabel)
	{	
		// First, create pairs of point correlations between projection and world space.
		CircularList<SelectedPixel*>* selectedPixelsProj = projectedImageLabel->getSelectedPixels();
		CircularList<SelectedPixel*>* selectedPixelsWorld = worldImageLabel->getSelectedPixels();
		
		int numSelectedPixels = selectedPixelsProj->size();
		vector<pair<VectorXd, VectorXd>> correlationPoints(numSelectedPixels);
		
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
		
		return projectedImageLabel->pixmap()->transformed(qProjToWorld, Qt::SmoothTransformation);
	}
}