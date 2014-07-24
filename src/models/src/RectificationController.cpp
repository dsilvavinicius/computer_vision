#include <iostream>
#include <Eigen/Dense>
#include "AssistedSimilarityFromProjRectificator.h"
#include "AffineFromProjRectificator.h"
#include "SimilarityFromAffineRectificator.h"
#include "SimilarityFromProjRectificator.h"
#include "RectificationController.h"

using namespace std;
using namespace Eigen;
using namespace math;

namespace models
{
	RectificationController::RectificationController() {}

	QPixmap RectificationController::assistedFromProjectionToSimilarity(ClickableLabel* projectedImageLabel, const QSize& POISize,
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
		AssistedSimilarityFromProjRectificator rectificator(correlationPoints);
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
	
	QPixmap RectificationController::toAffineFromProjection(ClickableLabel* projectedImageLabel)
	{
		CircularList<SelectedPixel*>* points = projectedImageLabel->getSelectedPixels();
		if (points->size() != 8)
		{
			throw logic_error("8 points that define two pairs of parallel lines in affine space should be"
				"selected in the label"
			);
		}
		
		vector<pair<VectorXd, VectorXd>> parallelPairs = pointsToLinesPairs(projectedImageLabel);
		
		AffineFromProjRectificator rectificator(parallelPairs);
		MatrixXd projToAffine = *rectificator.getTransformation();
		
		// Qt uses the transpose of the usual transformation representation.
		QTransform qProjToAffine(
			projToAffine(0, 0), projToAffine(1, 0), projToAffine(2, 0),
			projToAffine(0, 1), projToAffine(1, 1), projToAffine(2, 1),
			projToAffine(0, 2), projToAffine(1, 2), projToAffine(2, 2)
		);
		
		return projectedImageLabel->pixmap()->transformed(qProjToAffine, Qt::SmoothTransformation);
	}
	
	QPixmap RectificationController::toSimilarityFromAffine(ClickableLabel* affineImageLabel)
	{
		CircularList<SelectedPixel*>* points = affineImageLabel->getSelectedPixels();
		if (points->size() != 8)
		{
			throw logic_error("8 points that define two pairs of orthogonal lines in similarity space should be"
				"selected in the label."
			);
		}
		
		vector<pair<VectorXd, VectorXd>> orthoPairs = pointsToLinesPairs(affineImageLabel);
		
		SimilarityFromAffineRectificator rectificator(orthoPairs);
		MatrixXd AffineToSimilarity = *rectificator.getTransformation();
		
		// Qt uses the transpose of the usual transformation representation.
		QTransform qAffineToSimilarity(
			AffineToSimilarity(0, 0), AffineToSimilarity(1, 0), AffineToSimilarity(2, 0),
			AffineToSimilarity(0, 1), AffineToSimilarity(1, 1), AffineToSimilarity(2, 1),
			AffineToSimilarity(0, 2), AffineToSimilarity(1, 2), AffineToSimilarity(2, 2)
		);
		
		return affineImageLabel->pixmap()->transformed(qAffineToSimilarity, Qt::SmoothTransformation);
	}
	
	QPixmap RectificationController::toSimilarityFromProjection(ClickableLabel* projectionImageLabel)
	{
		CircularList<SelectedPixel*>* points = projectionImageLabel->getSelectedPixels();
		if (points->size() != 20)
		{
			throw logic_error("20 points that define five pairs of orthogonal lines in similarity space should be"
				"selected in the label."
			);
		}
		
		vector<pair<VectorXd, VectorXd>> orthoPairs = pointsToLinesPairs(projectionImageLabel);
		SimilarityFromProjRectificator rectificator = SimilarityFromProjRectificator(orthoPairs);
		
		MatrixXd projToSimilarity = *rectificator.getTransformation();
		
		// Qt uses the transpose of the usual transformation representation.
		QTransform qProjToSimilarity(
			projToSimilarity(0, 0), projToSimilarity(1, 0), projToSimilarity(2, 0),
			projToSimilarity(0, 1), projToSimilarity(1, 1), projToSimilarity(2, 1),
			projToSimilarity(0, 2), projToSimilarity(1, 2), projToSimilarity(2, 2)
		);
		
		return projectionImageLabel->pixmap()->transformed(qProjToSimilarity, Qt::SmoothTransformation);
	}
	
	vector<pair<VectorXd, VectorXd>> RectificationController::pointsToLinesPairs(ClickableLabel* label)
	{
		CircularList<SelectedPixel*>* points = label->getSelectedPixels();
		
		vector<pair<VectorXd, VectorXd>> linePairs;
		VectorXd linePair[2];
			
		for (int j = 0; j < points->size() * 0.25f; ++j) // Parallel line pair creation.
		{
			VectorXd line;
			for (int i = 0; i < 2; ++i) // Line creation.
			{
				QPoint qP0 = (*points)[j*4 + i*2]->getPos();
				QPoint qP1 = (*points)[j*4 + i*2 + 1]->getPos();
				Vector3d p0(3);
				p0 << qP0.x(), qP0.y(), 1.;
				Vector3d p1(3);
				p1 << qP1.x(), qP1.y(), 1.;
				Vector3d line = p0.cross(p1);
				line /= line[2];
				
				cout << "Image size: " << endl << label->size().width() << endl
				<< label->size().height() << endl << endl << "P0: " << endl << p0 << endl << endl
				<< "P1: " << endl << p1 << endl << endl << "Line: " << endl << line << endl << endl;
				
				linePair[i] = line;
			}
			linePairs.push_back( pair<VectorXd, VectorXd>(linePair[0], linePair[1]) );
		}
		
		return linePairs;
	}
}