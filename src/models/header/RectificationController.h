#ifndef RECTIFICATION_CONTROLLER_H
#define RECTIFICATION_CONTROLLER_H

#include "ClickableLabel.h"

using namespace ui;

namespace models
{
	class RectificationController
	{
	public:
		/** Rectifies the image in a given ClickableLabel from projection space to similarity space. Uses its selected 
		 * pixels, which should be an rectangle in the similarity space (point of interest - POI), the desired POI
		 * size in pixels and a flag that indicates if the final image will show POI only. */
		static QPixmap assistedFromProjectionToSimilarity(ClickableLabel* projectedImageLabel, const QSize& POISize, bool pointOfInterestFlag);
		
		/** Rectifies the image in a given ClickableLabel from projection space to affine space. The label
		 should have 8 points selected, which should form two pairs of parallel lines in affine space. */
		static QPixmap toAffineFromProjection(ClickableLabel* projectedImageLabel);
		
		/** Rectifies the image in a given ClickableLabel from affine space to similarity space. The label
		 should have 8 points selected, which should form two pairs of orthogonal lines in similarity space. */
		static QPixmap toSimilarityFromAffine(ClickableLabel* affineImageLabel);
		
		/** Rectifies the image in a given ClickableLabel from projection space to similarity space. The label
		 should have 20 points selected, which should form five pairs of orthogonal lines in similarity space. */
		static QPixmap toSimilarityFromProjection(ClickableLabel* projectionImageLabel);
	private:
		RectificationController();
		
		/** Resizes and translate the rectified image to show the point of interest.
		 * @param rectifiedImage is the rectified image, already resized to contain all transformed points.
		 * @param rectification is the rectification transformation passed to Qt.
		 * @param originalImageSize is the size of the original projection image.
		 * @param POIOrigin is the point of interest origin (upper-left corner) in projection image coordinates.
		 * @param POISize is the desired size of the point of interest.
		 */
		static QPixmap selectPointOfInterest(const QPixmap& rectifiedImage, const QTransform& rectification, const QSize& originalImageSize,
											 const QPoint& POIOrigin, const QSize& POISize);
		
		/** Given a ClickableLabel with selected points {p_0, p_1, ... p_2k}, creates a vector l with k lines, such that
		 * l_i passes through p_2i and p_(2i + 1). Then, creates a vector of line pairs lp, such that
		 * lp_j = { l_2j, l_(2j + 1) }. */
		static vector<pair<VectorXd, VectorXd>> pointsToLinesPairs(ClickableLabel* label);
	};
}
	
#endif // RECTIFICATION_CONTROLLER_H
