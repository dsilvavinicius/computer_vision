#ifndef RECTIFICATION_CONTROLLER_H
#define RECTIFICATION_CONTROLLER_H

#include "ClickableLabel.h"

using namespace ui;

namespace models
{
	class RectificationController
	{
	public:
		//TODO: Change the name of this method to ToSimilarity().
		/** Rectifies the image in a given ClickableLabel from projection space to similarity space. Uses its selected 
		 * pixels, which should be an rectangle in the similarity space (point of interest - POI), the desired POI
		 * size in pixels and a flag that indicates if the final image will show POI only. */
		static QPixmap rectify(ClickableLabel* projectedImageLabel, const QSize& POISize, bool pointOfInterestFlag);
		
		/** Rectifies the image in a given ClickableLabel from projection space to affine space. The label
		 should have 8 eight points selected, which should form two pairs of lines that should be parallel in
		 affine space. */
		static QPixmap toAffine(ClickableLabel* projectedImageLabel);
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
	};
}
	
#endif // RECTIFICATION_CONTROLLER_H
