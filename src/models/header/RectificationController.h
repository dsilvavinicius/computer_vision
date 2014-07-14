#ifndef RECTIFICATION_CONTROLLER_H
#define RECTIFICATION_CONTROLLER_H

#include "ClickableLabel.h"

using namespace ui;

namespace models
{
	class RectificationController
	{
	public:
		static QPixmap rectify(ClickableLabel* projectedImageLabel, const QSize& POISize, bool pointOfInterestFlag);
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
