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
		/** Resizes and translate the rectified image to show the point of interest. */
		static QPixmap selectPointOfInterest(const QPixmap& rectifiedImage, const QPoint& minPOICoords,
											 const QPoint& maxPOICoords, const QSize& worldImageSize);
	};
}
	
#endif // RECTIFICATION_CONTROLLER_H
