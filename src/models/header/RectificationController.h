#ifndef RECTIFICATION_CONTROLLER_H
#define RECTIFICATION_CONTROLLER_H

#include "ClickableLabel.h"

using namespace ui;

namespace models
{
	class RectificationController
	{
	public:
		static QPixmap rectify(ClickableLabel* projectedImageLabel, ClickableLabel* worldImageLabel);
	private:
		RectificationController();
	};
}
	
#endif // RECTIFICATION_CONTROLLER_H
