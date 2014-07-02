#ifndef CLICKABLE_PIX_MAP_H
#define CLICKABLE_PIX_MAP_H

#include <QPixmap>
#include <QMouseEvent>
#include <QPoint>
#include <set>
#include "CircularList.h"

using namespace utils; 

namespace ui
{
	class ClickablePixMap : public QPixmap
	{
		Q_OBJECT
	public slots:
		void clearSelectedPix();
	public:
		ClickablePixMap(const QString & fileName, const int& maxSelectedPix);		
	protected:
		 virtual void mousePressEvent(QMouseEvent *event);

		 CircularListPtr<QPoint> m_selectedPix;
	};
}

#endif