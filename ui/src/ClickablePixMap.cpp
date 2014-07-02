#include "ClickablePixMap.h"

namespace ui
{
	ClickablePixMap::ClickablePixMap(const QString & fileName, const int& maxSelectedPix) : QPixmap(fileName)
	{
		m_selectedPix = make_shared<CircularList<QPoint>>(maxSelectedPix);
	}

	void ClickablePixMap::clearSelectedPix()
	{
		m_selectedPix->clear();
	}

	void ClickablePixMap::mousePressEvent(QMouseEvent *event)
	{
		if (event->button() == Qt::LeftButton)
		{
			m_selectedPix->pushBack(event->pos());
		}
		else
		{
			QCheckBox::mousePressEvent(event);
		}
	}
}