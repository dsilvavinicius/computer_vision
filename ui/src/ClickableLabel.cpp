#include "ClickableLabel.h"

namespace ui
{
	ClickableLabel::ClickableLabel(const int& maxSelectedPix,
		const QString& iconFileName, QWidget* parent) : QLabel(parent)
	{
		m_selectedPix = make_shared<CircularList<QPoint>>(maxSelectedPix);
		m_selectionIcon = make_shared<QIcon>(iconFileName);
	}

	void ClickableLabel::clearSelectedPix()
	{
		m_selectedPix->clear();
	}

	void ClickableLabel::mousePressEvent(QMouseEvent *event)
	{
		if (event->button() == Qt::LeftButton)
		{
			SelectedPixel pixel = SelectedPixel(event->pos(), m_selectionIcon);
			m_selectedPix->pushBack(pos);
		}
	}
}