#include "ClickableLabel.h"

namespace ui
{
	ClickableLabel::ClickableLabel(const int& maxSelectedPix,
		const QString& iconFileName, QWidget* parent) : QLabel(parent)
	{
		m_selectedPix = make_shared<CircularList<SelectedPixel*>>(maxSelectedPix);
		m_selectionIcon = new QIcon(iconFileName);
	}

    ClickableLabel::~ClickableLabel()
    {
        delete m_selectionIcon;
    }

	void ClickableLabel::clearSelectedPix()
	{
		m_selectedPix->clear();
	}

	void ClickableLabel::mousePressEvent(QMouseEvent *event)
	{
		if (event->button() == Qt::LeftButton)
		{
			SelectedPixel* pixel = new SelectedPixel(event->pos(), m_selectionIcon, this);
			m_selectedPix->pushBack(pixel);
		}
	}
}
