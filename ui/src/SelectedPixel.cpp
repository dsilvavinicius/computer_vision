#include "SelectedPixel.h"

namespace ui
{
	SelectedPixel::SelectedPixel(QPoint pos, QIcon* icon, QWidget* parent)
	: QWidget(parent)
	{
        m_label = new QLabel(this);
        m_label->setPixmap(icon->pixmap(10));
        m_label->move(pos);
	}

	QLabel* SelectedPixel::getLabel() { return m_label; }
}
