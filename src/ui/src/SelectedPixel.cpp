#include "SelectedPixel.h"
#include <iostream>

namespace ui
{
	SelectedPixel::SelectedPixel(QPoint pos, QPixmap* pixmap, QWidget* parent)
	: QWidget(parent)
	{
        m_label = new QLabel(this);
        m_label->setPixmap(*pixmap);
        m_label->move(pos);
        m_label->show();
        show();
	}

	QPoint SelectedPixel::getPos() { return m_label->pos(); }

	QLabel* SelectedPixel::getLabel() { return m_label; }
}
