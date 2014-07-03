#include "SelectedPixel.h"

namespace ui
{
	SelectedPixel::SelectedPixel(QPoint pos, shared_ptr<QIcon>& icon) :
	m_pos(pos), m_icon(icon) {}

	QPoint& SelectedPixel::getPos() { return m_pos; }

	QLabel& SelectedPixel::getLabel() { return m_label; }
}