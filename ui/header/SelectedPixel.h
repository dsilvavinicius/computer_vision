#ifndef SELECTED_PIXEL_H
#define SELECTED_PIXEL_H

#include <memory>

using namespace std;

namespace ui
{
	class SelectedPixel
	{
	public:
		SelectedPixel(QPoint pos, shared_ptr<QIcon>& icon);
		QPoint& getPos();
		QLabel& getLabel();
	private:
		QPoint m_pos;
		shared_ptr<QIcon> m_icon;
	};

	typedef shared_ptr<SelectedPixel> SelectedPixelPtr;
}

#endif