#ifndef SELECTED_PIXEL_H
#define SELECTED_PIXEL_H

#include <memory>
#include <QWidget>
#include <QPoint>
#include <QLabel>
#include <QIcon>

using namespace std;

namespace ui
{
	class SelectedPixel : public QWidget
	{
        Q_OBJECT
	public:
		SelectedPixel(QPoint pos, QIcon* icon, QWidget* parent = 0);
		QLabel* getLabel();
	private:
		QLabel* m_label;
	};
}

#endif
