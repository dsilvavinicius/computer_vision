#ifndef SELECTED_PIXEL_H
#define SELECTED_PIXEL_H

#include <memory>
#include <QWidget>
#include <QPoint>
#include <QLabel>
#include <QIcon>
#include <QResizeEvent>

using namespace std;

namespace ui
{
	class SelectedPixel : public QWidget
	{
        Q_OBJECT
	public:
		SelectedPixel(QPoint pos, QPixmap* pixmap, QWidget* parent = 0);
		QPoint getPos();
		QLabel* getLabel();
    protected:
        //virtual void moveEvent(QMoveEvent* event);
	private:
		QLabel* m_label;
	};
}

#endif
