#ifndef CLICKABLE_LABEL_H
#define CLICKABLE_LABEL_H

#include <QLabel>
#include <QPixmap>
#include <QMouseEvent>
#include <QPoint>
#include <set>
#include "CircularList.h"
#include "SelectedPixel.h"

using namespace utils;

namespace ui
{
	class ClickableLabel : public QLabel
	{
		Q_OBJECT
	public slots:
		void clearSelectedPix();
	public:
		ClickableLabel(const int& maxSelectedPix, const QString& pixmapFileName, QWidget* parent = 0);
        ~ClickableLabel();
        void scale(double factor);
		CircularList<SelectedPixel*>* getSelectedPixels();
	protected:
		virtual void mousePressEvent(QMouseEvent *event);
		virtual void resizeEvent(QResizeEvent * event);
		
		CircularList<SelectedPixel*>* m_selectedPix;
		QPixmap* m_selectionPixmap;
		//CircularList<pair<QPoint, QPoint>>* m_selectedLines;
	};
}

#endif
