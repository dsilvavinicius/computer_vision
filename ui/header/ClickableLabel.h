#ifndef CLICKABLE_LABEL_H
#define CLICKABLE_LABEL_H

#include <QLabel>
#include <QIcon>
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
		ClickableLabel(const int& maxSelectedPix, const QString& iconFileName, QWidget* parent = 0);
        ~ClickableLabel();
	protected:
		virtual void mousePressEvent(QMouseEvent *event);

		CircularListPtr<SelectedPixel*> m_selectedPix;
		QIcon* m_selectionIcon;
	};
}

#endif
