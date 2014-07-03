#ifndef CLICKABLE_LABEL_H
#define CLICKABLE_LABEL_H

#include <QLabel>
#include <QIcon>
#include <QMouseEvent>
#include <QPoint>
#include <set>
#include "SelectedPixel.h"
#include "CircularList.h"

using namespace utils; 

namespace ui
{
	class ClickableLabel : public QLabel
	{
		Q_OBJECT
	public slots:
		void clearSelectedPix();
	public:
		ClickableLabel(const int& maxSelectedPix, const QString& iconFileName,
			QWidget* parent = 0);		
	protected:
		virtual void mousePressEvent(QMouseEvent *event);

		CircularListPtr<SelectedPixel> m_selectedPix;
		shared_ptr<QIcon> m_selectionIcon;
	};
}

#endif