#include "ClickableLabel.h"
#include <sstream>

namespace ui
{
	ClickableLabel::ClickableLabel(const int& maxSelectedPix,
		const QString& pixmapFileName, QWidget* parent) : QLabel(parent)
	{
		m_selectedPix = new CircularList<SelectedPixel*>(maxSelectedPix);
		m_selectionPixmap = new QPixmap(pixmapFileName);
		*m_selectionPixmap = m_selectionPixmap->scaled(10, 10);
		if(m_selectionPixmap->isNull())
		{
            throw logic_error("Cannot find icon image: " + pixmapFileName.toStdString());
		}
	}

    ClickableLabel::~ClickableLabel()
    {
        clearSelectedPix();
        delete m_selectedPix;
        delete m_selectionPixmap;
    }

    void ClickableLabel::scale(double factor)
    {
        resize(factor * pixmap()->size());
	}

    void ClickableLabel::clearSelectedPix()
    {
		vector<SelectedPixel*> selectedPixels = m_selectedPix->clear();
		for (SelectedPixel* pixel : selectedPixels) { delete pixel; }
    }

    void ClickableLabel::mousePressEvent(QMouseEvent *event)
    {
	    if (event->button() == Qt::LeftButton)
	    {
		    SelectedPixel* pixel = new SelectedPixel(event->pos(), m_selectionPixmap, this);
		    SelectedPixel* previous = m_selectedPix->push(pixel);
		    delete previous;
	    }
    }

    void ClickableLabel::resizeEvent(QResizeEvent* event)
    {
        QSize oldSize = event->oldSize();
        QSize newSize = event->size();
        for (int i = 0; i < m_selectedPix->size(); ++i)
        {
            SelectedPixel* pixel = (*m_selectedPix)[i];

            QPoint oldPos = pixel->getPos();
            QPoint normalizedPos;
            normalizedPos.setX(oldPos.x() / oldSize.rwidth());
            normalizedPos.setY(oldPos.y() / oldSize.rheight());

            QPoint newPos;
            newPos.setX(normalizedPos.x() * newSize.rwidth());
            newPos.setY(normalizedPos.y() * newSize.rheight());

            pixel->move(newPos);
        }
    }
    
    // TODO: Return a proper copy of the list.
    CircularList<SelectedPixel*>* ClickableLabel::getSelectedPixels()
	{
		return m_selectedPix;
	}
}
