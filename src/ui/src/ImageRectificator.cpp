/****************************************************************************
**
** Copyright (C) 2014 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtGui>
#include <QLabel>
#include <QScrollArea>
#include <QMessageBox>
#include <QAction>
#include <QString>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QScrollBar>
#include <QDesktopWidget>
#include <QRect>
#include <QGridLayout>
#include <Eigen/Dense>
#include <iostream>
#include <cassert>

#include "ImageRectificator.h"
#include "ClickableLabel.h"
#include "AssistedSimilarityFromProjRectificator.h"
#include "RectificationController.h"

using namespace std;
using namespace Eigen;
using namespace math;
using namespace models;

namespace ui
{
    ImageRectificator::ImageRectificator(int maxSelectedPixels)
    {
		m_maxSelectedPixels = maxSelectedPixels;
		
        QLabel* projectedImageSubtitle = new QLabel();
        projectedImageSubtitle->setText("Projected image");

        // TODO: eliminate these magic constants.
        QDir::setCurrent(QCoreApplication::applicationDirPath());
        const QString& iconFileName = tr("../../src/images/icon.png");
        projectedImageLabel = new ClickableLabel(m_maxSelectedPixels, iconFileName);
        projectedImageLabel->setBackgroundRole(QPalette::Base);
        projectedImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        projectedImageLabel->setScaledContents(true);

        projectedScroll = new QScrollArea;
        projectedScroll->setBackgroundRole(QPalette::Dark);
        projectedScroll->setAlignment(Qt::AlignCenter);
        projectedScroll->setWidget(projectedImageLabel);

        QLabel* rectifiedImageSubtitle = new QLabel();
        rectifiedImageSubtitle->setText("Rectified image");

        rectifiedImageLabel = new QLabel;
        rectifiedImageLabel->setBackgroundRole(QPalette::Base);
        rectifiedImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        rectifiedImageLabel->setScaledContents(true);

        rectifiedScroll = new QScrollArea;
        rectifiedScroll->setBackgroundRole(QPalette::Dark);
        rectifiedScroll->setWidget(rectifiedImageLabel);
        rectifiedScroll->setAlignment(Qt::AlignCenter);

        QGridLayout *imagesLayout = new QGridLayout();

        imagesLayout->addWidget(projectedImageSubtitle, 0, 0);
        imagesLayout->addWidget(projectedScroll, 1, 0);
        imagesLayout->addWidget(rectifiedImageSubtitle, 2, 0);
        imagesLayout->addWidget(rectifiedScroll, 3, 0);

        QWidget *window = new QWidget();
        window->setLayout(imagesLayout);

        setCentralWidget(window);

        createActions();
        createMenus();

        setWindowTitle(tr("Image Rectificator"));

        QDesktopWidget desktop;
        QRect res = desktop.availableGeometry(desktop.primaryScreen());
        resize(res.width(), res.height());
    }

    void ImageRectificator::openProjected() { open(projectedImageLabel); }
    
    void ImageRectificator::open(QLabel* targetLabel)
    {
        QString fileName = QFileDialog::getOpenFileName(this,
                                 tr("Open File"), QDir::currentPath());
        if (!fileName.isEmpty()) {
            QPixmap pixMap = QPixmap(fileName);
            if (pixMap.isNull()) {
                QMessageBox::information(this, tr("Image Rectificator"),
                                      tr("Cannot load %1.").arg(fileName));
                return;
            }

            targetLabel->setPixmap(pixMap);
			targetLabel->adjustSize();

            clearSelectedPixAct->setEnabled(true);
			clearSelectedPix();
        }
    }
    
    void ImageRectificator::clearSelectedPix()
	{
		projectedImageLabel->clearSelectedPix();
	}
    
    void ImageRectificator::rectifyAll()
	{
		rectify(QSizeF(81.9f, 61.3f), 640, false); //brahma
		//rectify(QSizeF(18.f, 25.5f), 50, false); //book.
	}
	
	void ImageRectificator::rectifyPointOfInterest()
	{
		rectify(QSizeF(81.9f, 61.3f), 640, true); //brahma
		//rectify(QSizeF(18.f, 25.5f), 50, true); //book.
	}
    
    void ImageRectificator::rectify(const QSizeF& POIRealSize, int desiredWidth, bool pointOfInterestFlag)
	{			
		if (projectedImageLabel->getSelectedPixels()->size() < 4)
		{
			QMessageBox::information(this, tr("Image Rectificator"),
										tr("%1 pixels should be selected before rectification.").arg(m_maxSelectedPixels));
			return;
		}
		
		float aspectRatio = POIRealSize.width() / POIRealSize.height();
		
		QPixmap rectifiedPixmap = RectificationController::assistedFromProjectionToSimilarity(projectedImageLabel,
			QSize(desiredWidth, desiredWidth / (float)aspectRatio), pointOfInterestFlag);
		rectifiedImageLabel->setPixmap(rectifiedPixmap);
		rectifiedImageLabel->adjustSize();
	}
	
	void ImageRectificator::rectifyToAffine()
	{
		if (projectedImageLabel->getSelectedPixels()->size() < 8)
		{
			QMessageBox::information(this, tr("Image Rectificator"),
										tr("%1 pixels should be selected before rectification.").arg(m_maxSelectedPixels));
			return;
		}
		
		QPixmap rectifiedPixmap = RectificationController::toAffineFromProjection(projectedImageLabel);
		rectifiedImageLabel->setPixmap(rectifiedPixmap.scaled(640, 480));
		rectifiedImageLabel->adjustSize();
	}
	
	void ImageRectificator::rectifyFromAffine()
	{
		if (projectedImageLabel->getSelectedPixels()->size() < 8)
		{
			QMessageBox::information(this, tr("Image Rectificator"),
										tr("%1 pixels should be selected before rectification.").arg(m_maxSelectedPixels));
			return;
		}
		
		QPixmap rectifiedPixmap = RectificationController::toSimilarityFromAffine(projectedImageLabel);
		rectifiedImageLabel->setPixmap(rectifiedPixmap.scaled(640, 480));
		rectifiedImageLabel->adjustSize();
	}
	
	void ImageRectificator::rectifyFromProjToSim()
	{
		if (projectedImageLabel->getSelectedPixels()->size() < 20)
		{
			QMessageBox::information(this, tr("Image Rectificator"),
										tr("%1 pixels should be selected before rectification.").arg(m_maxSelectedPixels));
			return;
		}
		
		QPixmap rectifiedPixmap = RectificationController::toSimilarityFromProjection(projectedImageLabel);
		rectifiedImageLabel->setPixmap(rectifiedPixmap.scaled(640, 480));
		rectifiedImageLabel->adjustSize();
	}

    void ImageRectificator::about()
    {
        QMessageBox::about(this, tr("About Image Viewer"),
            tr("<p>The <b>Image Rectificator</b> rectifies an image, eliminating its perspective.<br>"
            "<br><b>Usage</b>:"
            "<ul>"
            "<li>Open File menu, choose Open Projected Image to open the projected image.</li>"
            "<li>Click in 4 points of the projected image that should form a rectangle in the real world.</li>"
            "<li>Open Rectification menu, choose Rectify to transform the projected image.</li>"
			"</ul>"
            "</p>"));
    }

    void ImageRectificator::createActions()
    {
        openProjectedAct = new QAction(tr("&Open Projected Image..."), this);
        openProjectedAct->setShortcut(tr("Ctrl+O"));
        connect(openProjectedAct, SIGNAL(triggered()), this, SLOT(openProjected()));

		rectifyPointOfInterestAct = new QAction(tr("&Rectify Projected Image (Point of Interest only)"), this);
		rectifyPointOfInterestAct->setShortcut(tr("Ctrl+R"));
		connect(rectifyPointOfInterestAct, SIGNAL(triggered()), this, SLOT(rectifyPointOfInterest()));
		
		rectifyAllAct = new QAction(tr("&Rectify Projected Image"), this);
		rectifyAllAct->setShortcut(tr("Ctrl+T"));
		connect(rectifyAllAct, SIGNAL(triggered()), this, SLOT(rectifyAll()));
		
		rectifyToAffineAct = new QAction(tr("&Rectify Projected Image to Affine"), this);
		rectifyToAffineAct->setShortcut(tr("Ctrl+A"));
		connect(rectifyToAffineAct, SIGNAL(triggered()), this, SLOT(rectifyToAffine()));
		
		rectifyFromAffineAct = new QAction(tr("&Rectify Affine Image"), this);
		rectifyFromAffineAct->setShortcut(tr("Ctrl+F"));
		connect(rectifyFromAffineAct, SIGNAL(triggered()), this, SLOT(rectifyFromAffine()));
		
		rectifyFromProjToSimAct = new QAction(tr("&Rectify to Similarity (orthogonal lines)"), this);
		rectifyFromProjToSimAct->setShortcut(tr("Ctrl+S"));
		connect(rectifyFromProjToSimAct, SIGNAL(triggered()), this, SLOT(rectifyFromProjToSim()));
		
        exitAct = new QAction(tr("E&xit"), this);
        exitAct->setShortcut(tr("Ctrl+Q"));
        connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

        clearSelectedPixAct = new QAction(tr("Clear selected pixels"), this);
        clearSelectedPixAct->setShortcut(tr("Ctrl+P"));
        clearSelectedPixAct->setEnabled(false);
        connect(clearSelectedPixAct, SIGNAL(triggered()), this, SLOT(clearSelectedPix()));

        aboutAct = new QAction(tr("&About"), this);
        connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
    }

    void ImageRectificator::createMenus()
    {
        fileMenu = new QMenu(tr("&File"), this);
        fileMenu->addAction(openProjectedAct);
        fileMenu->addSeparator();
        fileMenu->addAction(exitAct);

		rectificationMenu = new QMenu(tr("&Rectification"), this);
		rectificationMenu->addAction(clearSelectedPixAct);
		rectificationMenu->addAction(rectifyAllAct);
		rectificationMenu->addAction(rectifyPointOfInterestAct);
		rectificationMenu->addAction(rectifyToAffineAct);
		rectificationMenu->addAction(rectifyFromAffineAct);
		rectificationMenu->addAction(rectifyFromProjToSimAct);
		
        helpMenu = new QMenu(tr("&Help"), this);
        helpMenu->addAction(aboutAct);

        menuBar()->addMenu(fileMenu);
		menuBar()->addMenu(rectificationMenu);
        menuBar()->addMenu(helpMenu);
    }

    void ImageRectificator::adjustScrollBar(QScrollBar *scrollBar, double factor)
    {
        scrollBar->setValue(int(factor * scrollBar->value()
                            + ((factor - 1) * scrollBar->pageStep()/2)));
    }
}
