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

#include "ImageRectificator.h"
#include "ClickableLabel.h"

namespace ui
{
    ImageRectificator::ImageRectificator()
    {
        QLabel* projectedImageSubtitle = new QLabel();
        projectedImageSubtitle->setText("Projected image");

        // TODO: eliminate these magic constants.
        QDir::setCurrent(QCoreApplication::applicationDirPath());
        const QString& iconFileName = tr("../../src/images/icon.png");
		int maxSelectedPixels = 4;
        projectedImageLabel = new ClickableLabel(maxSelectedPixels, iconFileName);
        projectedImageLabel->setBackgroundRole(QPalette::Base);
        projectedImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        projectedImageLabel->setScaledContents(true);

        projectedScroll = new QScrollArea;
        projectedScroll->setBackgroundRole(QPalette::Dark);
        projectedScroll->setAlignment(Qt::AlignCenter);
        projectedScroll->setWidget(projectedImageLabel);
		
		QLabel* worldImageSubtitle = new QLabel();
        worldImageSubtitle->setText("World image");
		
		worldImageLabel = new ClickableLabel(maxSelectedPixels, iconFileName);
		worldImageLabel->setBackgroundRole(QPalette::Base);
        worldImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        worldImageLabel->setScaledContents(true);
		
		worldScroll = new QScrollArea;
		worldScroll->setBackgroundRole(QPalette::Dark);
        worldScroll->setAlignment(Qt::AlignCenter);
        worldScroll->setWidget(worldImageLabel);

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
		imagesLayout->addWidget(worldImageSubtitle, 0, 1);
        imagesLayout->addWidget(worldScroll, 1, 1);
        imagesLayout->addWidget(rectifiedImageSubtitle, 0, 2);
        imagesLayout->addWidget(rectifiedScroll, 1, 2);

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
            scaleFactor = 1.0;

            fitToWindowAct->setEnabled(true);
            clearSelectedPixAct->setEnabled(true);
            updateActions();

            if (!fitToWindowAct->isChecked())
                targetLabel->adjustSize();
        }
    }

    void ImageRectificator::zoomIn()
    {
        scaleImage(1.25);
    }

    void ImageRectificator::zoomOut()
    {
        scaleImage(0.8);
    }

    void ImageRectificator::normalSize()
    {
        projectedImageLabel->adjustSize();
        rectifiedImageLabel->adjustSize();
        scaleFactor = 1.0;
    }

    void ImageRectificator::fitToWindow()
    {
        bool fitToWindow = fitToWindowAct->isChecked();
        projectedScroll->setWidgetResizable(fitToWindow);
        if (!fitToWindow) {
            normalSize();
        }
        updateActions();
    }

    void ImageRectificator::about()
    {
        QMessageBox::about(this, tr("About Image Viewer"),
            tr("<p>The <b>Image Rectificator</b> rectifies an image, eliminating its perspective.<br>"
            "<br><b>Usage</b>:"
            "<ul>"
            "<li>Open File menu, choose Open to open an image to be rectified.</li>"
            "<li>Click in 4 points of the image that should form a rectangle in the real world.</li>"
            "</ul>"
            "</p>"));
    }

    void ImageRectificator::createActions()
    {
        openProjectedAct = new QAction(tr("&Open Projected Image..."), this);
        openProjectedAct->setShortcut(tr("Ctrl+O"));
        connect(openProjectedAct, SIGNAL(triggered()), this, SLOT(open(projectedImageLabel)));
		
		openWorldAct = new QAction(tr("&Open World Image..."), this);
        openWorldAct->setShortcut(tr("Ctrl+I"));
        connect(openWorldAct, SIGNAL(triggered()), this, SLOT(open(worldImageLabel)));

        exitAct = new QAction(tr("E&xit"), this);
        exitAct->setShortcut(tr("Ctrl+Q"));
        connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

        zoomInAct = new QAction(tr("Zoom &In (25%)"), this);
        zoomInAct->setShortcut(tr("Ctrl++"));
        zoomInAct->setEnabled(false);
        connect(zoomInAct, SIGNAL(triggered()), this, SLOT(zoomIn()));

        zoomOutAct = new QAction(tr("Zoom &Out (25%)"), this);
        zoomOutAct->setShortcut(tr("Ctrl+-"));
        zoomOutAct->setEnabled(false);
        connect(zoomOutAct, SIGNAL(triggered()), this, SLOT(zoomOut()));

        normalSizeAct = new QAction(tr("&Normal Size"), this);
        normalSizeAct->setShortcut(tr("Ctrl+S"));
        normalSizeAct->setEnabled(false);
        connect(normalSizeAct, SIGNAL(triggered()), this, SLOT(normalSize()));

        fitToWindowAct = new QAction(tr("&Fit to Window"), this);
        fitToWindowAct->setEnabled(false);
        fitToWindowAct->setCheckable(true);
        fitToWindowAct->setShortcut(tr("Ctrl+F"));
        connect(fitToWindowAct, SIGNAL(triggered()), this, SLOT(fitToWindow()));

        clearSelectedPixAct = new QAction(tr("Clear selected pixels"), this);
        clearSelectedPixAct->setShortcut(tr("Ctrl+P"));
        clearSelectedPixAct->setEnabled(false);
        connect(clearSelectedPixAct, SIGNAL(triggered()), projectedImageLabel,
            SLOT(clearSelectedPix()));

        aboutAct = new QAction(tr("&About"), this);
        connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
    }

    void ImageRectificator::createMenus()
    {
        fileMenu = new QMenu(tr("&File"), this);
        fileMenu->addAction(openProjectedAct);
		fileMenu->addAction(openWorldAct);
        fileMenu->addSeparator();
        fileMenu->addAction(exitAct);

        viewMenu = new QMenu(tr("&View"), this);
        viewMenu->addAction(zoomInAct);
        viewMenu->addAction(zoomOutAct);
        viewMenu->addAction(normalSizeAct);
        viewMenu->addSeparator();
        viewMenu->addAction(fitToWindowAct);
        viewMenu->addSeparator();
        viewMenu->addAction(clearSelectedPixAct);

        helpMenu = new QMenu(tr("&Help"), this);
        helpMenu->addAction(aboutAct);

        menuBar()->addMenu(fileMenu);
        menuBar()->addMenu(viewMenu);
        menuBar()->addMenu(helpMenu);
    }

    void ImageRectificator::updateActions()
    {
        zoomInAct->setEnabled(!fitToWindowAct->isChecked());
        zoomOutAct->setEnabled(!fitToWindowAct->isChecked());
        normalSizeAct->setEnabled(!fitToWindowAct->isChecked());
    }

    void ImageRectificator::scaleImage(double factor)
    {
        Q_ASSERT(projectedImageLabel->pixmap());

        scaleFactor *= factor;
        projectedImageLabel->scale(factor);

        adjustScrollBar(projectedScroll->horizontalScrollBar(), factor);
        adjustScrollBar(projectedScroll->verticalScrollBar(), factor);

        if (rectifiedImageLabel->pixmap())
        {
            rectifiedImageLabel->resize(scaleFactor * rectifiedImageLabel->pixmap()->size());
            adjustScrollBar(rectifiedScroll->horizontalScrollBar(), factor);
            adjustScrollBar(rectifiedScroll->verticalScrollBar(), factor);
        }

        zoomInAct->setEnabled(scaleFactor < 3.0);
        zoomOutAct->setEnabled(scaleFactor > 0.333);
    }

    void ImageRectificator::adjustScrollBar(QScrollBar *scrollBar, double factor)
    {
        scrollBar->setValue(int(factor * scrollBar->value()
                            + ((factor - 1) * scrollBar->pageStep()/2)));
    }
}
