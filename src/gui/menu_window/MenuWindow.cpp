/****************************************************************************
 **
 ** Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
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
#include <QResource>
#include <QSettings>

#include "MenuWindow.h"

using std::cout;
using std::endl;

MenuWindow::MenuWindow(QMainWindow *parent)
     : QMainWindow(parent)
{
     ui.setupUi(this);

     central = new QWidget(this);
     setCentralWidget(central);
     verticalLayout = new QVBoxLayout(central);

     readSettings();     
     
     //QMainWindow::centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);
     
     connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
     connect(ui.actionQuit, SIGNAL(triggered()), this, SLOT(close()));
     connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
     connect(ui.actionExport_ROI, SIGNAL(triggered()), this, SLOT(export_roi()));
     
     // Keyboard shortcuts
     // Note: Get deleted automatically when program closes.
     ann_ = new Annotate();
     verticalLayout->addWidget(ann_);
     
     ann_->add_shortcut("Qt::CTRL + Qt::Key_O", QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(open()));
     ann_->add_shortcut("Qt::CTRL + Qt::Key_Q", QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));          

     //ann_->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Minimum);
     //ui.verticalLayout->addWidget(ann_,0,0);
     //ui.verticalLayout->addWidget(ann_);     

     //timer_refresh_ = new QTimer(this);
     //connect(timer_refresh_, SIGNAL(timeout()), this, SLOT(timer_refresh_loop()));     
     //timer_refresh_fps_ = 30;
     //timer_refresh_->setInterval(1000.0/timer_refresh_fps_);
     //timer_refresh_->start();     
}

void MenuWindow::timer_refresh_loop()
{
     //cout << "Ann: " << ann_->sizeHint().rheight() << "," <<  ann_->sizeHint().rwidth() << endl;      
     
     //this->updateGeometry();
     //this->adjustSize();
}

void MenuWindow::export_roi()
{
     ann_->export_roi();
}

void MenuWindow::open()
{
     // If the prev_open_path variable is set, use it, otherwise, use
     // the current directory.
     QString dir = QDir::currentPath();
     if (prev_open_path_ != "") {
          dir = prev_open_path_;
     }
     
     QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), dir);     
     if (!fileName.isEmpty()) {
          prev_open_path_ = QFileInfo(fileName).path();

          ann_->open(fileName);
          ann_->updateGeometry();
          ann_->adjustSize();
          //this->resize(central->sizeHint().rwidth()+50, central->sizeHint().rheight());
          //this->resize(verticalLayout->sizeHint().rwidth()+50, verticalLayout->sizeHint().rheight());
          
          //cout << "vert: " << verticalLayout->sizeHint().rwidth() << "," << verticalLayout->sizeHint().rheight() << endl;
          
          //QSize size = ann_->GoodSize();
          //size.setHeight(size.height()+30);
          //this->resize(size);

          //this->updateGeometry();
          //this->adjustSize();
          
          //// Free up memory from any windows that have been closed already:
          //std::vector<VideoWindow *>::iterator it = video_windows_.begin();
          //for(; it != video_windows_.end(); ) {
          //     if (!((*it)->isVisible())) {
          //          delete *it;
          //          video_windows_.erase(it);
          //     } else {
          //          it++;
          //     }
          //}          
          //// Send to VideoWindow Widget
          //video_windows_.push_back(new VideoWindow());          
          //video_windows_.back()->open(fileName);          
          ////connect(video_window_, SIGNAL(start_labeling()), this, SLOT(start_video_windowing()));                              
          //video_windows_.back()->raise();
          //video_windows_.back()->activateWindow();
          //video_windows_.back()->setFocus(Qt::ActiveWindowFocusReason);
          //video_windows_.back()->showNormal();          
     }     
}

void MenuWindow::about()
{
     QMessageBox::about(this, tr("About The OpenCV Workbench"),
                        tr("<p><b>The OpenCV Workbench</b> is a GUI application that simplifies the computer vision prototyping process.</p><p>Author: Kevin DeMarco (kevin.demarco@gmail.com)</p>"));
}

void MenuWindow::writeSettings()
{
     QSettings settings;

     // Window size and position
     settings.beginGroup("MenuWindow");
     settings.setValue("size", size());
     settings.setValue("pos", pos());

     // Previously opened directory
     settings.setValue("prev_open_path", prev_open_path_);
               
     settings.endGroup();
}

void MenuWindow::readSettings()
{
     QSettings settings;
     
     settings.beginGroup("MenuWindow");
     
     // Window size
     resize(settings.value("size", QSize(400, 400)).toSize());
     move(settings.value("pos", QPoint(200, 200)).toPoint());

     // Previously opened directories
     prev_open_path_ = settings.value("prev_open_path").toString();
               
     settings.endGroup();
}

void MenuWindow::closeEvent(QCloseEvent *event)
{
     this->writeSettings();
}
