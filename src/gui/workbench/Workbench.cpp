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

#include "Workbench.h"

using std::cout;
using std::endl;

Workbench::Workbench(QMainWindow *parent)
     : QMainWindow(parent)
{
     ui.setupUi(this);

     readSettings();
     
     connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
     connect(ui.actionQuit, SIGNAL(triggered()), this, SLOT(close()));
     connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
     
     // Keyboard shortcuts
     // Note: Get deleted automatically when program closes.
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(open()));
     //new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), this, SLOT(open_camera()));
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
}

void Workbench::open()
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

          // Send to VideoWindow Widget
          //video_window_dialog_ = new QDialog(this);
          //video_window_ = new VideoWindow(video_window_dialog_);
          video_window_ = new VideoWindow();
          
          video_window_->open(fileName);          
          //connect(video_window_, SIGNAL(start_labeling()), this, SLOT(start_video_windowing()));          
          //video_window_dialog_->show();
          video_window_->show();
     }     
}

void Workbench::about()
{
     QMessageBox::about(this, tr("About The OpenCV Workbench"),
                        tr("<p><b>The OpenCV Workbench</b> is a GUI application that simplifies the computer vision prototyping process.</p><p>Author: Kevin DeMarco (kevin.demarco@gmail.com)</p>"));
}

void Workbench::writeSettings()
{
     QSettings settings;

     // Window size and position
     settings.beginGroup("MainWindow");
     settings.setValue("size", size());
     settings.setValue("pos", pos());

     // Previously opened directory
     settings.setValue("prev_open_path", prev_open_path_);
               
     settings.endGroup();
}

void Workbench::readSettings()
{
     QSettings settings;
     
     settings.beginGroup("MainWindow");
     
     // Window size
     resize(settings.value("size", QSize(400, 400)).toSize());
     move(settings.value("pos", QPoint(200, 200)).toPoint());

     // Previously opened directories
     prev_open_path_ = settings.value("prev_open_path").toString();
               
     settings.endGroup();
}

void Workbench::closeEvent(QCloseEvent *event)
{
     this->writeSettings();
}
