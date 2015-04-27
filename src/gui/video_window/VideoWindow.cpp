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

#include <opencv_workbench/utils/SylloQt.h>

#include "VideoWindow.h"

using std::cout;
using std::endl;

VideoWindow::VideoWindow(QWidget *parent)
     : QWidget(parent)
{
     ui.setupUi(this);

     readSettings();

     state_ = none;
     
     connect(ui.frame_slider, SIGNAL(sliderMoved(int)), this, SLOT(set_frame_num_from_slider(int)));
     connect(ui.frame_slider, SIGNAL(sliderReleased()), this, SLOT(slider_released()));

     connect(ui.frame_num_spinbox, SIGNAL(valueChanged(int)), this, SLOT(set_frame_num_from_spinbox(int)));
     
     connect(ui.play_button, SIGNAL(released()), this, SLOT(space_bar()));   
     
     // Keyboard shortcuts
     // Note: Get deleted automatically when program closes.
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(open()));     
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
     new QShortcut(QKeySequence(Qt::Key_Space), this, SLOT(space_bar()));
     new QShortcut(QKeySequence(Qt::Key_Left), this, SLOT(back_one_frame()));
     new QShortcut(QKeySequence(Qt::Key_Right), this, SLOT(step_one_frame()));
     new QShortcut(QKeySequence(Qt::Key_Up), this, SLOT(double_frame_rate()));
     new QShortcut(QKeySequence(Qt::Key_Down), this, SLOT(divide_frame_rate()));
     
     timer_ = new QTimer(this);

     connect(timer_, SIGNAL(timeout()), this, SLOT(timer_loop()));     
}

void VideoWindow::set_frame_num_from_slider(int frame_num) 
{
     if (state_ == paused) {
          stream_.set_frame_number(frame_num);          
     }     
     stream_.set_frame_number(frame_num);     
}

void VideoWindow::slider_released()
{     
     this->draw();
}

void VideoWindow::set_frame_num_from_spinbox(int frame_num)
{
     stream_.set_frame_number(frame_num);     
     this->draw();
}

void VideoWindow::play()
{
     timer_->start();
     state_ = playing;
     ui.play_button->setIcon(QIcon(":/resources/pause.png"));
}

void VideoWindow::pause()
{
     timer_->stop();
     state_ = paused;
     ui.play_button->setIcon(QIcon(":/resources/play.png"));
}

void VideoWindow::set_fps(double fps)
{
     fps_ = fps;
     if (fps_ < 0.1) {
          fps_ = 0.1;
     }
     timer_->setInterval(1000.0/fps_);
}

void VideoWindow::back_one_frame()
{ 
     stream_.set_frame_number(stream_.get_frame_number()-2);
     this->draw();
}

void VideoWindow::step_one_frame()
{     
     stream_.set_frame_number(stream_.get_frame_number());
     this->draw();
}

void VideoWindow::double_frame_rate()
{
     this->set_fps(fps_*2.0);
}

void VideoWindow::divide_frame_rate()
{
     this->set_fps(fps_/2.0);
}

void VideoWindow::closeEvent(QCloseEvent *event)
{
     this->writeSettings();
}

void VideoWindow::space_bar()
{
     if (state_ == playing) {
          this->pause();          
     } else if (state_ == paused) {          
          this->set_fps(fps_);
          this->play();          
     }
}

void VideoWindow::timer_loop()
{
     this->draw();     
}

void VideoWindow::draw()
{
     if (!stream_.isOpened()) {
          cout << "Stream is not open" << endl;
          return;
     }
     
     if (stream_.type() == syllo::ImageType || !stream_.read(curr_image_)) {
          cout << "Done" << endl;
          this->pause();
          stream_.set_frame_number(0);
          return;
     }
     
     if (stream_.isLive()) {                    
     } else {
          // Set all appropriate GUI elements on each frame
          // CV_CAP_PROP_POS_FRAMES is 0-based index of
          // NEXT frame to be captured, we want the currently
          // displayed frame number
          ui.frame_slider->setValue(stream_.get_frame_number()-1);
          ui.frame_num_spinbox->setValue(stream_.get_frame_number()-1);
     }
          
     visible_img_ = curr_image_.clone();                    

     this->draw_image(visible_img_);     
     this->adjustSize();          
}

void VideoWindow::draw_image(const cv::Mat &img)
{
     q_image = SylloQt::Mat2QImage(img);
     ui.image_frame->setPixmap(QPixmap::fromImage(q_image));
     ui.image_frame->adjustSize();
}

void VideoWindow::open_camera(int id)
{
     if (stream_.isOpened()) {
          stream_.release();
     }          

     syllo::Status status = stream_.open(id);
     if (status == syllo::Success && stream_.isOpened()) {
               
          stream_.set_frame_number(0);
          stream_.read(curr_image_);          
          this->draw_image(curr_image_);               

          ui.filename_label->setText("Camera");

          this->set_fps(29);
          this->play();
     }
}

void VideoWindow::open()
{    
     this->pause();

     // If the prev_open_path variable is set, use it, otherwise, use
     // the current directory.
     QString dir = QDir::currentPath();
     if (prev_open_path_ != "") {
          dir = prev_open_path_;
     }
     
     QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), dir);
     this->open(fileName);     
}

void VideoWindow::open(QString fileName)
{
     if (!fileName.isEmpty()) {
          // Save the previously opened directory
          prev_open_path_ = QFileInfo(fileName).path();         

          std::string fn = fileName.toStdString();
          
          syllo::Status status = stream_.open(fn);          

          std::string ext = fn.substr(fn.find_last_of(".") + 1);
          std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

          // If the file is a sonar file and there is an associated avi file
          // We need to be in dual display (sonar/video) mode
          if (ext == "SON") {
               std::string query_file = fn.substr(0,fn.find_last_of(".")) + std::string(".avi");
               cout << "Looking for video file: " << query_file << endl;              
          }

          if (status == syllo::Success && stream_.isOpened()) {                              
               
               // set slider range
               ui.frame_slider->setRange(0,stream_.get_frame_count()-1);
               ui.frame_num_spinbox->setRange(0,stream_.get_frame_count()-1);
               ui.filename_label->setText(QFileInfo(fileName).fileName());
               
               fps_ = stream_.get_fps();
               if (fps_ <= 0) {
                    fps_ = 23;
               }
               this->set_fps(fps_);
               
               stream_.set_frame_number(0);
               this->draw();
               this->pause();               
          }
     }
}

void VideoWindow::about()
{
     QMessageBox::about(this, tr("About The OpenCV Workbench"),
                        tr("<p><b>The OpenCV Workbench</b> is a GUI application that simplifies the computer vision prototyping process.</p><p>Author: Kevin DeMarco (kevin.demarco@gmail.com)</p>"));
}

void VideoWindow::writeSettings()
{
     QSettings settings;

     // Window size and position
     settings.beginGroup("VideoWindow");
     settings.setValue("size", size());
     settings.setValue("pos", pos());

     // Previously opened directory
     settings.setValue("prev_open_path", prev_open_path_);
               
     settings.endGroup();
}

void VideoWindow::readSettings()
{
     QSettings settings;
     
     settings.beginGroup("VideoWindow");
     
     // Window size
     resize(settings.value("size", QSize(400, 400)).toSize());
     move(settings.value("pos", QPoint(200, 200)).toPoint());

     // Previously opened directories
     prev_open_path_ = settings.value("prev_open_path").toString();
          
     settings.endGroup();
}
