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
#include <opencv_workbench/syllo/syllo.h>

#include "VideoWindow.h"

using std::cout;
using std::endl;

VideoWindow::VideoWindow(QWidget *parent)
     : QWidget(parent)
{     
     ui.setupUi(this);

     readSettings();

     state_ = none;

     tooltip_enabled_ = false;     
     
     connect(ui.frame_slider, SIGNAL(sliderMoved(int)), this, SLOT(set_frame_num_from_slider(int)));
     connect(ui.frame_slider, SIGNAL(sliderReleased()), this, SLOT(slider_released()));

     connect(ui.frame_num_spinbox, SIGNAL(editingFinished()), this, SLOT(set_frame_num_from_spinbox()));
     
     connect(ui.play_button, SIGNAL(released()), this, SLOT(space_bar()));   
     
     add_shortcut("Qt::CTRL + Qt::Key_O", QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(open()));
     add_shortcut("Qt::CTRL + Qt::Key_Q", QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
     add_shortcut("Qt::Key_Space", QKeySequence(Qt::Key_Space), this, SLOT(space_bar()));
     add_shortcut("Qt::Key_Left", QKeySequence(Qt::Key_Left), this, SLOT(back_one_frame()));
     add_shortcut("Qt::Key_Right", QKeySequence(Qt::Key_Right), this, SLOT(step_one_frame()));
     add_shortcut("Qt::Key_Up", QKeySequence(Qt::Key_Up), this, SLOT(double_frame_rate()));
     add_shortcut("Qt::Key_Down", QKeySequence(Qt::Key_Down), this, SLOT(divide_frame_rate()));
     add_shortcut("Qt::Key_T", QKeySequence(Qt::Key_T), this, SLOT(tooltip_enabled()));          
     
     //// Keyboard shortcuts
     //// Note: Get deleted automatically when program closes.
     ////new QShortcut(QKeySequence(Qt::Key_Space), this, SLOT(space_bar()));
     //new QShortcut(QKeySequence(Qt::Key_Left), this, SLOT(back_one_frame()));
     //new QShortcut(QKeySequence(Qt::Key_Right), this, SLOT(step_one_frame()));
     //new QShortcut(QKeySequence(Qt::Key_Up), this, SLOT(double_frame_rate()));
     //new QShortcut(QKeySequence(Qt::Key_Down), this, SLOT(divide_frame_rate()));
     
     timer_video_ = new QTimer(this);
     timer_refresh_ = new QTimer(this);

     connect(timer_video_, SIGNAL(timeout()), this, SLOT(timer_video_loop()));     
     connect(timer_refresh_, SIGNAL(timeout()), this, SLOT(timer_refresh_loop()));
     
     connect(ui.image_frame, SIGNAL(mouseMoved(QMouseEvent*)), this, SLOT(mouseMoved(QMouseEvent*)));     

     timer_refresh_fps_ = 30;
     timer_refresh_->setInterval(1000.0/timer_refresh_fps_);
     timer_refresh_->start();     
}

void VideoWindow::add_shortcut(std::string name, const QKeySequence & key, QWidget * parent, const char * member = 0)
{
     if (shortcuts_.count(name) > 0) {
          std::map<std::string, QShortcut*>::iterator it = shortcuts_.find(name);
          delete it->second;
          shortcuts_.erase(it);
     }
     shortcuts_[name] = new QShortcut(key, parent, member);
}

void VideoWindow::mouseMoved(QMouseEvent * event)
{     
     mouse_pos_ = event->pos();
     this->on_mouseMoved(event);
}

// override this
void VideoWindow::on_mouseMoved(QMouseEvent * event)
{
}

void VideoWindow::set_frame_num_from_slider(int frame_num) 
{
     //if (state_ == paused) {
     //     stream_.set_frame_number(frame_num);          
     //}     
     stream_.set_frame_number(frame_num);     
}

void VideoWindow::slider_released()
{    
     this->get_video_frame();
}

void VideoWindow::set_frame_num_from_spinbox()
{     
     stream_.set_frame_number(ui.frame_num_spinbox->value());     
     this->get_video_frame();
}

void VideoWindow::play()
{
     timer_video_->start();
     state_ = playing;
     ui.play_button->setIcon(QIcon(":/resources/pause.png"));
}

void VideoWindow::pause()
{
     timer_video_->stop();
     state_ = paused;
     ui.play_button->setIcon(QIcon(":/resources/play.png"));
}

void VideoWindow::set_fps(double fps)
{
     fps_ = fps;
     if (fps_ < 0.1) {
          fps_ = 0.1;
     }
     timer_video_->setInterval(1000.0/fps_);
}

void VideoWindow::back_one_frame()
{ 
     stream_.step_backward();
     this->get_video_frame();
}

void VideoWindow::step_one_frame()
{          
     stream_.step_forward();
     this->get_video_frame();
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

void VideoWindow::timer_video_loop()
{
     this->step_one_frame();
}

void VideoWindow::timer_refresh_loop()
{
     this->draw();
}

void VideoWindow::get_video_frame()
{
     this->before_next_frame();
     
     if (!stream_.isOpened()) {
          cout << "Stream is not open" << endl;
          return;
     }
     
     if (stream_.type() == syllo::ImageType) {
          this->pause();
          stream_.set_frame_number(0);
          return;
     }

     // Returns false on the last valid frame
     bool status = stream_.read(curr_image_);
     
     if (stream_.isLive()) {                    
     } else {
          // Set all appropriate GUI elements on each frame
          // CV_CAP_PROP_POS_FRAMES is 0-based index of
          // NEXT frame to be captured, we want the currently
          // displayed frame number                    
          ui.frame_slider->setValue(stream_.frame_number());
          ui.frame_num_spinbox->setValue(stream_.frame_number());
     }

     if (!status) {
          this->pause();
          stream_.set_frame_number(0);
     }

     this->draw();
}

void VideoWindow::draw()
{     
     if (curr_image_.empty()) {
          return;
     }
     visible_img_ = curr_image_.clone();

     this->before_display(visible_img_);
     this->draw_tooltip(visible_img_);
     this->display_image(visible_img_);     
     
     this->updateGeometry();
     this->adjustSize();     
     
     //cout << "This: " << this->sizeHint().rheight() << "," <<  this->sizeHint().rwidth() << endl; 
     //cout << "ImageFrame: " << ui.image_frame->sizeHint().rheight() << "," <<  ui.image_frame->sizeHint().rwidth() << endl; 
}

//QSize VideoWindow::sizeHint() const
//{
//     //this->updateGeometry();
//     //this->adjustSize();
//
//     QSize size;
//     size.setWidth(ui.verticalLayoutWidget->sizeHint().rwidth());
//     size.setHeight(ui.verticalLayoutWidget->sizeHint().rheight());
//     
//     //cout << "Video: " << size.rheight() << "," <<  size.rwidth() << endl;      
//     
//     //QSize size(-1,-1);     
//     return size;
//}
//
//void VideoWindow::resizeEvent(QResizeEvent *)
//{
//     //cout << "Resize" << endl;
//     this->resize(ui.verticalLayoutWidget->sizeHint().rwidth(), ui.verticalLayoutWidget->sizeHint().rheight());
//}

void VideoWindow::tooltip_enabled()
{
     tooltip_enabled_ = !tooltip_enabled_;
}

void VideoWindow::draw_tooltip(cv::Mat &img)
{
     if (tooltip_enabled_) {
          std::string str = syllo::int2str(mouse_pos_.x()) + "," + 
               syllo::int2str(mouse_pos_.y());
          
          cv::Point p = cv::Point(mouse_pos_.x(),mouse_pos_.y());
          cv::Point p1 = cv::Point(mouse_pos_.x()-3,mouse_pos_.y()+3);
          cv::Point p2 = cv::Point(mouse_pos_.x()+65,mouse_pos_.y()-15);
          cv::rectangle(img, p1, p2, cv::Scalar(20,255,57),-1,8,0);
          cv::putText(img, str, p, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.75, 
                      cv::Scalar(0,0,0), 1, 8, false);
     }
}

// Overriden by subclasses
void VideoWindow::before_display(cv::Mat &img)
{
}

// Overriden by subclasses
void VideoWindow::before_next_frame()
{
     cout << "x_Saving data for frame: " << stream_.frame_number() << endl;
}

void VideoWindow::display_image(const cv::Mat &img)
{
     q_image = SylloQt::Mat2QImage(img);
     
     QPixmap pix = QPixmap::fromImage(q_image);

     ui.image_frame->setPixmap(pix);
     ui.image_frame->resize(ui.image_frame->pixmap()->size());
     ui.image_frame->updateGeometry();
     //ui.image_frame->adjustSize();     
}

void VideoWindow::open_camera(int id)
{
     if (stream_.isOpened()) {
          stream_.release();
     }          

     syllo::Status status = stream_.open(id);
     if (status == syllo::Success) {
          
          // set slider range
          ui.frame_slider->setRange(0,1);
          ui.frame_num_spinbox->setRange(0,1);
          ui.filename_label->setText("Camera");
          
          fps_ = stream_.get_fps();
          if (fps_ <= 0) {
               fps_ = 23;
          }

          this->set_fps(fps_);
          
          stream_.set_frame_number(0);
          this->on_open();
          this->get_video_frame();                    
          this->pause();
          
          this->updateGeometry();
          this->adjustSize();               

          this->play();
     } else {
          cout << "Failed to open camera" << endl;
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
     
     filename_ = QFileDialog::getOpenFileName(this, tr("Open File"), dir);
     this->open(filename_);     
}


// Override
void VideoWindow::on_open()
{
}

void VideoWindow::open(QString filename)
{
     filename_ = filename;
     if (!filename_.isEmpty()) {
          // Save the previously opened directory
          prev_open_path_ = QFileInfo(filename_).path();         
          
          std::string fn = filename_.toStdString();
          
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
               ui.filename_label->setText(QFileInfo(filename_).fileName());
               
               fps_ = stream_.get_fps();
               if (fps_ <= 0) {
                    fps_ = 23;
               }
               this->set_fps(fps_);
               
               stream_.set_frame_number(0);
               
               this->on_open();
               this->get_video_frame();                    
               this->pause();               

               //this->resize(ui.verticalLayoutWidget->sizeHint().rwidth()+50, ui.verticalLayoutWidget->sizeHint().rheight());               
               //this->resize(this->GoodSize());               
               this->updateGeometry();
               this->adjustSize();               
          }
     }
}

QSize VideoWindow::GoodSize()
{
     QSize size(ui.verticalLayoutWidget->sizeHint().rwidth()+50, ui.verticalLayoutWidget->sizeHint().rheight());
     cout << "Good: " << size.rwidth() << "," << size.rheight() << endl;
     return size;
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
