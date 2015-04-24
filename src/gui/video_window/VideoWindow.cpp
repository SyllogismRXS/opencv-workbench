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

#include "VideoWindow.h"

using std::cout;
using std::endl;

VideoWindow::VideoWindow(QWidget *parent)
     : QWidget(parent)
{
     ui.setupUi(this);

     readSettings();

     state_ = none;
     record_state_ = not_recording;
     
     //connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
     //connect(ui.actionOpen_Camera, SIGNAL(triggered()), this, SLOT(open_camera()));
     //connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save()));
     //connect(ui.actionQuit, SIGNAL(triggered()), this, SLOT(close()));
     //connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
     //
     //connect(ui.actionCut, SIGNAL(triggered()), this, SLOT(cut()));
     //connect(ui.actionRecord, SIGNAL(triggered()), this, SLOT(record()));    

     //connect(ui.fps_spinbox, SIGNAL(valueChanged(double)), this, SLOT(set_fps(double)));
     //connect(ui.frame_num_spinbox, SIGNAL(valueChanged(int)), this, SLOT(set_frame_num(int)));
     //connect(ui.frame_slider, SIGNAL(sliderMoved(int)), this, SLOT(set_frame_num_from_slider(int)));

     connect(ui.play_button, SIGNAL(released()), this, SLOT(space_bar()));   
     //connect(ui.rewind_button, SIGNAL(released()), this, SLOT(divide_frame_rate()));
     //connect(ui.forward_button, SIGNAL(released()), this, SLOT(double_frame_rate()));
     
     //connect(ui.cam_id_spinbox, SIGNAL(valueChanged(int)), this, SLOT(set_cam_id(int)));

     // Keyboard shortcuts
     // Note: Get deleted automatically when program closes.
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(open()));
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), this, SLOT(open_camera()));
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));
     new QShortcut(QKeySequence(Qt::Key_Space), this, SLOT(space_bar()));
     new QShortcut(QKeySequence(Qt::Key_Left), this, SLOT(divide_frame_rate()));
     new QShortcut(QKeySequence(Qt::Key_Right), this, SLOT(double_frame_rate()));
     
     new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_R), this, SLOT(record()));
     
     timer_ = new QTimer(this);

     connect(timer_, SIGNAL(timeout()), this, SLOT(timer_loop()));

     //this->setCentralWidget(ui.centralwidget);
     //this->adjustSize();
}

void VideoWindow::set_cam_id(int id)
{
     if (stream_.isLive()) {
          this->open_camera();
     }
}

void VideoWindow::cut()
{
     cut_dialog_ = new QDialog(this);     
     cut_ = new CutForm(cut_dialog_);

     // Inform the Cut dialog of the max and min frame numbers
     cut_->set_min_max(0,stream_.get_frame_count());

     connect(cut_, SIGNAL(export_video()), this, SLOT(export_video_frames()));
     
     cut_dialog_->show();
     cut_->show();
}

void VideoWindow::record()
{
     std::string temp_fn = QDir::homePath().toStdString() + "/output.avi";
     if (record_state_ == not_recording) {
          // Setup video export
          syllo::Status status;
          status = stream_.setup_export_video(temp_fn,0,1);
          if (status != syllo::Success) {
               QMessageBox::critical(this,tr("Error"), tr("Failed to setup video record."));
          } else {
               record_state_ = recording;
               ui.record_button->setIcon(QIcon(":/resources/record_enable.png"));
          }
     } else {
          // Recording
          // Stop the recording
          record_state_ = not_recording;
          ui.record_button->setIcon(QIcon(":/resources/record.png"));
          stream_.stop_camera_record();

          // Ask the user if he wants to save the file
          QString str_avi = "AVI (*.avi)";
          QFileDialog * save_diag = new QFileDialog(this, tr("Save file"), QDir::homePath(), tr(str_avi.toStdString().c_str()));
          save_diag->setAcceptMode(QFileDialog::AcceptSave);

          if (save_diag->exec()) {
               // Get the selected file
               QString fn = save_diag->selectedFiles()[0];
               QString base_ext = QFileInfo(fn).baseName() + ".avi";
               fn = QFileInfo(fn).absolutePath() + "/" + base_ext;
               cout << "filename: " << fn.toStdString() << endl;
               QFile::rename(QString::fromUtf8(temp_fn.c_str()), fn);
          }              
          delete save_diag;
     }
}

void VideoWindow::export_video_frames()
{
     int start_frame = cut_->get_start_frame();
     int end_frame = cut_->get_end_frame();
     QString file_name = cut_->get_filename();

     delete cut_;
     delete cut_dialog_;
     
     QDialog *diag_progress = new QDialog(this);
     QProgressBar *prog_bar = new QProgressBar(diag_progress);
     
     diag_progress->setWindowTitle("Exporting video...");
     prog_bar->setRange(0, end_frame-start_frame);
     diag_progress->show();
     prog_bar->show();

     // Setup video export
     syllo::Status status;
     status = stream_.setup_export_video(file_name.toStdString(), start_frame, end_frame);
     if (status != syllo::Success) {
          QMessageBox::critical(this,tr("Error"), tr("Failed to output video."));
     }

     int prog = 0;
     while (stream_.step_video_export()) {
          prog_bar->setValue(prog++);          
     }

     delete prog_bar;
     delete diag_progress;

     QMessageBox::information(this,tr("Export Complete!"), tr("Video export complete!"));
}

void VideoWindow::set_frame_num_from_slider(int frame_num)
{
     this->set_frame_num(frame_num);     
}

void VideoWindow::set_frame_num(int frame_num) 
{
     if (state_ == paused) {
          stream_.set_frame_number(frame_num);
          //this->draw();
     }
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
     //ui.fps_spinbox->setValue(fps_);
     timer_->setInterval(1000.0/fps_);
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
     if (stream_.isOpened()) {
          if (stream_.type() != syllo::ImageType) {
               if (stream_.read(curr_image_)) {
                    this->draw();
               } else {
                    cout << "Done" << endl;
                    this->pause();
                    this->set_frame_num(0);
               }
          }          
     }     
}

void VideoWindow::draw()
{
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
     
     if (record_state_ == recording) {
          stream_.step_camera_record();
     }
}

void VideoWindow::draw_image(const cv::Mat &img)
{
     q_image = Mat2QImage(img);
     ui.image_frame->setPixmap(QPixmap::fromImage(q_image));
     ui.image_frame->adjustSize();
}

void VideoWindow::open_camera()
{
     if (stream_.isOpened()) {
          stream_.release();
     }          

     //syllo::Status status = stream_.open(ui.cam_id_spinbox->value());
     syllo::Status status = stream_.open(0);
     if (status == syllo::Success && stream_.isOpened()) {
               
          this->set_frame_num(0);                                   
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
               
               this->set_frame_num(0);
               
               // set slider range
               ui.frame_slider->setRange(0,stream_.get_frame_count()-1);
               ui.frame_num_spinbox->setRange(0,stream_.get_frame_count()-1);
               ui.filename_label->setText(QFileInfo(fileName).fileName());
               
               fps_ = stream_.get_fps();
               if (fps_ <= 0) {
                    fps_ = 23;
               }
               this->set_fps(fps_);
               
               if (stream_.read(curr_image_)) {
                    // one time processing:
                    
                    this->draw();                    

                    if (stream_.type() == syllo::ImageType) {
                         this->play();
                    } else {
                         this->pause();                         
                    }                    
               } else {
                    this->pause();
                    this->set_frame_num(0);
               }
          }
     }
}

void VideoWindow::save()
{
     cout << "Save!" << endl;
}

void VideoWindow::about()
{
     QMessageBox::about(this, tr("About The OpenCV Workbench"),
                        tr("<p><b>The OpenCV Workbench</b> is a GUI application that simplifies the computer vision prototyping process.</p><p>Author: Kevin DeMarco (kevin.demarco@gmail.com)</p>"));
}

QImage VideoWindow::Mat2QImage(cv::Mat const& src)
{
     cv::Mat temp; // make the same cv::Mat
     //cv::cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copy, that what i need
     cv::cvtColor(src, src,CV_BGR2RGB); // cvtColor Makes a copy, that what i need
     //QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
     QImage dest((uchar*) src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
     //QImage dest2(dest);
     //dest2.detach(); // enforce deep copy
     //return dest2;
     dest.detach();
     return dest;
}

cv::Mat VideoWindow::QImage2Mat(QImage const& src)
{
     cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
     cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
     cv::cvtColor(tmp, result,CV_BGR2RGB);
     return result;
}

void VideoWindow::writeSettings()
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

void VideoWindow::readSettings()
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
