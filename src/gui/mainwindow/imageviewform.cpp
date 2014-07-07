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

#include "imageviewform.h"

using std::cout;
using std::endl;

ImageViewForm::ImageViewForm(QMainWindow *parent)
     : QMainWindow(parent)
{
     ui.setupUi(this);

     readSettings();

     state_ = none;
     record_state_ = not_recording;
     
     label_in_prog_ = false;

     mouse_dragging_ = false;

     moving_second_pt_ = false;
     moving_box_ = false;
     
     first_click_ = QPoint(-1,-1);
     second_click_ = QPoint(-1,-1);

     synced_sonar_ = false;

     box_mode_ = false;
     label_mode_ = false;

     if (prev_config_file_ != "") {
          chain_.LoadFile(prev_config_file_.toStdString());
     }
     
     chain_.LoadFile("blank"); //TODO: handle correctly

     connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
     connect(ui.actionLoad_Config, SIGNAL(triggered()), this, SLOT(load_config()));
     connect(ui.actionOpen_Camera, SIGNAL(triggered()), this, SLOT(open_camera()));
     connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save()));
     connect(ui.actionQuit, SIGNAL(triggered()), this, SLOT(close()));
     connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
     
     connect(ui.actionCut, SIGNAL(triggered()), this, SLOT(cut()));
     connect(ui.actionScuba_Face_Label, SIGNAL(triggered()), this, SLOT(scuba_face_label()));
     connect(ui.actionVideo_Object_Label, SIGNAL(triggered()), this, SLOT(video_object_label()));
     connect(ui.actionRecord, SIGNAL(triggered()), this, SLOT(record()));    

     connect(ui.fps_spinbox, SIGNAL(valueChanged(double)), this, SLOT(set_fps(double)));
     //connect(ui.frame_num_spinbox, SIGNAL(valueChanged(int)), this, SLOT(set_frame_num(int)));
     connect(ui.frame_slider, SIGNAL(sliderMoved(int)), this, SLOT(set_frame_num_from_slider(int)));

     connect(ui.play_button, SIGNAL(released()), this, SLOT(space_bar()));   
     connect(ui.rewind_button, SIGNAL(released()), this, SLOT(divide_frame_rate()));
     connect(ui.forward_button, SIGNAL(released()), this, SLOT(double_frame_rate()));
     
     connect(ui.cam_id_spinbox, SIGNAL(valueChanged(int)), this, SLOT(set_cam_id(int)));

     connect(ui.image_frame, SIGNAL(mousePressed(QPoint)), this, SLOT(mousePressed(QPoint)));
     connect(ui.image_frame, SIGNAL(mouseReleased(QPoint)), this, SLOT(mouseReleased(QPoint)));
     
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
}

void ImageViewForm::mousePressed(QPoint p)
{
     if (!mouse_dragging_) {
          // check to see if the user wants to adjust the rectangle
          if (first_click_.x() != -1) {               
               if (abs(p.x()-first_click_.x()) < 20 && 
                   // user clicks near first click
                   abs(p.y()-first_click_.y()) < 20) {
                    moving_second_pt_ = false;                    
               } else if (abs(p.x()-second_click_.x()) < 20 && 
                          abs(p.y()-second_click_.y()) < 20) {
                    // user clicks near second click
                    moving_second_pt_ = true;
               //} else if (p.x() > first_click_.x() && p.x() < second_click_.x() && p.y() > first_click_.y() && p.y() < second_click_.y()) {
               //     // user clicked inside of box
               //     moving_box_ = true;
               //     moving_second_pt_ = false;
               } else {
                    // user doesn't click near either
                    first_click_ = p;
                    moving_second_pt_ = true;
               }
          } else {
               first_click_ = p;
               moving_second_pt_ = true;
          }          
          
          mouse_dragging_ = true;
          connect(ui.image_frame, SIGNAL(mouseMoved(QPoint)), this, SLOT(mouseMoved(QPoint)));
     }           
}

void ImageViewForm::mouseMoved(QPoint p)
{
     mouse_loc_ = p;
     //cout << "X: " << p.x() << endl;
     //cout << "Y: " << p.y() << endl;
}

void ImageViewForm::mouseReleased(QPoint p)
{
     if (mouse_dragging_) {
          if (moving_second_pt_) {
               second_click_ = p;
          } else if (moving_box_) {
               
          } else {
               first_click_ = p;
          }          
          mouse_dragging_ = false;
          disconnect(ui.image_frame, SIGNAL(mouseMoved(QPoint)), this, SLOT(mouseMoved(QPoint)));
     }
     //cout << "X: " << p.x() << endl;
     //cout << "Y: " << p.y() << endl;

     if (label_mode_) {
          *out_ << QString::number(ui.frame_num_spinbox->value()) << ","
                << QString::number(p.x()) << ","
                << QString::number(p.y())
                << endl;
          
          // move to the next frame
          if (stream_.read(curr_image_)) {
               this->draw();
          } else {
               cout << "Done Labelling" << endl;
               video_label_file_->close();
          }     
     }
}

void ImageViewForm::set_cam_id(int id)
{
     if (stream_.isLive()) {
          this->open_camera();
     }
}

void ImageViewForm::scuba_face_label()
{
     scuba_face_label_dialog_ = new QDialog(this);
     scuba_face_label_ = new ScubaFaceLabel(scuba_face_label_dialog_);
     
     scuba_face_label_->set_input_dir(input_dir_);
     scuba_face_label_->set_output_dir(output_dir_);

     connect(scuba_face_label_, SIGNAL(start_labeling()), this, SLOT(start_scuba_face_labeling()));
     
     scuba_face_label_dialog_->show();
     scuba_face_label_->show();
}

void ImageViewForm::start_scuba_face_labeling()
{
     input_dir_ = scuba_face_label_->input_dir();
     output_dir_ = scuba_face_label_->output_dir();

     //cout << "input:" << input_dir_.toUtf8().constData() << endl;

     box_mode_ = true;

     delete scuba_face_label_;
     delete scuba_face_label_dialog_;

     label_in_prog_ = true;
     files_ = QDir(input_dir_).entryList(QStringList("*"), 
                                         QDir::Files | QDir::NoSymLinks);
     cur_file_ = 0;

     //for (int i = 0; i < files_.size(); ++i) {
     //     cout << files_.at(i).toLocal8Bit().constData() << endl;
     //}
     cout << "opening file: " << files_.at(cur_file_).toStdString() << endl;
     open_media(input_dir_ + "/" + files_.at(cur_file_));
     cur_file_++;
}

void ImageViewForm::video_object_label()
{
     video_object_label_dialog_ = new QDialog(this);
     video_object_label_ = new VideoObjectLabel(video_object_label_dialog_);
     
     //video_object_label_->set_output_dir(output_dir_);     

     connect(video_object_label_, SIGNAL(start_labeling()), this, SLOT(start_video_object_labeling()));
     
     video_object_label_dialog_->show();
     video_object_label_->show();
}

void ImageViewForm::start_video_object_labeling()
{
     video_label_output_dir_ = video_object_label_->output_dir();
     model_name_ = video_object_label_->model_name();

     if (model_name_ != "") {
          label_mode_ = true;
     }

     video_label_fn_ = video_label_output_dir_ + "/" + 
          ui.filename_label->text() + "." + model_name_ + ".label";

     cout << "Output file: " << video_label_fn_.toStdString() << endl;

     video_label_file_ = new QFile(video_label_fn_);
     video_label_file_->open(QIODevice::WriteOnly | QIODevice::Text);
     out_ = new QTextStream(video_label_file_);
     
     delete video_object_label_;
     delete video_object_label_dialog_;

     //files_ = QDir(input_dir_).entryList(QStringList("*"), 
     //                                    QDir::Files | QDir::NoSymLinks);
     //cur_file_ = 0;

     //for (int i = 0; i < files_.size(); ++i) {
     //     cout << files_.at(i).toLocal8Bit().constData() << endl;
     //}
     //cout << "opening file: " << files_.at(cur_file_).toStdString() << endl;
     //open_media(input_dir_ + "/" + files_.at(cur_file_));
     //cur_file_++;
}

void ImageViewForm::cut()
{
     cut_dialog_ = new QDialog(this);     
     cut_ = new CutForm(cut_dialog_);

     // Inform the Cut dialog of the max and min frame numbers
     cut_->set_min_max(0,stream_.get_frame_count());

     connect(cut_, SIGNAL(export_video()), this, SLOT(export_video_frames()));
     
     cut_dialog_->show();
     cut_->show();
}

void ImageViewForm::record()
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

void ImageViewForm::export_video_frames()
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

void ImageViewForm::set_frame_num_from_slider(int frame_num)
{
     this->set_frame_num(frame_num);     
}

void ImageViewForm::set_frame_num(int frame_num) 
{
     if (state_ == paused) {
          stream_.set_frame_number(frame_num);
          stream_2_.set_frame_number(frame_num);
          //this->draw();
     }
}

void ImageViewForm::play()
{
     timer_->start();
     state_ = playing;
     ui.play_button->setIcon(QIcon(":/resources/pause.png"));
}

void ImageViewForm::pause()
{
     timer_->stop();
     state_ = paused;
     ui.play_button->setIcon(QIcon(":/resources/play.png"));
}

void ImageViewForm::set_fps(double fps)
{
     fps_ = fps;
     ui.fps_spinbox->setValue(fps_);
     timer_->setInterval(1000.0/fps_);
}

void ImageViewForm::double_frame_rate()
{
     this->set_fps(fps_*2.0);
}

void ImageViewForm::divide_frame_rate()
{
     this->set_fps(fps_/2.0);
}


void ImageViewForm::closeEvent(QCloseEvent *event)
{
     this->writeSettings();
}

void ImageViewForm::space_bar()
{
     if (label_in_prog_) {
          // save file to output:          
          QString output_fn = output_dir_ + "/" + ui.filename_label->text();
          int x = first_click_.x();
          int y = first_click_.y();
          int width = second_click_.x() - first_click_.x();
          int height = second_click_.y() - first_click_.y();

          cout << "x: " << x << endl;
          cout << "y: " << y << endl;
          cout << "width: " << width << endl;
          cout << "height: " << height << endl;

          cv::Rect myRect(x, y, width, height);          
          chain_.crop_image(visible_img_, visible_img_, myRect);
          chain_.gray_and_equalize(visible_img_, visible_img_);
          chain_.save_image(output_fn.toStdString(), visible_img_);

          if (cur_file_ < files_.size()) {
               cout << "opening file: " << files_.at(cur_file_).toStdString() << endl;
               open_media(input_dir_ + "/" + files_.at(cur_file_));
               cur_file_++;
          } else {
               // all done
               cur_file_++;
               label_in_prog_ = false;
          }                    
     } else {
          if (state_ == playing) {
               this->pause();          
          } else if (state_ == paused) {          
               this->set_fps(fps_);
               this->play();          
          }
     }
}

void ImageViewForm::timer_loop()
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

void ImageViewForm::draw()
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
          
     if (ui.enable_chain_checkbox->isChecked()) {
          //chain_.process(curr_image_, visible_img_);          
     } else {
          visible_img_ = curr_image_.clone();
     }
          
     if (box_mode_) {
          if (mouse_dragging_) {
               cv::Point pt1;
               cv::Point pt2;
               if (moving_second_pt_) {
                    pt1 = cv::Point(first_click_.x(), first_click_.y());
                    pt2 = cv::Point(mouse_loc_.x(), mouse_loc_.y());
               } else {
                    pt1 = cv::Point(mouse_loc_.x(), mouse_loc_.y());
                    pt2 = cv::Point(second_click_.x(), second_click_.y());
               }
               chain_.draw_rectangle(visible_img_, visible_img_, pt1, pt2);
          
          } else {
               chain_.draw_rectangle(visible_img_, visible_img_,cv::Point(first_click_.x(), first_click_.y()), 
                                     cv::Point(second_click_.x(),second_click_.y()));
          }
     }

     this->draw_image(visible_img_);     

     if (synced_sonar_) {
          stream_2_.read(curr_image_2_);
          this->draw_image_2(curr_image_2_);     
     }

     if (record_state_ == recording) {
          stream_.step_camera_record();
     }
}

void ImageViewForm::draw_image(const cv::Mat &img)
{
     q_image = Mat2QImage(img);
     ui.image_frame->setPixmap(QPixmap::fromImage(q_image));
     ui.image_frame->adjustSize();
}

void ImageViewForm::draw_image_2(const cv::Mat &img)
{
     q_image = Mat2QImage(img);
     
     ui.image_frame_2->setPixmap(QPixmap::fromImage(q_image));
     ui.image_frame_2->adjustSize();
}

void ImageViewForm::open_camera()
{
     if (stream_.isOpened()) {
          stream_.release();
     }          

     syllo::Status status = stream_.open(ui.cam_id_spinbox->value());
     if (status == syllo::Success && stream_.isOpened()) {
               
          this->set_frame_num(0);                                   
          stream_.read(curr_image_);          
          this->draw_image(curr_image_);               

          ui.filename_label->setText("Camera");

          this->set_fps(29);
          this->play();
     }
}

void ImageViewForm::load_config()
{
     // If the prev_open_path variable is set, use it, otherwise, use
     // the current directory.
     QString dir = QDir::currentPath();
     if (prev_load_config_path_ != "") {
          dir = prev_load_config_path_;
     }

     QString fileName = QFileDialog::getOpenFileName(this, tr("Open Config File"), dir);

     if (!fileName.isEmpty()) {
          // Save the previously open directory
          prev_load_config_path_ = QFileInfo(fileName).path();         

          prev_config_file_ = fileName;

          std::string fn = fileName.toStdString();          
          chain_.LoadFile(fn);
     }
}

void ImageViewForm::open()
{    
     this->pause();

     // If the prev_open_path variable is set, use it, otherwise, use
     // the current directory.
     QString dir = QDir::currentPath();
     if (prev_open_path_ != "") {
          dir = prev_open_path_;
     }
     
     QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), dir);
     open_media(fileName);
}

void ImageViewForm::open_media(QString fileName)
{
     if (!fileName.isEmpty()) {
          // Save the previously open directory
          prev_open_path_ = QFileInfo(fileName).path();         

          std::string fn = fileName.toStdString();
          
          syllo::Status status = stream_.open(fn);          

          synced_sonar_ = false;
          std::string ext = fn.substr(fn.find_last_of(".") + 1);
          std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

          // If the file is a sonar file and there is an associated avi file
          // We need to be in dual display (sonar/video) mode
          if (ext == "SON") {
               std::string query_file = fn.substr(0,fn.find_last_of(".")) + std::string(".avi");
               cout << "Looking for video file: " << query_file << endl;              
               syllo::Status status2 = stream_2_.open(query_file);
               if (status2 == syllo::Success && stream_2_.isOpened()) {
                    //synced_sonar_ = true;
               }
          }

          if (status == syllo::Success && stream_.isOpened()) {
               
               this->set_frame_num(0);
               
               // set slider range
               ui.frame_slider->setRange(0,stream_.get_frame_count()-1);
               ui.frame_num_spinbox->setRange(0,stream_.get_frame_count()-1);
               ui.total_frames_spinbox->setValue(stream_.get_frame_count());
               ui.filename_label->setText(QFileInfo(fileName).fileName());
               
               fps_ = stream_.get_fps();
               this->set_fps(fps_);
               
               if (stream_.read(curr_image_)) {

                    stream_2_.read(curr_image_2_);

                    // one time processing:
                    //chain_.process(curr_image_, curr_image_);

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

void ImageViewForm::save()
{
     cout << "Save!" << endl;
}

void ImageViewForm::about()
{
     QMessageBox::about(this, tr("About The OpenCV Workbench"),
                        tr("<p><b>The OpenCV Workbench</b> is a GUI application that simplifies the computer vision prototyping process.</p><p>Author: Kevin DeMarco (kevin.demarco@gmail.com)</p>"));
}

//QImage *img = new QImage(frame.data, frame.cols, frame.rows, QImage::Format_RGB888);
QImage ImageViewForm::Mat2QImage(cv::Mat const& src)
{
     cv::Mat temp; // make the same cv::Mat
     cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
     QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
     QImage dest2(dest);
     dest2.detach(); // enforce deep copy
     return dest2;
}

cv::Mat ImageViewForm::QImage2Mat(QImage const& src)
{
     cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
     cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
     cvtColor(tmp, result,CV_BGR2RGB);
     return result;
}

void ImageViewForm::writeSettings()
{
     QSettings settings;

     // Window size and position
     settings.beginGroup("MainWindow");
     settings.setValue("size", size());
     settings.setValue("pos", pos());

     // Previously opened directory
     settings.setValue("prev_open_path", prev_open_path_);
     settings.setValue("prev_load_config_path", prev_load_config_path_);
     settings.setValue("prev_config_file", prev_config_file_);
     
     settings.setValue("scuba_face_input_dir", input_dir_);
     settings.setValue("scuba_face_output_dir", output_dir_);

     settings.endGroup();
}

void ImageViewForm::readSettings()
{
     QSettings settings;
     
     settings.beginGroup("MainWindow");
     
     // Window size
     resize(settings.value("size", QSize(400, 400)).toSize());
     move(settings.value("pos", QPoint(200, 200)).toPoint());

     // Previously opened directories
     prev_open_path_ = settings.value("prev_open_path").toString();
     prev_load_config_path_ = settings.value("prev_load_config_path").toString();
     prev_config_file_ = settings.value("prev_config_file", "").toString();
     
     input_dir_ = settings.value("scuba_face_input_dir").toString();
     output_dir_ = settings.value("scuba_face_output_dir").toString();
     
     settings.endGroup();
}
