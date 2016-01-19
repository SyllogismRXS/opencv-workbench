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

#ifndef IMAGEVIEWFORM_H
#define IMAGEVIEWFORM_H

// OpenCV headers
#include <cv.h>
//#include <highgui.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ui_imageviewform.h"

#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/Chain.h>
#include <opencv_workbench/gui/cut/cut.h>
#include <opencv_workbench/gui/scuba_face_label/scuba_face_label.h>
#include <opencv_workbench/gui/video_object_label/video_object_label.h>

#include <QTextStream>

class ImageViewForm : public QMainWindow
{
     Q_OBJECT

     public:

     typedef enum{
          img = 0,
          avi,
          mp4,
          wmv,
          usb_cam,
          sonar
     }Media_t;

     typedef enum{
          none = 0,
          paused,
          playing,
          recording,
          not_recording
     }State_t;

     ImageViewForm(QMainWindow *parent = 0);

     private slots:
     void about();
     void open();
     void open_media(QString fileName);
     void load_config();
     void open_camera();
     void save();
     void timer_loop();
     void mousePressed(QMouseEvent * event);
     void mouseReleased(QMouseEvent * event);
     void mouseMoved(QMouseEvent * event);

     void space_bar();

     void draw();
     void play();
     void pause();
     void record();

     void draw_image(const cv::Mat &img);
     void draw_image_2(const cv::Mat &img);

     void double_frame_rate();
     void divide_frame_rate();

     void set_fps(double fps);
     void set_frame_num(int frame_num);
     void set_frame_num_from_slider(int frame_num);

     void set_cam_id(int id);

     void cut();
     void export_video_frames();
     void start_scuba_face_labeling();
     void scuba_face_label();

     void video_object_label();
     void start_video_object_labeling();

     QImage Mat2QImage(cv::Mat const& src);
     cv::Mat QImage2Mat(QImage const& src);

private:
     Ui::ImageViewForm ui;

     QTimer *timer_;

     QImage q_image;
     //cv::Mat cv_image;     

     syllo::Stream stream_;
     syllo::Stream stream_2_;

     void readSettings();
     void writeSettings();

     QString m_sSettingsFile;
     QString prev_open_path_;     
     QString prev_load_config_path_;
     
     QString prev_config_file_;

     
     bool synced_sonar_;

protected:

     QDialog * cut_dialog_;
     CutForm *cut_;

     QDialog * scuba_face_label_dialog_;
     ScubaFaceLabel *scuba_face_label_;

     QDialog * video_object_label_dialog_;
     VideoObjectLabel *video_object_label_;

     State_t state_;
     State_t record_state_;
     double fps_;
     
     syllo::Chain chain_;     
     
     void closeEvent(QCloseEvent *event);

     bool label_in_prog_;
     QString input_dir_;
     QString output_dir_;
     QStringList files_;
     int cur_file_;

     cv::Mat curr_image_;
     cv::Mat visible_img_;

     cv::Mat curr_image_2_;

     bool mouse_dragging_;
     QPoint first_click_;
     QPoint second_click_;
     QPoint mouse_loc_;

     bool moving_second_pt_;
     bool moving_box_;

     bool box_mode_;

     QString video_label_output_dir_;
     bool label_mode_;
     QString model_name_;
     QString video_label_fn_;
     QFile *video_label_file_;
     QTextStream *out_;
};

#endif
