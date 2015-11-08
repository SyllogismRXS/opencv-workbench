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

#ifndef VIDEOWINDOW_H
#define VIDEOWINDOW_H

#include <map>

// OpenCV headers
#include <cv.h>
//#include <highgui.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <video_window/ui_VideoWindow.h>

#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/gui/cut/cut.h>

#include <QtGui>
#include <QResource>
#include <QSettings>
#include <QTextStream>
#include <QShortcut>

class VideoWindow : public QWidget
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
     }State_t;

     VideoWindow(QWidget *parent = 0);
     
     void add_shortcut(std::string name, const QKeySequence & key, QWidget * parent, const char * member);     

     //QSize sizeHint() const;
     
     QSize GoodSize();
     
     void open(QString fileName);     
     void open_camera(int id);
     
     private slots:
     void tooltip_enabled();
     void mouseMoved(QPoint p);
     void about();
     void open();     
     void timer_video_loop();
     void timer_refresh_loop();
     
     void space_bar();

     void draw();
     void get_video_frame();
     virtual void before_display(cv::Mat &img);
     virtual void before_next_frame();

     void play();
     void pause();

     void display_image(const cv::Mat &img);
     
     void step_one_frame();
     void back_one_frame();
     
     void double_frame_rate();
     void divide_frame_rate();   
     
     void set_fps(double fps);
     void set_frame_num_from_slider(int frame_num);
     void set_frame_num_from_spinbox();
     
     void slider_released();       

     void draw_tooltip(cv::Mat &img);     

protected:
     Ui::VideoWindow ui;    
     
     QTimer *timer_video_;
     QTimer *timer_refresh_;

     QImage q_image;
     
     QString filename_;

     syllo::Stream stream_;
     
     void readSettings();
     void writeSettings();

     QString m_sSettingsFile;
     QString prev_open_path_;     
     
     QDialog * cut_dialog_;
     CutForm *cut_;

     State_t state_;
     double fps_;       
     double timer_refresh_fps_;
     
     void closeEvent(QCloseEvent *event);
     //virtual void resizeEvent(QResizeEvent *);
     
     cv::Mat curr_image_;
     cv::Mat visible_img_;
     
     bool tooltip_enabled_;   
     QPoint mouse_pos_;

     virtual void on_open();
     virtual void on_mouseMoved(QPoint p);     
          
private:
     std::map<std::string, QShortcut*> shortcuts_;
};

#endif
