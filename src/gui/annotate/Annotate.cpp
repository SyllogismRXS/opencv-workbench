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

#include "Annotate.h"

using std::cout;
using std::endl;

Annotate::Annotate(VideoWindow *parent)
     : VideoWindow(parent)
{
     //ui.setupUi(this);
     connect(ui.image_frame, SIGNAL(mousePressed(QPoint)), this, SLOT(mousePressed(QPoint)));
     connect(ui.image_frame, SIGNAL(mouseReleased(QPoint)), this, SLOT(mouseReleased(QPoint)));
}

void Annotate::mousePressed(QPoint p)
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

void Annotate::mouseMoved(QPoint p)
{
     mouse_loc_ = p;
}

void Annotate::mouseReleased(QPoint p)
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

     //if (label_mode_) {
     //     *out_ << QString::number(ui.frame_num_spinbox->value()) << ","
     //           << QString::number(p.x()) << ","
     //           << QString::number(p.y())
     //           << endl;
     //     
     //     //// move to the next frame
     //     //if (stream_.read(curr_image_)) {
     //     //     this->draw();
     //     //} else {
     //     //     cout << "Done Labelling" << endl;
     //     //     //video_label_file_->close();
     //     //}     
     //}
}

void Annotate::before_display(cv::Mat &img)
{
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
          cv::rectangle(img,pt1,pt2,cv::Scalar(0,0,255),1,8,0);          
          
     } else {
          cv::rectangle(img, cv::Point(first_click_.x(), first_click_.y()),
                        cv::Point(second_click_.x(),second_click_.y()),
                        cv::Scalar(0,0,255),1,8,0);          
     }
}
