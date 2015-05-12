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
#include <iostream>
#include <algorithm>

#include <QtGui>
#include <QResource>
#include <QSettings>

#include <opencv_workbench/utils/SylloQt.h>
#include <opencv_workbench/utils/Frame.h>
#include <opencv_workbench/utils/Object.h>
#include <opencv_workbench/utils/BoundingBox.h>
//#include <opencv_workbench/utils/Rectangle.h>

#include "Annotate.h"

using std::cout;
using std::endl;

Annotate::Annotate(VideoWindow *parent)
     : VideoWindow(parent)
{
     //ui.setupUi(this);
     
     edit_pt1_ = false;
     edit_pt2_ = false; 
     edit_box_ = false;
     box_present_ = false;

     edit_enabled_ = false;

     near_thresh = 25;     
     
     connect(ui.image_frame, SIGNAL(mousePressed(QPoint)), this, SLOT(mousePressed(QPoint)));
     connect(ui.image_frame, SIGNAL(mouseReleased(QPoint)), this, SLOT(mouseReleased(QPoint)));       
     
     add_shortcut("Qt::Key_E", QKeySequence(Qt::Key_E), this, SLOT(erase_box()));
     add_shortcut("Qt::Key_M", QKeySequence(Qt::Key_M), this, SLOT(edit_enabled()));
     add_shortcut("Qt::Key_S", QKeySequence(Qt::Key_S), this, SLOT(save_annotation()));
     
     //new QShortcut(QKeySequence(Qt::Key_M), this, SLOT(edit_enabled()));
     //new QShortcut(QKeySequence(Qt::Key_S), this, SLOT(save_annotation()));     
}

void Annotate::export_roi()
{
     parser_.export_roi();
}

void Annotate::erase_box()
{
     if (edit_enabled_) {
          if (box_present_) {
               int frame_number = stream_.frame_number();
               parser_.frames.erase(frame_number);
          }     
          box_present_ = false;
     }
}

void Annotate::edit_enabled()
{
     edit_enabled_ = !edit_enabled_;
}

void Annotate::save_annotation()
{
     this->save_annotation_data();
     parser_.write_annotation();
     //parser_.print();
}

void Annotate::on_open()
{
     parser_.reset();

     parser_.CheckForFile(filename_.toStdString());
     parser_.set_width(stream_.width());
     parser_.set_height(stream_.height());
     parser_.set_depth(3);
     parser_.set_type("video");
     parser_.set_number_of_frames(stream_.get_frame_count());     
}

void Annotate::mousePressed(QPoint p)
{    
     // If the box has already been drawn, check to see if the user is trying
     // to edit the corners of the box
     if (box_present_) {
          // Is the user trying to edit point 1, point 2, or is the user
          // trying to move the entire box?
          if (nearby(p,pt1_,near_thresh)) {
               edit_pt1_ = true;               
          } else if (nearby(p,pt2_,near_thresh)) {
               edit_pt2_ = true;
          } else if (inside(p,pt1_,pt2_)) {
               edit_box_ = true;
               pt1_drag_offset_ = pt1_ - p;
               pt2_drag_offset_ = pt2_ - p;
          } else {
               // User is trying to draw a new box
               pt1_ = p;
               pt2_ = p;
               edit_pt2_ = true;
          }
     } else {
          // First time drawing a box
          pt1_ = p;
          pt2_ = p;
          edit_pt2_ = true;          
     }

     box_present_ = true;
}

void Annotate::on_mouseMoved(QPoint p)
{
     if (edit_pt1_) {
          pt1_ = p;
     } else if (edit_pt2_) {               
          pt2_ = p;
     } else if (edit_box_) {
          pt1_ = p + pt1_drag_offset_;
          pt2_ = p + pt2_drag_offset_;
     }               
}

void Annotate::mouseReleased(QPoint p)
{
     // Reset all state variables
     edit_pt1_ = false;
     edit_pt2_ = false;
     edit_box_ = false;
}

void Annotate::before_next_frame()
{
     if (edit_enabled_) {     
          this->save_annotation_data();          
     }

     // Does the next frame have annotation data?
     int next_frame_number = stream_.next_frame_number();
     // Is this frame annotated already?
     
     if (parser_.frames.count(next_frame_number) > 0) {
          box_present_ = true;
          cv::Rect rect = parser_.frames[next_frame_number].objects["diver"].bbox.rectangle();
          //pt1_ = QPoint(rect.pt1().x(), rect.pt1().y());
          //pt2_ = QPoint(rect.pt2().x(), rect.pt2().y());
          pt1_ = QPoint(rect.tl().x, rect.tl().y);
          pt2_ = QPoint(rect.br().x, rect.br().y);
     } else if (!edit_enabled_) {
          box_present_ = false;         
     }
}

void Annotate::save_annotation_data()
{
     // Save any data from this frame
     int frame_number = stream_.frame_number();
          
     // Ensure that we have a valid frame number and that a box is present
     if (frame_number < 0) {
          return;
     }
     
     if(!box_present_) {
          return;
     }

     Frame frame;
     frame.set_frame_number(frame_number);
     Object object;
     object.set_name("diver");
     object.bbox = BoundingBox(std::min(pt1_.x(),pt2_.x()), 
                               std::max(pt1_.x(),pt2_.x()),
                               std::min(pt1_.y(),pt2_.y()), 
                               std::max(pt1_.y(),pt2_.y()));

     // Save object to current frame
     frame.objects[object.name()] = object;

     // Save frame to parser
     if (parser_.frames.count(frame_number) > 0) {
          parser_.frames.erase(frame_number);
     }
     
     parser_.frames[frame_number] = frame;
}

void Annotate::before_display(cv::Mat &img)
{     
     if (edit_enabled_) {
          cv::rectangle(img,cv::Point(0,0),cv::Point(img.cols-1,img.rows-1),
                        cv::Scalar(0,0,255),2,8,0);
     }
     
     if (box_present_) {
          cv::Point pt1(pt1_.x(), pt1_.y());
          cv::Point pt2(pt2_.x(), pt2_.y());
                    
          cv::rectangle(img,pt1,pt2,cv::Scalar(0,255,0),1,8,0);
          cv::circle(img, pt1, 1, cv::Scalar(255,255,255), 1, 8, 0);
          cv::circle(img, pt2, 1, cv::Scalar(255,255,255), 1, 8, 0);
     }
}

int Annotate::distance(QPoint p1, QPoint p2)
{
     return sqrt( pow((p1.x()-p2.x()),2) + pow(p1.y()-p2.y(),2) );
}

bool Annotate::nearby(QPoint p1, QPoint p2, int threshold)
{
     if (this->distance(p1,p2) < threshold) {
          return true;
     }
     return false;
}

bool Annotate::inside(QPoint p, QPoint p1, QPoint p2)
{
     // Check all cases where there mouse could be outside the box defined
     // by the rectangle's two points
     if (p.x() > p1.x() && p.x() > p2.x()) { return false; }     
     if (p.y() > p1.y() && p.y() > p2.y()) { return false; }     
     if (p.x() < p1.x() && p.x() < p2.x()) { return false; }     
     if (p.y() < p1.y() && p.y() < p2.y()) { return false; }

     return true;     
}
