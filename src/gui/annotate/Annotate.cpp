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
//#include <opencv_workbench/utils/Object.h>
#include <opencv_workbench/wb/Entity.h>
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
     //point_present_ = false;
     edit_pt_ = false;

     edit_enabled_ = false;
     box_selected_ = false;

     near_thresh = 10;

     object_name_dialog_ = new QDialog(this);
     object_name_ = new ObjectName(object_name_dialog_);

     connect(ui.image_frame, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mousePressed(QMouseEvent*)));
     connect(ui.image_frame, SIGNAL(mouseReleased(QMouseEvent*)), this, SLOT(mouseReleased(QMouseEvent*)));

     add_shortcut("Qt::Key_E", QKeySequence(Qt::Key_E), this, SLOT(erase_box()));
     add_shortcut("Qt::Key_M", QKeySequence(Qt::Key_M), this, SLOT(edit_enabled()));
     add_shortcut("Qt::Key_S", QKeySequence(Qt::Key_S), this, SLOT(save_annotation()));
}

void Annotate::export_roi()
{
     parser_.export_roi();
}

void Annotate::erase_box()
{
     if (edit_enabled_) {
          if (box_selected_) {
               parser_.frames[current_frame_].erase(it_selected_->second.name());
               box_selected_ = false;
          }          
     }
}

void Annotate::edit_enabled()
{
     edit_enabled_ = !edit_enabled_;

     if (!edit_enabled_) {
          box_selected_ = false;
     }
}

void Annotate::save_annotation()
{
     parser_.write_annotation();
     parser_.print();
}

void Annotate::on_open()
{
     parser_.reset();

     parser_.CheckForFile(filename_.toStdString(), AnnotationParser::hand);
     parser_.set_width(stream_.width());
     parser_.set_height(stream_.height());
     parser_.set_depth(3);
     parser_.set_type("video");
     parser_.set_number_of_frames(stream_.get_frame_count());

     object_name_dialog_->show();
     object_name_->show();
}

void Annotate::mousePressed(QMouseEvent * event)
{
     QPoint p = event->pos();     
     
     if (event->button()==Qt::RightButton){
          //point_present_ = true;
          //pt_ = p;
          //edit_pt_ = true;
     } else {
          // A user can only select a box if in edit mode
          box_selected_ = false;
          if (edit_enabled_) {
               // Determine if a box was selected
               for (std::map<std::string, wb::Entity>::iterator it = objects_->begin();
                    it != objects_->end(); it++) {

                    QPoint p1(it->second.bbox().xmin(), it->second.bbox().ymin());
                    QPoint p2(it->second.bbox().xmax(), it->second.bbox().ymax());

                    if (nearby(p,p1,near_thresh)) {
                         edit_pt1_ = true;
                         box_selected_ = true;
                         it_selected_ = it;                         
                    } else if (nearby(p,p2,near_thresh)) {
                         edit_pt2_ = true;
                         box_selected_ = true;
                         it_selected_ = it;                         
                    } else if (it->second.bbox().contains(cv::Point(p.x(),p.y()))) {
                         edit_box_ = true;
                         pt1_drag_offset_ = p1 - p;
                         pt2_drag_offset_ = p2 - p;
                         box_selected_ = true;
                         it_selected_ = it;                         
                    }
               }

               if (box_selected_) {                    
                    pt1_ = QPoint(it_selected_->second.bbox().xmin(), it_selected_->second.bbox().ymin());
                    pt2_ = QPoint(it_selected_->second.bbox().xmax(), it_selected_->second.bbox().ymax());
                    object_name_->set_selected_name(str2qstr(it_selected_->second.name()));
               } else {
                    // New box?
                    // First time drawing a box
                    
                    // Create a new entity
                    wb::Entity object;
                    object.set_bbox(BoundingBox(std::min(p.x(),p.x()),
                                                std::max(p.x(),p.x()),
                                                std::min(p.y(),p.y()),
                                                std::max(p.y(),p.y())));

                    // Get the name from the ObjectName box
                    std::string name = object_name_->selected_name().toStdString();
                    object.set_name(name);

                    // Add it to the parser for this frame
                    parser_.frames[current_frame_].objects[name] = object;                    
                    //cout << "Current frame: " << current_frame_ << endl;
                    //cout << "Added object id: " << parser_.frames[current_frame_].objects[name].id() << endl;
                    //cout << "Added object type: " << parser_.frames[current_frame_].objects[name].type_str() << endl;

                    // Find the iterator for this new object
                    for (std::map<std::string, wb::Entity>::iterator it = objects_->begin();
                         it != objects_->end(); it++) {
                         if (it->second.name() == name) {
                              box_selected_ = true;
                              it_selected_ = it;
                              break;
                         }
                    }

                    if (box_selected_) {
                         pt1_ = p;
                         pt2_ = p;
                         edit_pt2_ = true;
                    } else {
                         cout << "Error: couldn't find: " << name << endl;
                    }
               }
          }
     }     
}

void Annotate::on_mouseMoved(QMouseEvent * event)
{
     QPoint p = event->pos();
     curr_pos_ = p;
          
     if (box_selected_ && edit_enabled_) {          
          if (edit_pt1_) {               
               pt1_ = p;
          } else if (edit_pt2_) {               
               pt2_ = p;                        
          } else if (edit_box_) {
               pt1_ = p + pt1_drag_offset_;
               pt2_ = p + pt2_drag_offset_;
          }

          if (edit_pt_) {
               pt_ = p;
          }

          // Update the object's bounding box
          BoundingBox b(std::min(pt1_.x(),pt2_.x()),
                        std::max(pt1_.x(),pt2_.x()),
                        std::min(pt1_.y(),pt2_.y()),
                        std::max(pt1_.y(),pt2_.y()));
          
          it_selected_->second.set_bbox(b);
     }     
}

void Annotate::mouseReleased(QMouseEvent * event)
{
     // Reset all state variables
     edit_pt1_ = false;
     edit_pt2_ = false;
     edit_box_ = false;
     edit_pt_ = false;
}

QString Annotate::str2qstr(std::string str)
{
     return QString::fromUtf8(str.c_str());
}

void Annotate::before_next_frame()
{     
     // Get the next objects
     int next_frame_number = stream_.next_frame_number();
     if (parser_.frames.count(next_frame_number) >= 0) {
          objects_ = &(parser_.frames[next_frame_number].objects);

          // Make sure all object names in this frame are displayed
          for (std::map<std::string, wb::Entity>::iterator it = objects_->begin();
               it != objects_->end(); it++) {
               object_name_->add_name(str2qstr(it->second.name()));
          }          

     } else {
          // The frame doesn't exist yet, create it.
          Frame frame;
          frame.set_frame_number(next_frame_number);
          parser_.frames[next_frame_number] = frame;
     }

     if (edit_enabled_) {
          // If we are editing frames, we need to copy over the rectangles from 
          // the previous frames, if those objects dont exist yet in the next frame
          for (std::map<std::string, wb::Entity>::iterator it1 = parser_.frames[current_frame_].objects.begin();
               it1 != parser_.frames[current_frame_].objects.end(); it1++) {
               
               bool match = false;
               for (std::map<std::string, wb::Entity>::iterator it2 = objects_->begin();
                    it2 != objects_->end(); it2++) {
                    if (it1->second.name() == it2->second.name()) {
                         match = true;
                    }
               }
               if (!match) {
                    // There wasn't a match, copy over the entity from the previous frame
                    (*objects_)[it1->second.name()] = it1->second;
               }
          }
               
     }
     
     current_frame_ = next_frame_number;

     //// Does the next frame have annotation data?
     //int next_frame_number = stream_.next_frame_number();
     //// Is this frame annotated already?
     //
     //if (!edit_enabled_) {
     //     box_present_ = false;
     //     point_present_ = false;
     //}
     //
     //if (parser_.frames.count(next_frame_number) >= 0) {
     //     // Find all objects in the next frame and get the box:
     //     for (std::map<std::string, wb::Entity>::iterator it = parser_.frames[next_frame_number].objects.begin();
     //          it != parser_.frames[next_frame_number].objects.end(); it++) {
     //
     //          QString qstr = QString::fromUtf8(it->second.name().c_str());
     //          object_name_->set_selected_name(qstr);
     //
     //          box_present_ = true;
     //          cv::Rect rect = it->second.bbox().rectangle();
     //          pt1_ = QPoint(rect.tl().x, rect.tl().y);
     //          pt2_ = QPoint(rect.br().x, rect.br().y);
     //     }
     //}
}

void Annotate::before_display(cv::Mat &img)
{
     // Draw the cross hairs
     cv::line(img, cv::Point(0,curr_pos_.y()), cv::Point(img.cols-1,curr_pos_.y()), cv::Scalar(0,0,255), 1, 8, 0);
     cv::line(img, cv::Point(curr_pos_.x(),0), cv::Point(curr_pos_.x(),img.rows-1), cv::Scalar(0,0,255), 1, 8, 0);

     if (edit_enabled_) {
          cv::rectangle(img,cv::Point(0,0),cv::Point(img.cols-1,img.rows-1),
                        cv::Scalar(0,0,255),2,8,0);
     }

     // Find all objects in the next frame and get the box:
     for (std::map<std::string, wb::Entity>::iterator it = objects_->begin();
          it != objects_->end(); it++) {          
          cv::Rect rect = it->second.bbox().rectangle();         
          
          if (box_selected_ && it_selected_ == it) {
               // Selected rect is red
               cv::rectangle(img,rect,cv::Scalar(0,0,255),1,8,0);
          } else {
               cv::rectangle(img,rect,cv::Scalar(0,255,0),1,8,0);
          }
          //img.at<cv::Vec3b>(rect.tl().y, rect.tl().x) = cv::Vec3b(0,255,0);
          //img.at<cv::Vec3b>(rect.br().y, rect.br().x) = cv::Vec3b(0,255,0);
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
