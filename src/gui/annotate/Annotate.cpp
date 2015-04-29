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
     
     edit_pt1_ = false;
     edit_pt2_ = false; 
     edit_box_ = false;
     box_drawn_ = false;
     
     near_thresh = 25;
     
     connect(ui.image_frame, SIGNAL(mousePressed(QPoint)), this, SLOT(mousePressed(QPoint)));
     connect(ui.image_frame, SIGNAL(mouseReleased(QPoint)), this, SLOT(mouseReleased(QPoint)));
}

void Annotate::on_open()
{
     parser_.CheckForFile(filename_.toStdString());     
}

void Annotate::mousePressed(QPoint p)
{    
     // If the box has already been drawn, check to see if the user is trying
     // to edit the corners of the box
     if (box_drawn_) {
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
          }
     } else {
          // First time drawing a box
          pt1_ = p;
          pt2_ = p;
     }

     connect(ui.image_frame, SIGNAL(mouseMoved(QPoint)), this, SLOT(mouseMoved(QPoint)));     
}

void Annotate::mouseMoved(QPoint p)
{
     if (edit_pt1_) {
          pt1_ = p;
     } else if (edit_pt2_) {               
          pt2_ = p;
     } else if (edit_box_) {
          pt1_ = p + pt1_drag_offset_;
          pt2_ = p + pt2_drag_offset_;
     } else {
          pt2_ = p;
     }
}

void Annotate::mouseReleased(QPoint p)
{
     // Reset all state variables
     edit_pt1_ = false;
     edit_pt2_ = false;
     edit_box_ = false;

     // A box has been drawn
     box_drawn_ = true;          
     
     // Disconnect mouse moving callback
     disconnect(ui.image_frame, SIGNAL(mouseMoved(QPoint)), this, SLOT(mouseMoved(QPoint)));

     //cout << "Frame num: " << stream_.get_frame_number()-1 << endl;
}

void Annotate::before_next_frame()
{
     // Save any data from this frame
     //cout << "Saving data for frame: " << stream_.get_frame_number()-1 << endl;
}

void Annotate::before_display(cv::Mat &img)
{
     cv::Point pt1(pt1_.x(), pt1_.y());
     cv::Point pt2(pt2_.x(), pt2_.y());
     cv::rectangle(img,pt1,pt2,cv::Scalar(0,0,255),1,8,0);
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
