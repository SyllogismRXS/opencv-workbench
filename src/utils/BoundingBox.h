#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_
/// ---------------------------------------------------------------------------
/// @file BoundingBox.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-10-09 14:43:35 syllogismrxs>
///
/// @version 1.0
/// Created: 29 Apr 2015
///
/// ---------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ---------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The BoundingBox class ...
/// 
/// ---------------------------------------------------------------------------
#include <math.h>

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <opencv_workbench/utils/Rectangle.h>

class BoundingBox {
public:
BoundingBox() :
     xmin_(0), xmax_(0), ymin_(0), ymax_(0) 
     {
     }

BoundingBox(int xmin, int xmax, int ymin, int ymax) :
     xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax) 
     {
     }     

BoundingBox(cv::Rect rect) :
     xmin_(rect.x), xmax_(rect.x+rect.width), ymin_(rect.y), ymax_(rect.y+rect.height) 
     {
     }  

     void set_box(int xmin, int xmax, int ymin, int ymax)
     {
          xmin_ = xmin;
          xmax_ = xmax;
          ymin_ = ymin;
          ymax_ = ymax;
     }
          
     int xmin() { return xmin_; }
     int xmax() { return xmax_; }
     int ymin() { return ymin_; }
     int ymax() { return ymax_; }     

     int width()
     {
          return xmax_ - xmin_;
     }

     int height()
     {
          return ymax_ - ymin_;
     }
     
     cv::Rect rectangle() 
     {
          //return Rectangle(Point<int>(xmin_,ymin_),Point<int>(xmax_,ymax_));
          return cv::Rect(xmin_, ymin_, xmax_-xmin_, ymax_-ymin_);
     }

     cv::Point centroid()
     {
          double x_pos = round(double(xmax_+xmin_) / 2.0);
          double y_pos = round(double(ymax_+ymin_) / 2.0);
          cv::Point p(x_pos, y_pos);
          return p;
     }

     // Given this Bounding Box's centroid and the desired width and height,
     // Create a rectangle with the centroid at the center of the box with
     // given width and height
     cv::Rect ForceBox(int width, int height)
     {
          cv::Point centroid = this->centroid();
          cv::Rect rect(centroid.x-width/2, centroid.y-height/2, width, height);
          return rect;
     }

     // Returns true if the Bounding Box contains the provided point
     bool contains(cv::Point point)
     {
          if (xmin_ < point.x && xmax_ > point.x && 
              ymin_ < point.y && ymax_ > point.y) { 
               return true;
          } else {
               return false;
          }
     }
          
protected:
     int xmin_;
     int xmax_;
     int ymin_;     
     int ymax_;

private:
};

#endif
