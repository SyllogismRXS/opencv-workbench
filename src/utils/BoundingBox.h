#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_
/// ---------------------------------------------------------------------------
/// @file BoundingBox.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-01-29 13:53:16 syllogismrxs>
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
#include <iostream>
#include <math.h>

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;

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

     static bool overlap(const BoundingBox &b1, const BoundingBox &b2)
     {
          // If one rectangle is on the left side of the other
          if (b1.xmax_ < b2.xmin_ || b2.xmax_ < b1.xmin_) {
               return false;
          }

          // If one rectangle is above the other
          if (b1.ymax_ < b2.ymin_ || b2.ymax_ < b1.ymin_) {
               return false;
          }               
          
          return true;
     }

     template<typename T> static double distance_temp(T p1, T p2)
     {
          return sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
     }

     void print()
     {
          std::cout << "Box: (" << xmin_ << "," << ymin_ << ") to (" 
                    << xmax_ << "," << ymax_ << ")" << endl;
     }
     
     static double distance(const BoundingBox &b1, const BoundingBox &b2)
     {
          if (overlap(b1,b2)) {
               return 0;
          }
     
          // b2 is the center of a keypad (#5)
          // b1 is moved around the keypad
     
          // Check #1 quadrant
          if (b1.xmax_ < b2.xmin_ && b1.ymax_ < b2.ymin_) {
               // Return the distance between the two corners
               return distance_temp<cv::Point2f>(cv::Point2f(b1.xmax_, b1.ymax_),
                                                 cv::Point2f(b2.xmin_, b2.ymin_));
          } 
          // Check #3 quadrant
          else if (b1.xmin_ > b2.xmax_ && b1.ymax_ < b2.ymin_) {
               return distance_temp<cv::Point2f>(cv::Point2f(b1.xmin_, b1.ymax_),
                                                 cv::Point2f(b2.xmax_, b2.ymin_));
          }
          // Check #7 quadrant
          else if (b1.xmax_ < b2.xmin_ && b1.ymin_ > b2.ymax_) {
               return distance_temp<cv::Point2f>(cv::Point2f(b1.xmax_, b1.ymin_),
                                                 cv::Point2f(b2.xmin_, b2.ymax_));
          }
          // Check #9 quadrant
          else if (b1.xmin_ > b2.xmax_ && b1.ymin_ > b2.ymax_) {
               return distance_temp<cv::Point2f>(cv::Point2f(b1.xmin_, b1.ymin_),
                                                 cv::Point2f(b2.xmax_, b2.ymax_));
          }
          // Check #2 quadrant
          else if (b1.ymax_ < b2.ymin_) {
               return distance_temp<cv::Point2f>(cv::Point2f(0, b1.ymax_),
                                                 cv::Point2f(0, b2.ymin_));
          }
          // Check #8 quadrant
          else if (b1.ymin_ > b2.ymax_) {
               return distance_temp<cv::Point2f>(cv::Point2f(0, b1.ymin_),
                                                 cv::Point2f(0, b2.ymax_));
          }
          // Check #4 quadrant
          else if (b1.xmax_ < b2.xmin_) {
               return distance_temp<cv::Point2f>(cv::Point2f(b1.xmax_, 0),
                                                 cv::Point2f(b2.xmin_, 0));
          }
          // Check #6 quadrant
          else if (b1.xmin_ > b2.xmax_) {
               return distance_temp<cv::Point2f>(cv::Point2f(b1.xmin_, 0),
                                                 cv::Point2f(b2.xmax_, 0));
          } else {
               std::cout << "======> WARNING CAN'T FIND RECT DISTANCE" << std::endl;
               return 1000;
          }
     }

     // TODO: Something is strange with this file and Cluster.h/cpp
     //friend std::ostream& operator<<(std::ostream& os, const BoundingBox& b);
     
protected:
     int xmin_;
     int xmax_;
     int ymin_;     
     int ymax_;

private:
};

//std::ostream& operator<<(std::ostream& os, const BoundingBox& b)
//{
//     os << "Box: (" << b.xmin_ << "," << b.ymin_ << ") to (" << b.xmax_ 
//        << "," << b.ymax_ << ")";
//     return os;
//}

#endif
