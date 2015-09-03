#ifndef POINT_H_
#define POINT_H_
/// ---------------------------------------------------------------------------
/// @file Point.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-09-02 18:09:44 syllogismrxs>
///
/// @version 1.0
/// Created: 31 Aug 2015
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
/// The Point class ...
/// 
/// ---------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

//#include "Cluster.h"

namespace wb {
     
     class Cluster;
     
     class Point {
     public:
          Point();   
          
          void set_position(cv::Point point) { point_ = point; }
          cv::Point position() { return point_; }
          
          int value() { return value_; }
          void set_value(int value) { value_ = value; }
          
          float distance() { return distance_; }
          void set_distance(float distance) { distance_ = distance; }
          
          Cluster * parent() { return parent_; }
          void set_parent(wb::Cluster *parent);

          float distance(Point &other);

          bool assigned() { return assigned_; }
          void set_assigned(bool assigned) { assigned_ = assigned; }

     protected:
     private:          
          cv::Point point_;
          int value_;
          bool assigned_;
          float distance_;
          Cluster * parent_;
     };
}

#endif
