#ifndef TRACK_H_
#define TRACK_H_
/// ---------------------------------------------------------------------------
/// @file Track.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-07-02 18:20:09 syllogismrxs>
///
/// @version 1.0
/// Created: 02 Jul 2015
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
/// The Track class ...
/// 
/// ---------------------------------------------------------------------------
// OpenCV headers
#include <cv.h>
#include <highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace syllo {

     typedef enum TrackType {
          Unknown = 0,
          Diver,
          Ground,
          Clutter,
          Rock,
          Barrel,
          Mammal          
     }TrackType_t;
     
     class Track {
     public:
          Track() { name_ = "undefined"; }
          
          void set_type(TrackType_t type) { type_ = type; }
          TrackType_t type() { return type_; }
          int age() { return age_; }
          std::string name() { return name_; }

void set_position(cv::Point3d point) { point_ = point; }
cv::Point3d position() { return point_; }

     protected:
          TrackType_t type_;
          cv::Point3d point_;
          int age_;
          std::string name_;
     private:
     
     };

}

#endif
