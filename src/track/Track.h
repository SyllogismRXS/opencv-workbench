#ifndef TRACK_H_
#define TRACK_H_
/// ---------------------------------------------------------------------------
/// @file Track.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-07-03 19:47:18 syllogismrxs>
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
#include <opencv_workbench/syllo/syllo.h>

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
          Track() 
          { 
               name_ = "Unknown:-1"; 
               type_ = Unknown;
               id_ = -1;
          }
          
          void set_type(TrackType_t type) { type_ = type; }
          TrackType_t type() { return type_; }
          int age() { return age_; }
          void set_age(int age) { age_ = age; }

          std::string name() 
          {
               switch (type_) {
               case Unknown:
                    name_ = "Unknown";
                    break;
               case Diver:
                    name_ = "Diver";
                    break;
               case Clutter:
                    name_ = "Clutter";
                    break;
               default:
                    name_ = "Unknown";
               }                             

               return name_; 
          }

          void set_id(int id) { id_ = id; }
          int id() { return id_; }

void set_position(cv::Point3d point) { point_ = point; }
cv::Point3d position() { return point_; }

     protected:
          TrackType_t type_;
          cv::Point3d point_;
          int age_;
          std::string name_;
          int id_;
     private:
     
     };

}

#endif
