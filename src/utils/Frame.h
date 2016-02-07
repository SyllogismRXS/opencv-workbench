#ifndef FRAME_H_
#define FRAME_H_
/// ---------------------------------------------------------------------------
/// @file Frame.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-02-07 12:49:39 syllogismrxs>
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
/// The Frame class ...
/// 
/// ---------------------------------------------------------------------------

//#include <opencv_workbench/utils/Object.h>
#include <opencv_workbench/wb/Entity.h>

class Frame {
public:

     typedef enum FrameType {
          unused = 0,
          train,
          validate,
          test
     }FrameType_t;
     
     Frame() : frame_number_(0), frame_type_(train)  {}
     
     void set_frame_type(FrameType_t type) { frame_type_ = type; }
     FrameType_t frame_type() { return frame_type_; }
     
     void set_frame_number(int frame_number) {frame_number_ = frame_number;}
     int frame_number() { return frame_number_; }
     std::map<std::string, wb::Entity> objects;
     
     void erase(std::string str) 
     {
          objects.erase(str);
     }
     
     bool contains_object(const std::string &name, wb::Entity &object) 
     {
          if (objects.count(name) > 0) {
               object = objects[name];
               return true;
          } else {
               return false;
          }
     }

     bool contains_type(wb::Entity::EntityType_t &type, wb::Entity &object)
     {
          for (std::map<std::string, wb::Entity>::iterator it = objects.begin();
               it != objects.end(); it++) {
               if (it->second.type() == type) {
                    object = it->second;
                    return true;
               }
          }
          return false;                    
     }
     
protected:
     int frame_number_;
     FrameType_t frame_type_;
private:
};

#endif
