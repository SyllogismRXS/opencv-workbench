#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_
/// ---------------------------------------------------------------------------
/// @file BoundingBox.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-04-29 17:51:53 syllogismrxs>
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
#include <opencv_workbench/utils/Rectangle.h>

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
          
     int xmin() { return xmin_; }
     int xmax() { return xmax_; }
     int ymin() { return ymin_; }
     int ymax() { return ymax_; }     
               
     Rectangle rectangle() 
     {
          return Rectangle(Point<int>(xmin_,ymax_),Point<int>(xmax_,ymin_));
     }
     
protected:
     int xmin_;
     int xmax_;
     int ymin_;     
     int ymax_;

private:
};

#endif
