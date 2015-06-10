#ifndef SAM_TYPES_H_
#define SAM_TYPES_H_
/// ---------------------------------------------------------------------------
/// @file types.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2014-09-15 18:46:19 syllogismrxs>
///
/// @version 1.0
/// Created: 15 Apr 2014
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
/// The types class ...
/// 
/// ---------------------------------------------------------------------------
#include <boost/array.hpp>
#include <math.h>

namespace syllo {
     class Point3D
     {
     public:
          double x;
          double y;
          double z;

          Point3D() 
          {
               this->x = 0;
               this->y = 0;
               this->z = 0;
          }

          Point3D(double x, double y, double z)
          {
               this->x = x;
               this->y = y;
               this->z = z;
          }

          static double distance(const Point3D &p0, const Point3D &p1)
          {
               return sqrt( pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) 
                            + pow(p0.z - p1.z, 2));
          }

          friend std::ostream& operator<<(std::ostream& os, const Point3D& p)
          {
               os << p.x << ", " << p.y << ", " << p.z;
               return os;
          }
     };     

     class Point {
     public:
          double x;
          double y;
          
          Point() 
          { 
               this->x = 0;
               this->y = 0;
          }

          Point(double x, double y) 
          {
               this->x = x;
               this->y = y;
          }

          static double distance(const Point &p0, const Point &p1)
          {
               return sqrt( pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
          }
     };

typedef boost::array< double, 1 > state_1d_type;
typedef boost::array< double , 3 > state_3d_type;     
typedef boost::array< double , 5 > state_5d_type;

}     

#endif
