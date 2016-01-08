#ifndef POLAR_H_
#define POLAR_H_
/// ---------------------------------------------------------------------------
/// @file Polar.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-01-08 13:14:13 syllogismrxs>
///
/// @version 1.0
/// Created: 13 Oct 2015
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
/// The Polar class ...
/// 
/// ---------------------------------------------------------------------------

class Polar {
public:
     Polar() : r(0), theta(0) { }
     Polar(double range, double th) : r(range), theta(th) { }  
     double r;
     double theta;

     static Polar cart2polar(cv::Point2d &p)
     {
          Polar polar;
          polar.r = sqrt( pow(p.x,2) + pow(p.y,2) );
          polar.theta = atan(p.y / p.x);
          return polar;
     }

     static double distance(Polar &p1, Polar &p2)
     {
          double dist;
          dist = sqrt( pow(p1.r,2) + pow(p2.r, 2) - 2*p1.r*p2.r*cos(p1.theta-p2.theta));
          return dist;
     }
     
protected:
private:
};

#endif
