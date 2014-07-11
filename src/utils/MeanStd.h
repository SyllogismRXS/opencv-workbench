#ifndef MEANSTD_H_
#define MEANSTD_H_
/// ---------------------------------------------------------------------------
/// @file MeanStd.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2014-07-11 13:56:06 syllogismrxs>
///
/// @version 1.0
/// Created: 11 Jul 2014
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
/// The MeanStd class ...
/// 
/// ---------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <math.h>


class MeanStd {
public:
     void push(double sample)
     {
          samples_.push_back(sample);
     }

     double mean()
     {
          double sum = 0;
          std::vector<double>::iterator it;
          for (it = samples_.begin(); it != samples_.end(); it++) {
               sum += *it;
          }
          return sum / (double)samples_.size();
     }
     
     double variance()
     {
          double mean = this->mean();
          double sum = 0;
          std::vector<double>::iterator it;
          for (it = samples_.begin(); it != samples_.end(); it++) {
               sum += pow((*it)-mean,2);
          }
          return sum / (double)samples_.size();
     }

     double std()
     {
          return sqrt(this->variance());
     }

     void reset()
     {
          samples_.clear();
     }
protected:
private:
     std::vector<double> samples_;
};

#endif
