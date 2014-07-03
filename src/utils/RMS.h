#ifndef RMS_H_
#define RMS_H_
/// ---------------------------------------------------------------------------
/// @file RMS.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2014-07-03 19:14:13 syllogismrxs>
///
/// @version 1.0
/// Created: 03 Jul 2014
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
/// The RMS class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <vector>

class RMS {
public:     
     void push(double actual, double predicted)
     {
          actual_.push_back(actual);
          predicted_.push_back(predicted);
     }

     double rms_error()
     {
          // RMS:
          // 1. Square the difference
          // 2. Average the squared differences
          // 3. Take the square root
          double sum = 0;

          std::vector<double>::iterator it_actual = actual_.begin();
          std::vector<double>::iterator it_predicted = predicted_.begin();
          for (it_actual = actual_.begin(); it_actual != actual_.end(); it_actual++) {
               sum += pow(*it_actual - *it_predicted,2);
               
               it_predicted++;
          }
          
     }    

     void reset()
     {
          actual_.clear();
          predicted_.clear();
     }

protected:
private:     
     std::vector<double> actual_;
     std::vector<double> predicted_;
};

#endif
