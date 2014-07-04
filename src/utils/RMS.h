#ifndef RMS_H_
#define RMS_H_
/// ---------------------------------------------------------------------------
/// @file RMS.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2014-07-04 12:16:18 syllogismrxs>
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
#include <math.h>

#include <opencv2/core/core.hpp>

using std::cout;
using std::endl;

class RMS {
public:     
     void push(cv::Point2f actual, cv::Point2f predicted)
     {
          actual_.push_back(actual);
          predicted_.push_back(predicted);
     }

     cv::Point2f rms_error()
     {
          // RMS:
          // 1. Square the difference
          // 2. Average the squared differences
          // 3. Take the square root
          cv::Point2f sum;
          sum.x = sum.y = 0;
          
          if (actual_.size() != predicted_.size()) {
               cout << "Actual and Predicted data points don't match." << endl;
          }
          int count = actual_.size();

          std::vector<cv::Point2f>::iterator it_actual;
          std::vector<cv::Point2f>::iterator it_predicted;
          for (it_actual = actual_.begin(), it_predicted = predicted_.begin(); 
               it_actual != actual_.end(); it_actual++, it_predicted++) {
               sum.x += pow(it_predicted->x - it_actual->x, 2);
               sum.y += pow(it_predicted->y - it_actual->y, 2);
          }

          cv::Point2f avg;
          avg.x = sum.x / (float)count;
          avg.y = sum.y / (float)count;
          
          cv::Point2f rms;
          rms.x = sqrt(avg.x);
          rms.y = sqrt(avg.y);
          
          return rms;
     }    

     void reset()
     {
          actual_.clear();
          predicted_.clear();
     }

protected:
private:     
     std::vector<cv::Point2f> actual_;
     std::vector<cv::Point2f> predicted_;
};

#endif
