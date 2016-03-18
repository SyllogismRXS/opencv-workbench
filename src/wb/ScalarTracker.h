#ifndef SCALARTRACKER_H_
#define SCALARTRACKER_H_
/// ---------------------------------------------------------------------------
/// @file ScalarTracker.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-03-18 14:41:50 syllogismrxs>
///
/// @version 1.0
/// Created: 18 Mar 2016
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
/// The ScalarTracker class ...
/// 
/// ---------------------------------------------------------------------------
#include <opencv_workbench/track/KalmanFilter.h>
#include <Eigen/Dense>

class ScalarTracker {
public:
     ScalarTracker();
     void set_value(float value);
     void predict();
     float value();
     float lower_end(float num_stds);
protected:

     syllo::KalmanFilter kf_;

     bool initialized_;

     Eigen::MatrixXf z_;
     Eigen::MatrixXf u_;
     Eigen::MatrixXf x0_;
     Eigen::MatrixXf P0_;          
private:     
};

#endif
