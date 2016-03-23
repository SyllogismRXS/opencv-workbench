#ifndef MATRIXTRACKER_H_
#define MATRIXTRACKER_H_
/// ---------------------------------------------------------------------------
/// @file MatrixTracker.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-03-22 20:06:46 syllogismrxs>
///
/// @version 1.0
/// Created: 22 Mar 2016
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
/// The MatrixTracker class ...
/// 
/// ---------------------------------------------------------------------------
#include <opencv_workbench/wb/ScalarTracker.h>
#include <Eigen/Dense>

class MatrixTracker {
public:
     MatrixTracker();     
     void init(int rows, int cols);
     void set_values(Eigen::MatrixXd &vals);
     void set_values(std::vector<double> &vals);
     void predict();
     Eigen::MatrixXd values();
protected:
     std::vector<ScalarTracker> elems_;
private:     
     int rows_;
     int cols_;
};

#endif
