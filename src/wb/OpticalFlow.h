#ifndef OPTICALFLOW_H_
#define OPTICALFLOW_H_
/// ---------------------------------------------------------------------------
/// @file OpticalFlow.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-12-17 15:54:23 syllogismrxs>
///
/// @version 1.0
/// Created: 17 Dec 2015
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
/// The OpticalFlow class ...
/// 
/// ---------------------------------------------------------------------------

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"

class OpticalFlow {
public:
     OpticalFlow();

     void drawOptFlowMap(const cv::Mat &flow, cv::Mat &cflowmap, 
                         int step, double scale, 
                         cv::Scalar color);
     int dense_flow(const cv::Mat &input_gray, cv::Mat &outputImg);

     int sparse_flow(cv::Mat &input_gray, cv::Mat &output);

protected:

     cv::Mat prev_gray;

     bool needToInit;
     std::vector<cv::Point2f> points[2];
     cv::TermCriteria termcrit;
     cv::Size subPixWinSize;
     cv::Size winSize;
     int MAX_COUNT;

private:
};

#endif
