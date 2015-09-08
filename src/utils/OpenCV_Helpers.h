#ifndef OPENCV_HELPERS_H_
#define OPENCV_HELPERS_H_
/// ---------------------------------------------------------------------------
/// @file OpenCV_Helpers.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-09-08 17:50:13 syllogismrxs>
///
/// @version 1.0
/// Created: 21 May 2014
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
/// The OpenCV_Helpers class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

using std::cout;
using std::endl;

namespace wb
{    
     void show(const cv::Mat &img)
     {
          cv::imshow("DEBUG!",img);
          cv::waitKey(0);
     }

     void print_size(const cv::Mat &img)
     {
          //cout << img.rows << "x" < img.cols << endl;
          printf("%dx%d\n",img.rows,img.cols);
     }

     void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color, int d)
     {
          cv::line(img, cv::Point(center.x - d, center.y - d), cv::Point(center.x + d, center.y + d), color, 2, CV_AA, 0);
          cv::line(img, cv::Point(center.x + d, center.y - d), cv::Point(center.x - d, center.y + d), color, 2, CV_AA, 0 );
     }              
}
#endif
