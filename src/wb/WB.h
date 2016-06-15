#ifndef WB_H_
#define WB_H_
/// ---------------------------------------------------------------------------
/// @file WB.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-06-15 11:09:30 syllogismrxs>
///
/// @version 1.0
/// Created: 31 Aug 2015
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
/// The WB class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <opencv_workbench/wb/Cluster.h>

#include <Eigen/Dense>

namespace wb {

     template<typename T> double distance(T p1, T p2)
     {
          return sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
     }

     void cluster_points(cv::Mat &src, std::list<wb::Cluster*> &clusters, 
                         int thresh, float gate, int min_cluster_size);

     void draw_clusters(cv::Mat &src, cv::Mat &dst, 
                        std::list<wb::Cluster*> &clusters);

     void adaptive_threshold(cv::Mat &src, cv::Mat& dst, int &thresh, 
                             double ratio_low, double ratio_high, 
                             int thresh_step, int max_iter);
     
     void gradient_sobel(cv::Mat &src, cv::Mat &dst);
     void gradient_simple(cv::Mat &src, cv::Mat &dst);

     void showHistogram(const cv::Mat& img, cv::Mat &mask);
     void opencv_histogram(cv::Mat &src);

     void get_sonar_mask(const cv::Mat &jet, cv::Mat &mask);

     void adaptive_threshold(cv::Mat &src, cv::Mat& dst, int &thresh, 
                             double desired_ratio, int max_iter, cv::Mat &mask);

     double getThreshVal_Otsu_8u(cv::Mat& _src , cv::Mat &mask );
     void multi_otsu(cv::Mat &_src, cv::Mat &_dst, cv::Mat &_mask, cv::Mat &_first);

     double gaussian_probability(Eigen::MatrixXd &x, 
                                 Eigen::MatrixXd &u, 
                                 Eigen::MatrixXd &cov);

     void cartesian2polar(double x, double y, double &range, double &bearing);
     cv::Point2d rowcol2cartesian(syllo::Stream *stream, int r, int c);
     cv::Point cartesian2rowcol(syllo::Stream *stream, cv::Point2d cart);
}

#endif
