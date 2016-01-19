#ifndef NDT_H_
#define NDT_H_
/// ---------------------------------------------------------------------------
/// @file NDT.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-01-18 13:56:26 syllogismrxs>
///
/// @version 1.0
/// Created: 06 Sep 2015
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
/// The NDT class ...
/// 
/// ---------------------------------------------------------------------------
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/utils/Stream.h>

#include <Eigen/Dense>


class Cell {
public:
     Cell() { 
     mean_.resize(2,1);// = cv::Mat_<double>(2,1,CV_64F);
     covar_.resize(2,2);// = cv::Mat_<double>(2,2,CV_64F);
          is_valid_ = false;
          mod_eigs_ = false;
     }

     cv::Rect rect_;

     Eigen::MatrixXd mean_idx_;
     Eigen::MatrixXd covar_idx_;

     Eigen::MatrixXd mean_;
     Eigen::MatrixXd covar_;

     bool mod_eigs_;
//cv::Mat_<double> mean_;//(2,1,CV_64F);
//cv::Mat_<double> covar_;//(2,2);
     bool is_valid_;
};

class NDT {
public:
     NDT();
void set_frame(cv::Mat &src, cv::Mat &dst, syllo::Stream *stream);
protected:
     int cell_col_size_;
     int cell_row_size_;

     std::vector< std::vector< std::vector<Cell> > > cells_;
     std::vector< std::vector< std::vector<Cell> > > prev_cells_;     

     cv::Mat prev_ndt_img_;
     cv::Mat prev_img_;

     Eigen::MatrixXd xy_est_;     
     long double phi_est_;

     int frame_;
     
private:
};

#endif
