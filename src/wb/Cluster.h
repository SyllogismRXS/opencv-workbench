#ifndef _WB_CLUSTER_H_
#define _WB_CLUSTER_H_
/// ---------------------------------------------------------------------------
/// @file Cluster.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-09-25 12:34:13 syllogismrxs>
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
/// The Cluster class ...
/// 
/// ---------------------------------------------------------------------------
#include <vector>

#include "Point.h"

#include <opencv2/video/tracking.hpp>
//#include <opencv_workbench/track/EKF.h>

#include <opencv_workbench/wb/Entity.h>

//#include <Eigen/Dense>

namespace wb {

     class Cluster : public Entity {
     public:
          //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          Cluster();                    

          void init();
          
          cv::Point estimated_centroid() { return est_centroid_; }

          void set_matched(bool matched) { matched_ = matched; }
          bool matched() { return matched_; }
          
          void set_distance(float distance) { distance_ = distance; }
          float distance() { return distance_; }

          void set_match(Cluster * match) { match_ = match; }
          Cluster * match() { return match_; }                   

          void predict_tracker();
          void correct_tracker();

          void set_tracker(cv::KalmanFilter KF) { KF_ = KF;}
          cv::KalmanFilter tracker() { return KF_; }
          
     protected:
     private:
          float distance_;
          bool matched_;
          Cluster * match_;  
          
          //syllo::EKF ekf_;
          //
          //Eigen::MatrixXf A;
	  //Eigen::MatrixXf B;
	  //Eigen::VectorXf C;
	  //Eigen::MatrixXf R;
	  //Eigen::MatrixXf Q;
	  //Eigen::VectorXf mu;
	  //Eigen::MatrixXf covar;
	  //Eigen::VectorXf u;
          //
	  //Eigen::VectorXf estVel;
          cv::KalmanFilter KF_;

          cv::Mat_<float> transition_matrix_;
          
     };
}

#endif
