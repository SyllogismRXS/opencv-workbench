#ifndef ENTITY_H_
#define ENTITY_H_
/// ---------------------------------------------------------------------------
/// @file Entity.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-10-09 14:53:58 syllogismrxs>
///
/// @version 1.0
/// Created: 25 Sep 2015
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
/// The Entity class ...
/// 
/// ---------------------------------------------------------------------------
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_workbench/utils/BoundingBox.h>
//#include <opencv2/video/tracking.hpp>

#include <opencv_workbench/wb/Point.h>
#include <opencv_workbench/track/KalmanFilter.h>

namespace wb {

     class Entity {
     public:

          typedef enum EntityType {
               Unknown = 0,
               Diver,
               Ground,
               Clutter,
               Rock,
               Barrel,
               Mammal          
          }EntityType_t;
          
          Entity();

          void set_id(int id) { id_ = id; }
          int id() { return id_; }
          
          void set_type(EntityType_t type) { type_ = type; }
          EntityType_t type() { return type_; }
          std::string name();
          void set_name(std::string name) { name_ = name; }
          
          void compute_metrics();

          cv::Point centroid();
          cv::Rect rectangle();
          BoundingBox & bbox() { return bbox_; }
          void set_bbox(BoundingBox bbox) { bbox_ = bbox; }

          void set_centroid(cv::Point p) { centroid_ = p; }
          //void set_rectangle(cv::Rect rect) { rectangle_ = rect; }

          // Point related
          std::vector<wb::Point> & points() { return points_; }
          int size() { return points_.size(); }
          void add_point(wb::Point &p) { points_.push_back(p); }
          void remove_point(wb::Point &p);
               
          // Age related functions
          void inc_age();
          void dec_age();
          int age() { return age_; }
          void set_age(int age);
          bool is_visible();
          bool is_tracked();
          bool is_dead();
          
          bool occluded() { return occluded_; }
          void set_occluded(bool occluded) { occluded_ = occluded; }

          //void set_tracker(cv::KalmanFilter KF) { KF_ = KF;}          
          //cv::KalmanFilter tracker() { return KF_; }
          void set_tracker(syllo::KalmanFilter kf) { kf_ = kf; }
          syllo::KalmanFilter tracker() { return kf_; }

          void init();
          void predict_tracker();
          void correct_tracker();
          cv::Point estimated_centroid();
          
          void set_matched(bool matched) { matched_ = matched; }
          bool matched() { return matched_; }
          
     protected:
          int id_;    
          std::string name_;
          EntityType_t type_;
          std::vector<wb::Point> points_;
          cv::Point centroid_;
          cv::Point est_centroid_;
          //cv::Rect rectangle_;
          int age_;
          bool occluded_;
          bool is_tracked_;
          BoundingBox bbox_;

          //cv::KalmanFilter KF_;
          cv::Mat_<float> transition_matrix_;

          syllo::KalmanFilter kf_;
          Eigen::MatrixXf A;     // system matrix
          Eigen::MatrixXf B;     // input matrix
          Eigen::MatrixXf H;     // measurement matrix
          Eigen::MatrixXf Q;     // process noise matrix
          Eigen::MatrixXf R;     // measurement noise matrix
          Eigen::MatrixXf x0;    // initial position
          Eigen::MatrixXf covar; // initial state covariance matrix 
          Eigen::MatrixXf u; // Input acceleration 
          Eigen::MatrixXf z; // Measurement (position)
          Eigen::MatrixXf state; // estimated state

          float distance_;
          bool matched_;

          void set_distance(float distance) { distance_ = distance; }
          float distance() { return distance_; }

     private:
     };

}

#endif
