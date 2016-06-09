#ifndef ENTITY_H_
#define ENTITY_H_
/// ---------------------------------------------------------------------------
/// @file Entity.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-06-08 20:01:31 syllogismrxs>
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
//#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/utils/Ellipse.h>
#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/wb/ScalarTracker.h>
#include <opencv_workbench/wb/PositionTracker.h>

     typedef enum OverlayFlags {
          BLOBS          = 1 << 0,
          RECTS          = 1 << 1,
          TRACKS         = 1 << 2,
          IDS            = 1 << 3,
          ERR_ELLIPSE    = 1 << 4,
          CONFIRMED_ONLY = 1 << 5,
          VELOCITIES     = 1 << 6
     }OverlayFlags_t;
     
     inline OverlayFlags_t operator|(OverlayFlags_t a, OverlayFlags_t b)
     {
          return static_cast<OverlayFlags_t>(static_cast<int>(a) | 
                                             static_cast<int>(b));
     }


namespace wb {
     
     class Entity {
     public:

          typedef enum EntityType {
               Unknown = 0,
               Diver,
               Fish,
               Ground,
               Clutter,
               Rock,
               Barrel,
               Mammal,
               Sensor,
               APoint
          }EntityType_t;

          typedef enum MhtType {
               fp = 0,
               dt,
               nt,
               root
          }MhtType_t;

          MhtType_t mht_type;                    
          
          Entity();

          void set_id(int id) { id_ = id; }
          int id() { return id_; }
          
          void set_type(EntityType_t type) { type_ = type; }
          EntityType_t type() { return type_; }
          std::string name();
          void set_name(std::string name);

          static wb::Entity::EntityType_t str_2_type(std::string str);
          std::string type_str();
          
          void compute_metrics();

          cv::Point pixel_centroid() { return pixel_centroid_; }
          cv::Point2d centroid() { return centroid_; }
          
          cv::Point start_pixel_centroid() { return start_pixel_centroid_; }
          void set_start_pixel_centroid(cv::Point2d start_pixel_centroid) { start_pixel_centroid_ = start_pixel_centroid; }
          
          cv::Point2d start_centroid() { return start_centroid_; }
          void set_start_centroid(cv::Point2d start_centroid) { start_centroid_ = start_centroid; }
          
          cv::Rect rectangle();
          BoundingBox & bbox() { return bbox_; }
          void set_bbox(BoundingBox bbox) { bbox_ = bbox; }

          void set_centroid(cv::Point2d p) { centroid_ = p; }
          void set_pixel_centroid(cv::Point p) { pixel_centroid_ = p; }
          
          // Point related
          std::vector<wb::Point> & points() { return points_; }
          int size() { return points_.size(); }
          void add_point(wb::Point &p) { points_.push_back(p); }
          void add_points( std::vector<wb::Point> &points);
          void remove_point(wb::Point &p);
               
          int class_age() { return class_age_; }
          void inc_class_age() { class_age_++; }
          
          // Age related functions
          void inc_age();
          void dec_age();
          
          void inc_occluded_age() { occluded_age_++; }
          void dec_occluded_age() { occluded_age_--; }
          void set_occluded_age(int age) { occluded_age_ = age; } 
          int occluded_age() { return occluded_age_;} 
          
          int age() { return age_; }
          void set_age(int age);
          bool is_confirmed();
          bool is_dead();

          void new_track(int id);
          void missed_track();
          void detected_track();

          void matched_track(Entity &match);
          void copy_track_info(Entity &other);

          void copy_meas_info(Entity &other);
          
          bool occluded() { return occluded_; }
          void set_occluded(bool occluded);          

          //void set_tracker(syllo::KalmanFilter kf) { kf_ = kf; }
          //syllo::KalmanFilter &tracker() { return kf_; }
          void set_pixel_tracker(PositionTracker t) { pixel_tracker_ = t; }
          PositionTracker &pixel_tracker() { return pixel_tracker_; }

          void set_tracker(PositionTracker t) { tracker_ = t; }
          PositionTracker &tracker() { return tracker_; }

          void init();
          void predict_tracker();
          void correct_tracker();
          cv::Point estimated_pixel_centroid();
          cv::Point2d estimated_centroid();

          cv::Point estimated_pixel_velocity();
          
          void set_estimated_pixel_centroid(cv::Point p);
          void set_estimated_centroid(cv::Point2d p);
          
          Ellipse error_ellipse(double confidence);
          
          void set_matched(bool matched) { matched_ = matched; }
          void set_visited(bool visited) { visited_ = visited; }
          bool visited() { return visited_; }
          bool matched() { return matched_; }
          int matched_id() { return matched_id_; }
          void set_matched_id(int matched_id) { matched_id_ = matched_id; }

          void set_frame(int frame) { frame_ = frame; }
          int frame() { return frame_; }
          
          // Only public, so they can be used in graphviz
          int id_;
          double prob_; // branch probability
          double norm_prob_;

          void set_cluster_id(int cluster_id) { cluster_id_ = cluster_id; }
          int cluster_id() { return cluster_id_; }

          std::list<cv::Point2f> & trail() { return trail_; }
          cv::Point trail_history(int past);
          void update_trail();

          
          void set_prob(double prob, bool norm=false) 
          { 
               if (norm) norm_prob_ = prob;                   
               prob_ = prob; 
          }
          double prob() { return prob_; }          

          void set_pixel_R(double r);
          void set_pixel_P(double p);

          void set_R(double r);
          void set_R(double r0, double r1, double r2, double r3);
          void set_P(double p);

          void set_stream(syllo::Stream *stream) { stream_ = stream; }
          
          float avg_pixel_value() { return avg_pixel_value_; }

          int estimated_pixel_value();
          int estimated_blob_size();

          int lower_pixel_value(float num_stds);
          int lower_blob_size(float num_stds);

          void set_confirmed_age(int a) { confirmed_age_ = a; }
          void set_dead_occluded_age(int a) { dead_occluded_age_ = a; }
          
     protected:     

          int class_age_;
          
          bool prob_is_set_;
          std::string name_;
          EntityType_t type_;
          std::vector<wb::Point> points_;
          
          cv::Point pixel_centroid_; // image (col,row) location
          cv::Point2d centroid_;     // x/y coordinate in space
          
          int age_;
          int occluded_age_;
          bool occluded_;
          bool is_confirmed_;
          BoundingBox bbox_;

          int frame_;
          
          int matched_id_;          

          PositionTracker pixel_tracker_;
          PositionTracker tracker_;
          
          ////cv::KalmanFilter KF_;
          ////cv::Mat_<float> transition_matrix_;
          //
          //syllo::KalmanFilter kf_;
          //Eigen::MatrixXf A;     // system matrix
          //Eigen::MatrixXf B;     // input matrix
          //Eigen::MatrixXf H;     // measurement matrix
          //Eigen::MatrixXf Q;     // process noise matrix
          //Eigen::MatrixXf R;     // measurement noise matrix
          //Eigen::MatrixXf x0;    // initial position
          //Eigen::MatrixXf covar; // initial state covariance matrix 
          //Eigen::MatrixXf u; // Input acceleration 
          //Eigen::MatrixXf z; // Measurement (position)
          //Eigen::MatrixXf state; // estimated state

          float distance_;
          bool matched_;
          bool visited_;

          int cluster_id_;

          cv::Point start_pixel_centroid_;          
          cv::Point2d start_centroid_;          
          
          std::list<cv::Point2f> trail_;
          
          void set_distance(float distance) { distance_ = distance; }
          float distance() { return distance_; }   

          syllo::Stream *stream_;

          ScalarTracker pixel_value_tracker_;
          ScalarTracker size_tracker_;

          float avg_pixel_value_;

          int confirmed_age_;
          int dead_occluded_age_;

     private:
     };
}

#endif
