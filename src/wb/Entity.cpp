#include <iostream>

#include <boost/tokenizer.hpp>

#include <opencv_workbench/syllo/syllo.h>

#include "Entity.h"

using std::cout;
using std::endl;

#define MAX_AGE 10
#define VISIBLE_AGE 5
#define START_TRACK_AGE 5
#define MIN_AGE -2
#define DEAD_OCCLUDED_AGE 10

namespace wb {

     Entity::Entity() : id_(-1), name_("unknown:-1"), type_(Unknown), age_(0),
                        occluded_age_(0), occluded_(false), is_tracked_(false), 
                        visited_(false), cluster_id_(0)
     {
          //KF_ = cv::KalmanFilter(4, 2, 0);          
          //transition_matrix_ = cv::Mat_<float>(4,4);
          //transition_matrix_  << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1;
          //KF_.transitionMatrix = transition_matrix_;     
          
          A.resize(4,4);
          B.resize(4,2);
          H.resize(2,4);
          Q.resize(4,4);
          R.resize(2,2);
          x0.resize(4,1);
          covar.resize(4,4);
          
          double T = 0.066666667;//; dt     
          A << 1, 0, T, 0,
               0, 1, 0, T,
               0, 0, 1, 0,
               0, 0, 0, 1;
     
          B << 0, 0,
               1, 0,
               0, 0,
               0, 1;
     
          H << 1, 0, 0, 0,
               0, 1, 0, 0;
     
          Q = Eigen::MatrixXf::Identity(A.rows(), A.cols()) * 1e-3;
     
          R << 0.01,    0,
                  0, 0.01;
          //R << 10, 0,
          //     0, 10;
     
          double v = 0.1;
          //double v = 1;
          covar << v, 0, 0, 0,
                   0, v, 0, 0,
                   0, 0, v, 0,
                   0, 0, 0, v;
     
          z.resize(2,1);
          u.resize(2,1);
          
          kf_.setModel(A,B,H,Q,R);          
     }

     void Entity::init()
     {
          compute_metrics();
          
          //KF_.statePre.at<float>(0) = centroid_.x;
          //KF_.statePre.at<float>(1) = centroid_.y;
          //KF_.statePre.at<float>(2) = 0;
          //KF_.statePre.at<float>(3) = 0;
          //cv::setIdentity(KF_.measurementMatrix);
          //cv::setIdentity(KF_.processNoiseCov, cv::Scalar::all(1e-3));
          ////cv::setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(10));
          //cv::setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(.01));
          //cv::setIdentity(KF_.errorCovPost, cv::Scalar::all(.1));

          x0 << centroid_.x,
                centroid_.y,
                0,
                0;

          start_centroid_ = centroid_;

          kf_.init(x0, covar);
     }

     cv::Point Entity::start_centroid()
     {
          return start_centroid_;
     }

     void Entity::set_start_centroid(cv::Point start_centroid) 
     { 
          start_centroid_ = start_centroid; 
     }
          
     cv::Point Entity::estimated_centroid()
     {          
          // the Kalman filter's estimate centroid. Otherwise, use the entity's
          // current centroid if it isn't tracked yet
          if (this->is_tracked()) {
               return est_centroid_;
          } else {
               return centroid_;
          }          
     }
     
     Ellipse Entity::error_ellipse(int dim0, int dim1, double confidence)
     {
          return kf_.error_ellipse(dim0, dim1, confidence);
     }

     void Entity::predict_tracker()
     {
          //cv::Mat prediction = KF_.predict();
          //est_centroid_ = cv::Point(prediction.at<float>(0),prediction.at<float>(1));

          u << 0,0;
          kf_.predict(u);
          state = kf_.state();          
          est_centroid_ = cv::Point(round(state(0,0)),round(state(1,0)));
     }

     void Entity::correct_tracker()
     {
          // correct tracker update using the newly computed centroid.
          cv::Mat_<float> measurement(2,1);
          measurement(0) = centroid_.x;
          measurement(1) = centroid_.y;
          
          //cv::Mat estimated = KF_.correct(measurement);

          z << centroid_.x, centroid_.y;
          kf_.update(z);
          state = kf_.state();          
          //est_centroid_ = cv::Point(estimated.at<float>(0),estimated.at<float>(1));
          est_centroid_ = cv::Point(round(state(0,0)),round(state(1,0)));          
     }          

     void Entity::compute_metrics()
     {
          ////////////////////////////////////////////////////////
          // Compute Centroid and Bounding Box Rectangle
          ////////////////////////////////////////////////////////
          int sum_x = 0, sum_y = 0, sum_value = 0;          
          int xmin = 999999999, ymin = 999999999, xmax = -999, ymax = -999;
          std::vector<Point>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               int x = it->position().x;
               int y = it->position().y;
          
               int value = it->value();
               sum_value += value;
               
               sum_x += x * value;
               sum_y += y * value;

               if (x < xmin) {
                    xmin = x;
               }
               if (x > xmax) {
                    xmax = x;
               }
               if (y < ymin) {
                    ymin = y;
               }
               if (y > ymax) {
                    ymax = y;
               }
          }
          
          //double avg_x = (double)sum_x / points_.size();
          //double avg_y = (double)sum_y / points_.size();
          double avg_x = (double)sum_x / (double)sum_value;
          double avg_y = (double)sum_y / (double)sum_value;

          centroid_ = cv::Point(round(avg_x), round(avg_y));
          
          // +1's to account for slight error in rectangle drawing?
          //bbox_ = cv::Rect rect(xmin, ymin, xmax-xmin+1, ymax-ymin+1);
          bbox_.set_box(xmin, xmax+1, ymin, ymax+1);
          
          //rectangle_ = cv::Rect();
     }

     cv::Point Entity::centroid()
     {
          return centroid_;
     }

     cv::Rect Entity::rectangle()
     {
          //return rectangle_;
          return bbox_.rectangle();
     }

     bool Entity::is_visible()
     {
          if (age_ > VISIBLE_AGE) {
               return true;
          } else {
               return false;
          }
     }

     bool Entity::is_tracked()
     {
          if (is_tracked_) return true;

          if (age_ > START_TRACK_AGE) {
               is_tracked_ = true;
               return true;
          } else {
               is_tracked_ = false;
               return false;
          }
     }

     bool Entity::is_dead()
     {
          if (occluded_age_ > DEAD_OCCLUDED_AGE) { 
               return true; 
          } else {
               return false; 
          }
          
          //if (age_ < MIN_AGE) {
          //     return true;
          //} else {
          //     return false;
          //}
     }

     void Entity::inc_age()
     {
          age_++;
          //if (age_ > MAX_AGE) {
          //     age_ = MAX_AGE;
          //}
     }

     void Entity::set_age(int age)
     {
          age_ = age;          
          //if (age_ > MAX_AGE) {
          //     age_ = MAX_AGE;
          //}
     }

     void Entity::dec_age()
     {
          age_--;
     }

     void Entity::missed_track()
     {
          this->set_occluded(true);
          this->inc_occluded_age();
     }
     
     void Entity::detected_track()
     {
          this->inc_age();
          this->set_occluded(false);

          // Update the Kalman Filter Tracker
          this->correct_tracker();
     }

     void Entity::new_track(int id)
     {
          this->set_id(id);
          this->set_age(1);
          this->set_occluded(false);
     }

     void Entity::copy_track_info(Entity &other)
     {
          this->set_id(other.id());
          this->set_age(other.age());
          //this->set_centroid(other.centroid());
          //this->set_estimated_centroid(other.estimated_centroid());
          this->set_start_centroid(other.start_centroid());
          this->set_tracker(other.tracker());          
     }

     void Entity::matched_track(Entity &match)
     {
          // Update the current entity with the track info from "match"
          this->copy_track_info(match);
          
          // Update track variables
          this->detected_track();
     }

     void Entity::remove_point(wb::Point &p)
     {          
          std::vector<Point>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               if (it->position() == p.position()) {
                    points_.erase(it);
                    break;
               }
          }
     }             

     void Entity::add_points(std::vector<wb::Point> &points)
     {
          std::vector<wb::Point>::iterator it = points.begin();
          for (; it != points.end(); it++) {
               points_.push_back(*it);
          }
     }

     std::string Entity::name()
     {
          std::ostringstream convert;
          convert << id_;
          
          switch (type_) {
          case Unknown:
               name_ = "unknown:" + convert.str();
               break;
          case Diver:
               name_ = "diver:" + convert.str();
               break;
          case Clutter:
               name_ = "clutter:" + convert.str();
               break;
          case APoint:
               name_ = "point:" + convert.str();
               break;
          default:
               name_ = "unknown:" + convert.str();
          }                             

          return name_; 
     }

     void Entity::set_name(std::string name)
     {
          boost::char_separator<char> sep(":");
          boost::tokenizer<boost::char_separator<char> > tokens(name, sep);
          int iter = 0;
          std::string type_str = "unknown";
          id_ = -2;          
          for (boost::tokenizer<boost::char_separator<char> >::iterator tok_iter = tokens.begin();
               tok_iter != tokens.end(); ++tok_iter) {

               if (iter == 0) {
                    type_str = *tok_iter;
               } else if (iter == 1) {
                    id_  = syllo::str2int(std::string(*tok_iter));
               }
               iter++;
          }
          
          if (type_str == "unknown") {
               type_ = Unknown;
          } else if (type_str == "diver") {
               type_ = Diver;
          } else if (type_str == "clutter") {
               type_ = Clutter;
          } else if (type_str == "point") {
               type_ = APoint;
          } else 
               type_ = Unknown;
     } 
}
