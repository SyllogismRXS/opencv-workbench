#include <iostream>

#include "Entity.h"

using std::cout;
using std::endl;

namespace wb {

     Entity::Entity() : age_(0), occluded_(false)
     {
          KF_ = cv::KalmanFilter(4, 2, 0);          
          transition_matrix_ = cv::Mat_<float>(4,4);
          transition_matrix_  << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1;
          KF_.transitionMatrix = transition_matrix_;     
     }

     void Entity::init()
     {
          compute_metrics();
          
          KF_.statePre.at<float>(0) = centroid_.x;
          KF_.statePre.at<float>(1) = centroid_.y;
          KF_.statePre.at<float>(2) = 0;
          KF_.statePre.at<float>(3) = 0;
          cv::setIdentity(KF_.measurementMatrix);
          cv::setIdentity(KF_.processNoiseCov, cv::Scalar::all(1e-4));
          cv::setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(10));
          cv::setIdentity(KF_.errorCovPost, cv::Scalar::all(.1));
     }

     void Entity::predict_tracker()
     {
          cv::Mat prediction = KF_.predict();
          cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
          
          //ekf_.predict();
     }

     void Entity::correct_tracker()
     {
          // correct tracker update using the newly computed centroid.
          cv::Mat_<float> measurement(2,1);
          measurement(0) = centroid_.x;
          measurement(1) = centroid_.y;
          
          cv::Mat estimated = KF_.correct(measurement);
          est_centroid_ = cv::Point(estimated.at<float>(0),estimated.at<float>(1));
     }          

     void Entity::compute_metrics()
     {
          ////////////////////////////////////////////////////////
          // Compute Centroid and Bounding Box Rectangle
          ////////////////////////////////////////////////////////
          int sum_x = 0, sum_y = 0;
          int xmin = 999999999, ymin = 999999999, xmax = -999, ymax = -999;
          std::vector<Point>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               int x = it->position().x;
               int y = it->position().y;
               
               sum_x += x;
               sum_y += y;

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
          
          double avg_x = (double)sum_x / points_.size();
          double avg_y = (double)sum_y / points_.size();

          centroid_ = cv::Point(round(avg_x), round(avg_y));
          
          // +1's to account for slight error in rectangle drawing?
          rectangle_ = cv::Rect(xmin, ymin, xmax-xmin+1, ymax-ymin+1);
     }

     cv::Point Entity::centroid()
     {
          return centroid_;
     }

     cv::Rect Entity::rectangle()
     {
          return rectangle_;
     }

     bool Entity::is_visible()
     {
          if (age_ > 5) {
               return true;
          } else {
               return false;
          }
     }

     bool Entity::is_dead()
     {
          if (age_ < -2) {
               return true;
          } else {
               return false;
          }
     }

     void Entity::inc_age()
     {
          age_++;
          if (age_ > 10) {
               age_ = 10;
          }
     }

     void Entity::dec_age()
     {
          age_--;
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

     
}
