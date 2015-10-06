#include <iostream>

#include "Entity.h"

using std::cout;
using std::endl;

#define MAX_AGE 10
#define VISIBLE_AGE 5
#define MIN_AGE -2

namespace wb {

     Entity::Entity() : age_(0), occluded_(false)
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
     
          covar << 0.1, 0, 0, 0,
               0, 0.1, 0, 0,
               0, 0, 0.1, 0,
               0, 0, 0, 0.1;
     
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

          kf_.init(x0, covar);
     }

     cv::Point Entity::estimated_centroid()
     {          
          // If the entity is visible, it has a valid track; thus, we can use
          // the Kalman filter's estimate centroid. Otherwise, use the entity's
          // current centroid
          if (this->is_visible()) {
               return est_centroid_;
          } else {
               return centroid_;
          }          
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
          if (age_ > VISIBLE_AGE) {
               return true;
          } else {
               return false;
          }
     }

     bool Entity::is_dead()
     {
          if (age_ < MIN_AGE) {
               return true;
          } else {
               return false;
          }
     }

     void Entity::inc_age()
     {
          age_++;
          if (age_ > MAX_AGE) {
               age_ = MAX_AGE;
          }
     }

     void Entity::set_age(int age)
     {
          age_ = age;
          
          if (age_ > MAX_AGE) {
               age_ = MAX_AGE;
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
