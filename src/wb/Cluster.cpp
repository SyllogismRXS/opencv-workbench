#include <iostream>

#include "Cluster.h"

using std::cout;
using std::endl;

namespace wb {

     static inline cv::Point calcPoint(cv::Point2f center, double R, double angle)
     {
          return center + cv::Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
     }
     
     Cluster::Cluster()
     {
          age_ = 0;
          distance_ = 999999;
          matched_ = false;
          match_ = NULL;
          
          //A.resize(2,2);
          //B.resize(2,2);
          //C.resize(2);
          //R.resize(2,2);
          //Q.resize(2,2);
          //mu.resize(2);
          //covar.resize(2,2);
          //u.resize(2);
          //
          //estVel.resize(2);
          //
          //A << 0,0,0,0;
          //B << 1,0,0,1;
          //C << 1,1;
          //R << 0.1,0.1,0.1,0.1;
          //Q << 0.1,0.1,0.1,0.1;
	  //     
          //mu << 0,1;
          //covar << 0.1,0.1,0.1,0.1;
          //u << 1,1;
          //
          //estVel << 0,0;
          //
          //ekf_.setModel(A,B,C,R,Q,0.066666667);
          
          KF_ = cv::KalmanFilter(4, 2, 0);
          
          KF_.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
     }
     
     void Cluster::init()
     {
          age_ = 1;

          this->compute_metrics();

          KF_.statePre.at<float>(0) = centroid_.x;
          KF_.statePre.at<float>(1) = centroid_.y;
          KF_.statePre.at<float>(2) = 0;
          KF_.statePre.at<float>(3) = 0;
          cv::setIdentity(KF_.measurementMatrix);
          cv::setIdentity(KF_.processNoiseCov, cv::Scalar::all(1e-4));
          cv::setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(10));
          cv::setIdentity(KF_.errorCovPost, cv::Scalar::all(.1));

          //// Setup EKF model
          //Eigen::MatrixXf x0;
          //Eigen::MatrixXf P0;
          //
          //x0.resize(2,1);
          //P0.resize(2,2);
          //
          //x0 << centroid_.x , centroid_.y;
          //P0 << 0 , 0, 0, 0;
          //
          //ekf_.init(x0,P0);
     }

     void Cluster::predict_tracker()
     {
          cv::Mat prediction = KF_.predict();
          cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
          
          //ekf_.predict();
     }

     void Cluster::correct_tracker()
     {
          // correct tracker update using the newly computed centroid.
          cv::Mat_<float> measurement(2,1);
          measurement(0) = centroid_.x;
          measurement(1) = centroid_.y;
          
          cv::Mat estimated = KF_.correct(measurement);
          est_centroid_ = cv::Point(estimated.at<float>(0),estimated.at<float>(1));
     }

     bool Cluster::is_visible()
     {
          if (age_ > 10) {
               return true;
          } else {
               return false;
          }
     }

     bool Cluster::is_dead()
     {
          if (age_ < -2) {
               return true;
          } else {
               return false;
          }
     }

     void Cluster::inc_age()
     {
          age_++;
          if (age_ > 20) {
               age_ = 20;
          }
     }

     void Cluster::dec_age()
     {
          age_--;
     }

     void Cluster::remove_point(wb::Point &p)
     {          
          std::vector<Point*>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               if ((*it)->position() == p.position()) {
                    points_.erase(it);
                    break;
               }
          }
     }
     
     void Cluster::compute_metrics()
     {
          ////////////////////////////////////////////////////////
          // Compute Centroid and Bounding Box Rectangle
          ////////////////////////////////////////////////////////
          int sum_x = 0, sum_y = 0;
          int xmin = 999999999, ymin = 999999999, xmax = -999, ymax = -999;
          std::vector<Point*>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               int x = (*it)->position().x;
               int y = (*it)->position().y;
               
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

     cv::Point Cluster::centroid()
     {
          return centroid_;
     }

     cv::Rect Cluster::rectangle()
     {
          return rectangle_;
     }

}
