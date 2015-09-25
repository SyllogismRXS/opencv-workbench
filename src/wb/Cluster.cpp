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
          
          transition_matrix_ = cv::Mat_<float>(4,4);
          transition_matrix_  << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1;
          KF_.transitionMatrix = transition_matrix_;
     }
     
     void Cluster::init()
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
}
