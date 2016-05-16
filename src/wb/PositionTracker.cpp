#include <iostream>

#include "PositionTracker.h"

using std::cout;
using std::endl;

PositionTracker::PositionTracker() : initialized_(false)
{  
     A_.resize(4,4);     // State transition
     B_.resize(4,2);     // Input matrix
     H_.resize(2,4);     // Measurement matrix
     Q_.resize(4,4);     // Process noise
     R_.resize(2,2);     // Measurement noise
     x0_.resize(4,1);    // Initial state
     P_.resize(4,4);    // Covariance matrix
     
     double T = 0.066666667;//; dt
     //double T = 1;//; dt
     //double T = 0.05;
     A_ << 1, 0, T, 0,
          0, 1, 0, T,
          0, 0, 1, 0,
          0, 0, 0, 1;

     // Inputs are from velocity
     B_ << 0, 0,
          0, 0,
          1, 0,
          0, 1;

     // Measurements are from x/y positions
     H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

     double q = 1 * T;
     Q_ = Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * q;

     double r = 10;
     R_ << r, 0,
          0, r;               

     z_.resize(2,1);
     z_ << 0,0;
     
     u_.resize(2,1);
     u_ << 0, 0;     

     P_ << Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * 10;
     
     kf_.setModel(A_, B_, H_, Q_, R_);
}

Eigen::MatrixXf PositionTracker::R()
{
     return kf_.R();
}

void PositionTracker::set_Q(double q)
{
     Q_ = Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * q;
     kf_.setModel(A_, B_, H_, Q_, R_);
}
void PositionTracker::set_R(double r)
{
     R_ << r, 0,
          0, r;

     kf_.setModel(A_, B_, H_, Q_, R_);
}

void PositionTracker::set_R(double r0, double r1, double r2, double r3)
{
     R_ << r0, r1,
          r2, r3;

     kf_.setModel(A_, B_, H_, Q_, R_);
}

void PositionTracker::set_P(double p)
{
     P_ << Eigen::MatrixXf::Identity(A_.rows(), A_.cols()) * p;
     kf_.init(kf_.state(), P_);
}

void PositionTracker::set_measurement(cv::Point2d m)
{
     if (initialized_) {          
          z_ << m.x , m.y;
          kf_.update(z_);
     } else {
          initialized_ = true;
          x0_ << m.x , m.y, 0, 0;          
          kf_.init(x0_,P_);
     }
}

void PositionTracker::predict()
{
     if (initialized_) {
          kf_.predict(u_);
     }
}

cv::Point2d PositionTracker::position()
{
     Eigen::MatrixXf s = kf_.state();
     cv::Point2d p(s(0,0), s(1,0));
     return p;
}

cv::Point2d PositionTracker::velocity()
{
     Eigen::MatrixXf s = kf_.state();
     cv::Point2d v(s(2,0), s(3,0));     
     return v;
}

Ellipse PositionTracker::error_ellipse(double confidence)
{
     return kf_.error_ellipse(confidence);
}

bool PositionTracker::is_within_region(cv::Point2d z, double confidence)
{
     z_ << z.x , z.y;
     return kf_.is_within_region(z_, confidence);
}

Eigen::MatrixXd PositionTracker::meas_covariance()
{
     return kf_.meas_covariance().cast<double>();
}

void PositionTracker::set_position(cv::Point2d p)
{
     Eigen::MatrixXf x(4,1);
     x << p.x, p.y, 0, 0;
     kf_.set_state(x);
}

void PositionTracker::print()
{
     kf_.print();
}
