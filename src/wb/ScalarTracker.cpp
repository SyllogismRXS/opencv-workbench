#include <iostream>
#include "ScalarTracker.h"

using std::cout;
using std::endl;

ScalarTracker::ScalarTracker() : initialized_(false)
{
     Eigen::MatrixXf F(1,1); // System dynamics
     Eigen::MatrixXf B(1,1); // Control matrix
     Eigen::MatrixXf H(1,1); // Measurement matrix
	  
     Eigen::MatrixXf Q(1,1); // Process variance     ; V*V'
     Eigen::MatrixXf R(1,1); // Measurement variance ; W*W'
	  	  
     F << 1;
     B << 0;
     H << 1;
     Q << 0.01;
     R << 2;

     x0_.resize(1,1);
     P0_.resize(1,1);
     z_.resize(1,1);
     u_.resize(1,1);
     u_ << 0;
     
     kf_.setModel(F,B,H,Q,R);     
}

void ScalarTracker::set_value(float value)
{
     if (initialized_) {          
          z_ << value;
          kf_.update(z_);
     } else {
          initialized_ = true;
          x0_ << value;
          P0_ << 10;
          kf_.init(x0_,P0_);
     }
}

void ScalarTracker::predict()
{          
     if (initialized_) {
          kf_.predict(u_);
     }
}

float ScalarTracker::value()
{
     return kf_.state()(0,0);
}

float ScalarTracker::lower_end(float num_stds)
{
     float mean = this->value();
     Eigen::MatrixXf P = kf_.covariance();
     float var = P(0,0);

     return mean - (sqrt(var) * num_stds);
}

