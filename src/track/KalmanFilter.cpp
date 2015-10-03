#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric;

using std::cout;
using std::endl;

#include "KalmanFilter.h"

namespace syllo {

     KalmanFilter::KalmanFilter()
     {
     }

     KalmanFilter::KalmanFilter(const Eigen::MatrixXf &F, 
				const Eigen::MatrixXf &B, 
				const Eigen::MatrixXf &H, 
				const Eigen::MatrixXf &Q, 
				const Eigen::MatrixXf &R)
     {
	  setModel(F, B, H, Q, R);
     }

     int KalmanFilter::setModel(const Eigen::MatrixXf &F, 
				const Eigen::MatrixXf &B, 
				const Eigen::MatrixXf &H, 
				const Eigen::MatrixXf &Q, 
				const Eigen::MatrixXf &R)
     {
	  F_ = F;
	  B_ = B;
	  H_ = H;
          //V_ = V;
          //W_ = W;
          //Q_ = V_ * V_.transpose();
	  //R_ = W_ * W_.transpose();	  
          Q_ = Q;
          R_ = R;
	  eye_ = Eigen::MatrixXf::Identity(F.rows(), F.cols());

	  return 0;
     }

     int KalmanFilter::init(const Eigen::MatrixXf &x0, 
			    const Eigen::MatrixXf &P0)
     {
	  x_ = x0;
	  P_ = P0;
	  return 0;
     }

     int KalmanFilter::predict(const Eigen::MatrixXf &u)
     {
          //x_ = F_*x_ + B_*u;
          x_ = F_*x_;
          P_ = F_*P_*F_.transpose() + Q_;          
	  return 0;
     }
     
     int KalmanFilter::update(const Eigen::MatrixXf &z)
     {
          cout << "P = " << P_ << endl;
          cout << "H = " << H_ << endl;
          cout << "R = " << R_ << endl;
          
	  K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();

          cout << "K_: " << K_ << endl;
          
	  x_ = x_ + K_*(z - H_*x_);
          P_ = (eye_ - K_*H_)*P_;
	  return 0;
     }
     
     Eigen::MatrixXf KalmanFilter::state()
     {
	  return x_;
     }
     
     Eigen::MatrixXf KalmanFilter::variance()
     {
	  return P_;
     }
}

