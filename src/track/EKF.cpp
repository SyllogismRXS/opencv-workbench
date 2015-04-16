#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric;

using std::cout;
using std::endl;

#include "MotionModels.h"
#include "EKF.h"

namespace syllo {

     EKF::EKF()
     {
     }

     EKF::EKF(const Eigen::MatrixXf &F, 
	      const Eigen::MatrixXf &B, 
	      const Eigen::MatrixXf &H, 
	      const Eigen::MatrixXf &R, 
	      const Eigen::MatrixXf &Q,
	      double dt)
     {
	  setModel(F, B, H, R, Q, dt);
     }

     int EKF::setModel(const Eigen::MatrixXf &F, 
		       const Eigen::MatrixXf &B, 
		       const Eigen::MatrixXf &H, 
		       const Eigen::MatrixXf &R, 
		       const Eigen::MatrixXf &Q,
		       double dt)
     {
	  F_ = F;
	  B_ = B;
	  H_ = H;
	  R_ = R;
	  Q_ = Q;
	  eye_ = Eigen::MatrixXf::Identity(F.rows(), F.cols());
	  dt_ = dt;

	  return 0;
     }

     int EKF::init(const Eigen::MatrixXf &x0, 
		   const Eigen::MatrixXf &P0)
     {
	  x_ = x0;
	  P_ = P0;
	  return 0;
     }

     int EKF::predict(const Eigen::MatrixXf &u)
     {
	  runge_kutta4< syllo::state_3d_type > stepper;
	  syllo::state_3d_type x = {x_(0,0), x_(1,0), x_(2,0)};
	  stepper.do_step(cart_model, x , 0 , dt_ );

	  x_ << x[0], x[1], x[2];

	  ////x_ = F_*x_ + B_*u;
	  //Eigen::MatrixXf xdot;
	  //xdot.resize(3,1);
	  //
	  //xdot << u(0,0)   * cos(x_(2,0)), 
	  //        u(0,0)   * sin(x_(2,0)), 
   	  //        u(0,0)/3 * tan(u(1,0));
	  //
	  //x_ =  x_ + xdot;
	  
	  // compute jacobian...
	  Eigen::MatrixXf J;
	  J.resize(3,3);
	  J << 0, 0, -u(0,0)*sin(x_(2,0)),
	       0, 0, u(0,0)*cos(x_(2,0)),
	       0, 0, 0;

	  //J << 0, 0, 0,
	  //     0, 0, 0,
	  //     0, 0, 0;
	  
	  //P_ = F_*P_*F_.transpose() + Q_;
	  P_ = J*P_*J.transpose() + Q_;
	  return 0;
     }
     
     int EKF::update(const Eigen::MatrixXf &z)
     {
	  K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();
	  x_ = x_ + K_*(z - H_*x_);
	  P_ = (eye_ - K_*H_)*P_;
	  return 0;
     }
     
     Eigen::MatrixXf EKF::state()
     {
	  return x_;
     }
     
     Eigen::MatrixXf EKF::variance()
     {
	  return P_;
     }
}

