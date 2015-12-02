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
          x_ = F_*x_ + B_*u;
          P_ = F_*P_*F_.transpose() + Q_;          
	  return 0;
     }
     
     int KalmanFilter::update(const Eigen::MatrixXf &z)
     {                    
	  K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();
	  x_ = x_ + K_*(z - H_*x_);
          P_ = (eye_ - K_*H_)*P_;
	  return 0;
     }
     
     Eigen::MatrixXf KalmanFilter::state()
     {
	  return x_;
     }
     
     Eigen::MatrixXf KalmanFilter::covariance()
     {
	  return P_;
     }

     Ellipse KalmanFilter::error_ellipse(int dim0, int dim1, double confidence)
     {
          if (confidence < 0) {
               confidence = 0;
          } else if (confidence > 1) {
               confidence = 1;
          }

          Eigen::EigenSolver<Eigen::MatrixXf> es(P_);
          cv::Point2d center = cv::Point2d(x_(0,0),x_(1,0));

          // Compute the eigenvectors and eigenvalues
          Eigen::EigenSolver< Eigen::MatrixXf >::EigenvectorsType evecs = es.eigenvectors();
          Eigen::EigenSolver< Eigen::MatrixXf >::EigenvectorsType evalues = es.eigenvalues();
          
          double q0x = evecs(dim0,0).real();
          double q0y = evecs(dim1,0).real();
          double angle = 180.0/3.14159265359 * atan2(q0y,q0x);          

          double lambda0 = evalues(dim0).real();
          double lambda1 = evalues(dim1).real();
          
          double p = confidence;
          double r0 = sqrt(-2*log(1-p/1.00)*lambda0);
          double r1 = sqrt(-2*log(1-p/1.00)*lambda1);

          return Ellipse(center, cv::Vec2d(r0,r1), angle);
     }
}

