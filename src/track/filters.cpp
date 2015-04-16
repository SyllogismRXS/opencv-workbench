#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>
//using namespace Eigen;

//#include <boost/array.hpp>
//#include <boost/numeric/odeint.hpp>

//using namespace boost::numeric;
//
//#include "filters.h"
//
//namespace syllo {
//
//     Kalman::Kalman()
//     {
//     }
//
//     Kalman::Kalman(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::VectorXf &C, const Eigen::MatrixXf &R, const Eigen::MatrixXf &Q)
//     {
//	  setModel(A, B, C, R, Q);
//     }
//
//     int Kalman::setModel(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::VectorXf &C, const Eigen::MatrixXf &R, const Eigen::MatrixXf &Q)
//     {
//	  this->A = A;
//	  this->B = B;
//	  this->C = C;
//	  this->R = R;
//	  this->Q = Q;
//	  eye = Eigen::MatrixXf::Identity(A.rows(), A.cols());
//
//	  return 0;
//     }
//
//     int Kalman::init(const Eigen::VectorXf &mu0, const Eigen::MatrixXf &covar0)
//     {
//	  this->mu = mu0;
//	  this->covar = covar0;
//	  return 0;
//     }
//
//     int Kalman::step(Eigen::VectorXf &mu_prev, Eigen::MatrixXf &covar_prev, const Eigen::VectorXf &u)
//     {
//	  Eigen::VectorXf mu_dx;
//
//	  mu_dx = A*mu_prev + B*u;
//
//	  mu_prev += mu_dx;
//
//	  covar = A*covar_prev*A.transpose() + R;
//	  //K = covar*C.transpose() * (C*covar*C.transpose() + Q).inverse();
//	  ////mu = mu + K*(z-C*mu);
//	  //covar = (eye - K*C) * covar;
//	  
//	  
//
//	  return 0;
//     }
//
//     int Kalman::step(Eigen::VectorXf &mu_prev, Eigen::MatrixXf &covar_prev, const Eigen::VectorXf &u, const Eigen::VectorXf &z)
//     {
//	  mu = A*mu_prev + B*u;
//	  covar = A*covar_prev*A.transpose() + R;
//
//	  std::cout << "covar: \n" << covar << std::endl;
//	  std::cout << "C: \n" << C << std::endl;
//	  std::cout << "C': \n" << C.transpose() << std::endl;
//	  std::cout << "Q: \n" << Q << std::endl;
//
//	  //K = covar*C.transpose() * (C*covar*C.transpose() + Q).inverse();
//	  
//          K = covar*C * (C.transpose()*covar*C + Q).inverse();
//	  
//	  std::cout << "mu: \n" <<  mu << std::endl;
//	  std::cout << "K: \n" <<  K << std::endl;
//	  std::cout << "z: \n" <<  z << std::endl;
//	  std::cout << "C: \n" <<  C << std::endl;
//
//	  //mu = mu + K*(z-C*mu);
//	  //mu = mu + K*(z-C*mu);
//	  z-C.transpose()*mu;
//	  
//          //covar = (eye - K*C) * covar;
//	  
//	  return 0;
//     }
//
//     Eigen::VectorXf Kalman::getMu()
//     {
//	  return mu;
//     }
//     
//     Eigen::MatrixXf Kalman::getCovar()
//     {
//	  return covar;
//     }
//}
