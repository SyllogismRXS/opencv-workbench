#ifndef _FILTERS_H_
#define _FILTERS_H_

#include <Eigen/Dense>

namespace syllo {

     //class Kalman {
     //protected:
     //	  Eigen::MatrixXf A;
     //	  Eigen::MatrixXf B;
     //	  Eigen::VectorXf C;
     //	  
     //	  Eigen::MatrixXf R;
     //	  Eigen::MatrixXf Q;
     //	  
     //	  // Matrices / vectors for Kalman step
     //	  Eigen::VectorXf mu;
     //	  Eigen::MatrixXf covar;
     //	  Eigen::VectorXf K; // Kalman gain
     //	  Eigen::MatrixXf eye;// = Eigen::Matrix<float, 2, 2>::Identity();
     //
     //public:
     //	  Kalman();
     //	  Kalman(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::VectorXf &C, const Eigen::MatrixXf &R, const Eigen::MatrixXf &Q);
     //	  
     //	  int setModel(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::VectorXf &C, const Eigen::MatrixXf &R, const Eigen::MatrixXf &Q);
     //	  int init(const Eigen::VectorXf &mu0, const Eigen::MatrixXf &covar0);
     //	  int step(Eigen::VectorXf &mu_prev, Eigen::MatrixXf &covar_prev, const Eigen::VectorXf &u);
     //	  int step(Eigen::VectorXf &mu_prev, Eigen::MatrixXf &covar_prev, const Eigen::VectorXf &u, const Eigen::VectorXf &z);
     //	  
     //	  Eigen::VectorXf getMu();
     //	  Eigen::MatrixXf getCovar();
     //};
     //
     //int eigenTest();
}

#endif
