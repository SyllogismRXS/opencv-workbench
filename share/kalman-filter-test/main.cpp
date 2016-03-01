#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <opencv_workbench/syllo/syllo.h>
//
#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/track/Dynamics.h>
#include <opencv_workbench/track/KalmanFilter.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     cout << "Kalman Filter Test" << endl;     

     double t0 = 0;
     double dt = 0.1;
     double tend = 10;     
     
     ///////////////////////////////////////////////////////
     // Setup Cart Model 
     Dynamics::state_5d_type input;
     input[0] = 5;
     input[1] = 0;//3.14159265359/20;//0
          
     Dynamics dyn;
     dyn.set_time(t0, dt, tend);
     dyn.set_model(Dynamics::cart);
     dyn.set_input(input);
     dyn.set_process_noise(0.0);
     dyn.set_measurement_noise(0.1);
     
     dyn.compute_trajectory();     
     std::vector<cv::Point2d> truth = dyn.truth_points();
     std::vector<cv::Point2d> measured = dyn.measured_points();

     ////////////////////////////////////////////////////////
     // Setup Kalman Filter
     // Kalman filter setup...
     syllo::KalmanFilter kf;
     Eigen::MatrixXf A;     // system matrix
     Eigen::MatrixXf B;     // input matrix
     Eigen::MatrixXf H;     // measurement matrix
     Eigen::MatrixXf Q;     // process noise matrix
     Eigen::MatrixXf R;     // measurement noise matrix
     Eigen::MatrixXf x0;    // initial position
     Eigen::MatrixXf covar; // initial state covariance matrix 
     
     A.resize(4,4);
     B.resize(4,2);
     H.resize(2,4);
     Q.resize(4,4);
     R.resize(2,2);
     x0.resize(4,1);
     covar.resize(4,4);
          
     double T = dt;     
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
     
     Q = Eigen::MatrixXf::Identity(A.rows(), A.cols()) * 1e-4;
     
     R << 10, 0,
          0, 10;
     
     x0 << 0,
           0,
           0,
           0;
     
     double var = 0.1;
     covar << var, 0, 0, 0,
              0, var, 0, 0,
              0, 0, var, 0,
              0, 0, 0, var;
     
     kf.setModel(A,B,H,Q,R);
     kf.init(x0, covar);

     // Setup OpenCV's kalman filter tracker
     cv::KalmanFilter opencv_kf = cv::KalmanFilter(4, 2, 0);          
     cv::Mat_<float> transition_matrix = cv::Mat_<float>(4,4);
     transition_matrix  << 1,0,T,0,   
                           0,1,0,T,  
                           0,0,1,0,  
                           0,0,0,1;
     
     opencv_kf.transitionMatrix = transition_matrix;

     opencv_kf.statePre.at<float>(0) = 0;
     opencv_kf.statePre.at<float>(1) = 0;
     opencv_kf.statePre.at<float>(2) = 2;
     opencv_kf.statePre.at<float>(3) = 0;
     cv::setIdentity(opencv_kf.measurementMatrix);
     cv::setIdentity(opencv_kf.processNoiseCov, cv::Scalar::all(1e-4));
     cv::setIdentity(opencv_kf.measurementNoiseCov, cv::Scalar::all(10));
     cv::setIdentity(opencv_kf.errorCovPost, cv::Scalar::all(.1));

     /////////////////////////////////////////////////////////
     // Process the measured points with the kalman filter
     std::vector<cv::Point2d> kf_points;
     std::vector<cv::Point2d> opencv_kf_points;
     std::vector<cv::Point2d> kf_var_x;
     std::vector<cv::Point2d> kf_var_y;
     std::vector<cv::Point2d> error;
     std::vector<cv::Point2d>::iterator it = measured.begin();
     std::vector<cv::Point2d>::iterator it_truth = truth.begin();
     double t = t0;
     for(; it != measured.end(); it++) {
          Eigen::MatrixXf u, z;
          u.resize(2,1);
          u << 0,
               0;
          
          z.resize(2,1);
          z << it->x , it->y;
          
          kf.predict(u);
          kf.update(z);

          //////////////////////////////////////////
          // Opencv
          cv::Mat prediction = opencv_kf.predict();
          cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

          // correct tracker update using the newly computed centroid.
          cv::Mat_<float> measurement(2,1);
          measurement(0) = it->x;
          measurement(1) = it->y;
          
          cv::Mat estimated = opencv_kf.correct(measurement);
          cv::Point2d est_centroid = cv::Point2d(estimated.at<float>(0),estimated.at<float>(1));
          opencv_kf_points.push_back(est_centroid);
          ////////////////////////////////////////////
                    
          Eigen::MatrixXf state = kf.state();          
          kf_points.push_back(cv::Point2d(state(0,0),state(1,0)));

          Eigen::MatrixXf covar = kf.covariance();
          kf_var_x.push_back(cv::Point2d(t,covar(0,0)));
          kf_var_y.push_back(cv::Point2d(t,covar(1,1)));

          double err = sqrt( pow(state(0,0) - it_truth->x, 2) + pow(state(1,0) - it_truth->y, 2) );
          error.push_back(cv::Point2d(t,err));
          
          it_truth++;          
          t += dt;
          
#if 0
          std::string temp;
          std::cin >> temp;
#endif
     }
    
     /////////////////////////////////////////////////////////     
     // Plot the tracks
     std::vector< std::vector<cv::Point2d> > vectors;     
     const std::string title = "Tracks";
     std::vector<std::string> labels;
     std::vector<std::string> styles;
     
     vectors.push_back(truth);
     labels.push_back("Truth");
     styles.push_back("points");
     
     vectors.push_back(measured);
     labels.push_back("Measured");
     styles.push_back("points");

     vectors.push_back(kf_points);
     labels.push_back("KF");
     styles.push_back("linespoints");

     vectors.push_back(opencv_kf_points);
     labels.push_back("OPENCV KF");
     styles.push_back("linespoints");
     
     syllo::Plot plot;
     plot.plot(vectors, title, labels, styles);
     
     ////////////////////////////////////////////////////
     vectors.clear() ; labels.clear(); styles.clear();
     const std::string title_covar = "Covariance";

     vectors.push_back(kf_var_x);
     labels.push_back("X Var");
     styles.push_back("points");
          
     vectors.push_back(kf_var_y);
     labels.push_back("Y Var");
     styles.push_back("points");

     vectors.push_back(error);
     labels.push_back("Error");
     styles.push_back("points");
     
     syllo::Plot plot_covar;
     plot_covar.plot(vectors, title_covar, labels, styles);

     std::string temp;
     std::cin >> temp;
     return 0;
}
