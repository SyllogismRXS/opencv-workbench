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
//
//#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/syllo.h>
//
#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/track/Dynamics.h>
#include <opencv_workbench/track/KalmanFilter.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     cout << "Kalman Filter Test" << endl;     

     double t0 = 0;
     double dt = 0.1;
     double tend = 5;
     
     ///////////////////////////////////////////////////////
     // Setup Cart Model 
     Dynamics::state_5d_type input;
     input[0] = 5;
     input[1] = 0;//3.14159265359/20;     
          
     Dynamics dyn;
     dyn.set_time(t0, dt, tend);
     dyn.set_model(Dynamics::cart);
     dyn.set_input(input);
     dyn.set_process_noise(0.0);
     dyn.set_measurement_noise(0.1);
     
     dyn.compute_trajectory();     
     std::vector<cv::Point2f> truth = dyn.truth_points();
     std::vector<cv::Point2f> measured = dyn.measured_points();
          

     ////////////////////////////////////////////////////////
     // Setup Kalman Filter
     // Kalman filter setup...
     syllo::KalmanFilter kf;
     Eigen::MatrixXf A;     // system matrix
     Eigen::MatrixXf B;     // input matrix
     Eigen::MatrixXf H;     // measurement matrix
     Eigen::MatrixXf Q;     // 
     Eigen::MatrixXf R;     // 
     Eigen::MatrixXf x0;    // initial position
     Eigen::MatrixXf covar; // initial state covariance matrix 
     Eigen::MatrixXf u;     // input vector     

     A.resize(4,4);
     ////B.resize();
     H.resize(2,4);
     Q.resize(4,4);
     R.resize(2,2);
     x0.resize(4,1);
     covar.resize(4,4);
     //u.resize(2);
     
     double T = dt;     
     A << 1, 0, T, 0,
          0, 1, 0, T,
          0, 0, 1, 0,
          0, 0, 0, 1;
     
     //B << 1,0,0,1;
     
     H << 1, 0, 0, 0,
          0, 1, 0, 0;
     
     Q << T/7, T/6, T/5, T/4,
          T/6, T/5, T/4, T/3,
          T/5, T/4, T/3, T/2,
          T/4, T/3, T/2, T/1;
     
     R << T, T,
          T, T;
     
     x0 << 0,
           0,
           2,
           0;
     
     covar << 1000, 0, 0, 0,
              1000, 0, 0, 0,
              1000, 0, 0, 0,
              1000, 0, 0, 0;
     
     kf.setModel(A,B,H,Q,R);
     kf.init(x0, covar);

     /////////////////////////////////////////////////////////
     // Process the measured points with the kalman filter
     std::vector<cv::Point2f> kf_points;
     std::vector<cv::Point2f>::iterator it = measured.begin();
     for(; it != measured.end(); it++) {
          cout << "---------------------------------" << endl;

          Eigen::MatrixXf u, z;
          u.resize(2,1);
          u << 1,0;
          
          z.resize(2,1);
          z << it->x , it->y;
          
          kf.predict(u);
          kf.update(z);
                    
          Eigen::MatrixXf state = kf.state();          
          cout << "(" << state(0,0) << ", " << state(1,0) << endl;
          kf_points.push_back(cv::Point2f(state(0,0),state(1,0)));
          
#if 0
          std::string temp;
          std::cin >> temp;
#endif
     }
    
     /////////////////////////////////////////////////////////     
     // Plot the tracks
     std::vector< std::vector<cv::Point2f> > vectors;     
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
     
     syllo::Plot plot;
     plot.plot(vectors, title, labels, styles);
          
     return 0;
}
