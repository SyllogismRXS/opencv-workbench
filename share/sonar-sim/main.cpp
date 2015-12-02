#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include <Eigen/Eigenvalues>

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
#include <opencv_workbench/track/EKF.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     cout << "================================" << endl;
     cout << "  Simple Sonar Simulator" << endl;
     cout << "================================" << endl;

     double t0 = 0;
     double dt = 0.1;
     double tend = 20;     
 
     // Setup boost RNG
     boost::mt19937 rng;
     boost::normal_distribution<> nd(0,1.0);
     boost::variate_generator<boost::mt19937&, 
                              boost::normal_distribution<> > var_nor(rng,nd);
     
     ///////////////////////////////////////////////////////
     // Setup Cart Model 
     Dynamics::state_5d_type input;
     input[0] = 5;
     input[1] = -3.14159265359/20;//0
     //input[1] = 0;
          
     Dynamics dyn;
     dyn.set_time(t0, dt, tend);
     dyn.set_model(Dynamics::cart);
     dyn.set_input(input);
     dyn.set_process_noise(0.0);
     //dyn.set_measurement_noise(0.1);
     dyn.set_measurement_noise(2);
     
     dyn.compute_trajectory();     
     std::vector<cv::Point2d> truth = dyn.truth_points();
     std::vector<cv::Point2d> measured = dyn.measured_points();
     
     std::vector< std::vector<cv::Point2d> > vectors;     
     const std::string title = "Tracks";
     std::vector<std::string> labels;
     std::vector<std::string> styles;               
     
     syllo::Plot plot;     

     std::string options;
     options = "set size ratio -1\n";
     options += "set view equal xy\n"; 
     options += "set size 1,1\n";     

     /////////////////////////////////////////////////////////
     // Process the measured points with the kalman filter     
     std::vector<cv::Point2d>::iterator it = measured.begin();
     std::vector<cv::Point2d>::iterator it_truth = truth.begin();     
     double t = t0;
     int obj_count = 1;
     std::vector<cv::Point2d> measured_disp;

     cv::Mat blank = cv::Mat::zeros(cv::Size(300,300), CV_8UC1);
     cv::imshow("control", blank);
     cv::moveWindow("control", 800,800);
     
     bool step_flag = false;
     
     for(; it != measured.end(); it++) {                             
          
          vectors.clear() ; labels.clear(); styles.clear();
          
          it_truth++;          
          t += dt;          
     
          measured_disp.push_back(*it);          
          vectors.push_back(measured_disp);
          labels.push_back("Measured");
          styles.push_back("points");              

          plot.plot(vectors, title, labels, styles, options);

          if (step_flag) {
               int key = cv::waitKey(0);    
               if (key == 'q' || key == 1048689) { // 'q' key
                    cout << "Ending early." << endl;
                    break;
               } else if (key == 'p' || key == 1048688) {
                    step_flag = 0;
               }     
          } else {
               int key = cv::waitKey(30);               
               if (key == 'q' || key == 1048689) {
                    cout << "Ending early." << endl;
                    break;
               } else if (key == 'p' || key == 1048688) {
                    step_flag = 1;
               } 
          }                    
     }         

     /////////////////////////////////////////////////////////     
     // Plot the tracks
     vectors.clear() ; labels.clear(); styles.clear();          
     
     vectors.push_back(truth);
     labels.push_back("Truth");
     styles.push_back("points");
     
     vectors.push_back(measured);
     labels.push_back("Measured");
     styles.push_back("points");
     
     //std::string options;
     //options = "set size ratio -1\n";
     //options += "set view equal xy\n"; 
     //options += "set size 1,1\n";
     
     syllo::Plot plot_end;
     plot_end.plot(vectors, title, labels, styles, options);
     
     cv::waitKey(0);     
     
     return 0;
}
