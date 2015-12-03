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
#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/track/Dynamics.h>
#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/track/EKF.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using std::cout;
using std::endl;

#define PI 3.14159265359

cv::Point2d rotate_2D(cv::Point2d v, double rad)
{
     cv::Point2d ans;
     ans.x = v.x*cos(rad) - v.y*sin(rad);
     ans.y = v.x*sin(rad) + v.y*cos(rad);
     return ans;
}

int main(int argc, char *argv[])
{
     cout << "================================" << endl;
     cout << "  Simple Sonar Simulator" << endl;
     cout << "================================" << endl;

     double t0 = 0;
     double dt = 0.05;
     double tend = 10;

     ///////////////////////////////////////////////////////
     // Setup Sonar Sensor Motion Model
     Dynamics::state_5d_type sonar_input;
     //sonar_input[0] = 1;
     //sonar_input[1] = PI/10;//0;
     sonar_input[0] = -0.1; // right
     sonar_input[1] = 0.1; // left
          
     Dynamics::state_5d_type sonar_x0;
     sonar_x0[0] = 0; //x
     sonar_x0[1] = 0; //y
     sonar_x0[2] = 22.5*PI/180; //heading
     
     Dynamics sonar_dyn;
     sonar_dyn.set_x0(sonar_x0);
     sonar_dyn.set_time(t0, dt, tend);
     sonar_dyn.set_model(Dynamics::roomba);
     sonar_dyn.set_input(sonar_input);
     sonar_dyn.set_process_noise(0.0);
     sonar_dyn.set_measurement_noise(0);
          
     sonar_dyn.compute_trajectory();     
     std::vector<cv::Point2d> sonar_truth = sonar_dyn.truth_points();
     std::vector<cv::Point2d> sonar_measured = sonar_dyn.measured_points();
     std::vector<Dynamics::state_5d_type> sonar_state = sonar_dyn.state();
     std::vector<Dynamics::state_5d_type>::iterator it_sonar_state = sonar_state.begin();
     std::vector<cv::Point2d>::iterator it_sonar_measured = sonar_truth.begin();
          
     ///////////////////////////////////////////////////////
     // Setup Cart Model 
     Dynamics::state_5d_type input;
     input[0] = 0;
     input[1] = 0;
     //input[0] = 5;
     //input[1] = 0;//-PI/20;//0
     //input[1] = 0;

     Dynamics::state_5d_type x0;
     x0[0] = 15; //x
     x0[1] = 0;  //y
     x0[2] = 0;  //heading
     //x0[0] = 10;
     //x0[1] = 40;
     //x0[2] = -PI/2;
          
     Dynamics dyn;
     dyn.set_x0(x0);
     dyn.set_time(t0, dt, tend);
     dyn.set_model(Dynamics::cart);
     dyn.set_input(input);
     dyn.set_process_noise(0.0);
     dyn.set_measurement_noise(0.0);
     //dyn.set_measurement_noise(2);
     
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
     std::vector<cv::Point2d> measured_disp;

     std::vector<cv::Point2d> sonar_disp;

     cv::Mat blank = cv::Mat::zeros(cv::Size(300,300), CV_8UC1);
     cv::imshow("control", blank);
     cv::moveWindow("control", 800,800);
     
     bool step_flag = true;
     
     syllo::Plot plot_frame;

     // Sonar image generation
     double max_angle_ = 0.392699082;
     double min_angle_ = -0.392699082;
     double x_min = 0.001;  // R_min
     double x_max = 20.0; // R_max
     //double y_min = 0;    // 
     double y_max = sin(max_angle_)*x_max*2;
     double beam_width_ = max_angle_ * 2;
     
     int img_width = 600;
     int img_height = 600;
     cv::Mat img = cv::Mat::zeros(img_height, img_width, CV_8UC3);
     img = cv::Scalar(255,255,255);

     // Rotate the sonar image to pointing "up"
     cv::Point center = cv::Point( img.cols/2, img.rows/2 );
     double angle = 180.0;
     double rot_scale = 1.0;
     
     cv::Mat rot_mat(2,3,CV_8UC1);
     rot_mat = cv::getRotationMatrix2D( center, angle, rot_scale );
     
     for(; it != measured.end(); it++) {                             
          //img = cv::Scalar(255,255,255); // if we want to clear it          
          
          cv::warpAffine(img, img, rot_mat, img.size());
          vectors.clear() ; labels.clear(); styles.clear();          
     
          /////////////////////////////////////////////////
          // Can the sonar see the object at this point?
          // Within range and bearing limits?          
          // Compute bearing from sonar to target          

          // Position of contact, relative to ownship
          cv::Point2d offset = *it - cv::Point2d((*it_sonar_state)[0], (*it_sonar_state)[1]);          
          cv::Point2d cnt_relative_pos = rotate_2D(offset, -(*it_sonar_state)[2]);
          double bearing_to_cnt = atan2(cnt_relative_pos.y, 
                                        cnt_relative_pos.x);

          double dist = cv::norm(offset);
          if ((min_angle_ <= bearing_to_cnt) && (bearing_to_cnt <= max_angle_) && 
              (dist <= x_max && dist >= x_min)) {
               // Put target in sonar's image

               cout << "Bearing: " << bearing_to_cnt * 180 / PI;
               
               // convert offset to polar coordinates
               double R = dist;
               double theta = bearing_to_cnt;
               
               // x = R*cos(theta), y = R*sin(theta);
               double x = R*cos(theta);
               double y = R*sin(theta);
                    
               //int x_pos = img_width/2 + offset.y / (y_max/2) * img_height*sin(max_angle_);
               //int y_pos = offset.x / x_max * img_height;
               int x_pos = cvRound(img_width/2 + y / (y_max/2) * img_height*sin(max_angle_));
               int y_pos = cvRound(x / x_max * img_height);
                                                 
               if (x_pos < 0 || x_pos > img.cols || y_pos < 0 || y_pos > img.rows) {
                    // error, skip
                    cout << "bounds" << endl;
                    continue;
               }                         
               cv::circle(img,cv::Point(x_pos, y_pos),1,cv::Scalar(0,0,0),-1,8,0);
          }          
          ///////////////////////////////////////////////////          
          //cv::Mat img_color;
          //Gray2Jet_matlab(img, img_color);
          cv::Mat img_color = img;
          
          // Draw the sonar's outside polygon
          double start_angle = (PI - beam_width_) / 2 ;
          double end_angle = start_angle + beam_width_;// * PI / 180.0;//90.0 * PI / 180.0;//1.963495408;
          cv::Point start(img_width/2,0);
          cv::Point end_1(start.x+img_height*cos(start_angle), start.y+img_height*sin(start_angle));
          cv::Point end_2(start.x+img_height*cos(end_angle), start.y+img_height*sin(end_angle));
                    
          // Line from sonar head to start of outer-arc
          cv::line(img_color, start, end_1, cv::Scalar(0,0,0), 1, 8, 0);
          cv::Point prev = end_1;
          
          // Draw outer-arc polygon
          double num_pts = 25;
          double step = (end_angle - start_angle) / num_pts;
          for (double ang = start_angle; ang <= end_angle; ang += step) {
               //contour.push_back(cv::Point(start.x+img_height*cos(ang), start.y+img_height*sin(ang)));
               cv::Point p(cv::Point(start.x+img_height*cos(ang), start.y+img_height*sin(ang)));
               cv::line(img_color, prev, p, cv::Scalar(0,0,0), 1, 8, 0);
               prev = p;
          }

          // Line from outer-arc to real final arc position
          cv::line(img_color, prev, end_2, cv::Scalar(0,0,0), 1, 8, 0);

          // Line from outer-arc back to sonar head
          cv::line(img_color, end_2, start, cv::Scalar(0,0,0), 1, 8, 0);                   

          // Rotate image and display
          cv::warpAffine(img_color, img_color, rot_mat, img_color.size());
          cv::imshow("Sonar", img_color);     
          ///////////////////////////////////////////////////
          
          measured_disp.push_back(*it);       
          sonar_disp.push_back(*it_sonar_measured);
          
          vectors.push_back(measured_disp);
          labels.push_back("Target-M");
          styles.push_back("points");              

          vectors.push_back(sonar_disp);
          labels.push_back("Sonar-M");
          styles.push_back("points");              
          
          plot.plot(vectors, title, labels, styles, options);

          // Frame plot          
          vectors.clear() ; labels.clear(); styles.clear();
          std::vector<cv::Point2d> frame;
          frame.push_back(*it);
          labels.push_back("Measured");
          styles.push_back("points");                        
          vectors.push_back(frame);
          plot_frame.plot(vectors, title, labels, styles);
          
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

          it_sonar_measured++;
          it_sonar_state++;
          it_truth++;          
          t += dt;
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
     
     syllo::Plot plot_end;
     plot_end.plot(vectors, title, labels, styles, options);
     
     cv::waitKey(0);     
     
     return 0;
}
