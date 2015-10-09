#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

//// OpenCV headers
//#include <cv.h>
//#include <highgui.h>
//
//#include <opencv2/contrib/contrib.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/syllo.h>
//
#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/wb/Entity.h>
#include <opencv_workbench/track/Dynamics.h>

using std::cout;
using std::endl;

void trajectory_diff(std::vector<wb::Entity> &traj, 
                     std::vector<cv::Point2f> &diffs)
{
     std::vector<wb::Entity>::reverse_iterator it = traj.rbegin();
     cv::Point prev;
     for(; it != traj.rend(); it++) {
          if (it == traj.rbegin()) {
               prev = it->centroid();
               continue;
          }

          // Compute dx and dy
          diffs.push_back(cv::Point2f(it->centroid().x - prev.x, 
                                      it->centroid().y - prev.y));
               
          prev = it->centroid();
     }
}

int main(int argc, char *argv[])
{
     cout << "Trajectory Similarity" << endl;                    
     
     AnnotationParser parser;
     std::vector<wb::Entity> t1, t2;
     
     std::string t1_str = "t1";
     std::string t2_str = "t2";

     std::string tracks_file_name = "";     
     if (argc >= 4) {
          tracks_file_name = argv[1];
          t1_str = argv[2];
          t2_str = argv[3];
     }

     if (tracks_file_name == "") {    
          ///////////////////////////////////////////////////////
          // Generate the two trajectories
          ///////////////////////////////////////////////////////
          // Setup Cart Model 1
          Dynamics::state_5d_type input, x0;
          input[0] = 5;
          input[1] = 3.14159265359/20;//0

          x0[0] = 10;
          x0[1] = 10;
          x0[2] = 0;
     
          double t0 = 0;
          double dt = 0.1;
          double tend = 10;
     
          Dynamics dyn1;
          dyn1.set_time(t0, dt, tend);
          dyn1.set_model(Dynamics::cart);
          dyn1.set_x0(x0);
          dyn1.set_input(input);
          dyn1.set_process_noise(0.0);
          dyn1.set_measurement_noise(0.1);
     
          dyn1.compute_trajectory();     
          std::vector<cv::Point2f> truth_1 = dyn1.truth_points();
          std::vector<cv::Point2f> measured_1 = dyn1.measured_points();

          ///////////////
          x0[0] = 50;
          x0[1] = 20;
          x0[2] = 0;

          input[0] = 5;
          input[1] = 3.14159265359/10;//0

          Dynamics dyn2;
          dyn2.set_time(t0, dt, tend);
          dyn2.set_model(Dynamics::cart);
          dyn2.set_x0(x0);
          dyn2.set_input(input);
          dyn2.set_process_noise(0.0);
          dyn2.set_measurement_noise(0.1);
     
          dyn2.compute_trajectory();     
          std::vector<cv::Point2f> truth_2 = dyn2.truth_points();
          std::vector<cv::Point2f> measured_2 = dyn2.measured_points();
     
          /////////////////////////////////////////////////
          // Convert trajectory points to frames with objects
          /////////////////////////////////////////////////
          int number_of_frames = truth_1.size();
               
          parser.set_width(100);
          parser.set_height(100);
          parser.set_number_of_frames(number_of_frames);     
          
          std::vector<cv::Point2f>::iterator it_1 = truth_1.begin();
          std::vector<cv::Point2f>::iterator it_2 = truth_2.begin();          
     
          int frame_number = 0;
          for (; it_1 != truth_1.end(); it_1++, it_2++) {
               Frame frame;
               frame.set_frame_number(frame_number);
          
               wb::Entity e1;
          
               e1.set_name(t1_str);
               e1.set_centroid(*it_1);

               wb::Entity e2;
               e2.set_name(t2_str);
               e2.set_centroid(*it_2);

               t1.push_back(e1);
               t2.push_back(e2);
          
               frame.objects[t1_str] = e1;
               frame.objects[t2_str] = e2;
               parser.frames[frame_number] = frame;          
          
               frame_number++;
          } 

     } else {
          int status = parser.ParseFile(tracks_file_name);
          if (status != 0) {
               cout << "Error parsing tracks file." << endl;
               return -1;
          }        
          t1 = parser.get_tracks(t1_str);
          t2 = parser.get_tracks(t2_str);
     }         
     
     // Plot the two trajectories
     std::vector<std::string> to_plot;
     to_plot.push_back(t1_str);
     to_plot.push_back(t2_str);     
     parser.plot_tracks(to_plot, 1);
     
     /////////////////////////////////////////////////////
     // Compare the two trajectories for similarities
     /////////////////////////////////////////////////////
     // Starting from back of trajectory, compute dx, dy for each frame
     std::vector<cv::Point2f> traj_1_diffs, traj_2_diffs;
     
     trajectory_diff(t1, traj_1_diffs);
     trajectory_diff(t2, traj_2_diffs);
     
     // Compute differences in trajectory dx dy's
     {
          double total_cost = 0;
          std::vector<cv::Point2f>::iterator it1 = traj_1_diffs.begin();
          std::vector<cv::Point2f>::iterator it2 = traj_2_diffs.begin();
          for(; it1 != traj_1_diffs.end() && it2 != traj_2_diffs.end(); 
              it1++, it2++) {
               double cost;
               cost = 0.5 * pow( sqrt(pow(it1->x - it2->x, 2) + 
                                      pow(it1->y - it2->y, 2)), 2);
               total_cost += cost;               
          }
          cout << "Total Cost: " << total_cost << endl;
     }
     
     
     return 0;
}
