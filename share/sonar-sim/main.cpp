#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <map>
#include <list>

#include <Eigen/Eigenvalues>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <opencv_workbench/syllo/syllo.h>

#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/track/Dynamics.h>
#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/track/EKF.h>
#include <opencv_workbench/wb/Entity.h>
#include <opencv_workbench/wb/Blob.h>
#include <opencv_workbench/wb/BlobProcess.h>
#include <opencv_workbench/trajectory/TrajectoryAnalysis.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <yaml-cpp/yaml.h>

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

// our data types
struct Vec3_Yaml {
     double x, y, z;
};

struct Vec2_Yaml {
     double x, y;
};

void operator >> (const YAML::Node& node, Vec2_Yaml& v)
{
     node[0] >> v.x;
     node[1] >> v.y;
}


void operator >> (const YAML::Node& node, Vec3_Yaml& v)
{
     node[0] >> v.x;
     node[1] >> v.y;
     node[2] >> v.z;
}

double sonar_noise = 0.0;
double sonar_range = 20.0;

void operator >> (const YAML::Node& node, Dynamics &dyn) {
     std::string str_temp;
     node["type"] >> str_temp;
     dyn.set_type(str_temp);

     int id;
     node["id"] >> id;
     dyn.set_id(id);

     Vec3_Yaml vec3;
     node["position"] >> vec3;
     double heading;
     node["heading"] >> heading;
     
     Dynamics::state_5d_type x0;     
     x0[0] = vec3.x;
     x0[1] = vec3.y;
     x0[2] = heading * PI / 180.0;
     x0[3] = 0;
     x0[4] = 0;
     dyn.set_x0(x0);
     
     str_temp = "";
     node["motion"] >> str_temp;
     if (str_temp == "roomba") {
          dyn.set_model(Dynamics::roomba);
     } else {          
          dyn.set_model(Dynamics::cart);
     }
     
     if(const YAML::Node *pInputs = node.FindValue("input_sequence")) {
          for(unsigned int i = 0; i < pInputs->size(); i++) {
               Vec2_Yaml vec2;
               (*pInputs)[i]["control_input"] >> vec2;
               Dynamics::state_5d_type input;
               input[0] = vec2.x;
               input[1] = vec2.y;
               
               double time;
               (*pInputs)[i]["time"] >> time;
               
               dyn.set_input(input,time);               
          }
     } else {
          Vec2_Yaml vec2;
          node["control_input"] >> vec2;
          Dynamics::state_5d_type input;
          input[0] = vec2.x;
          input[1] = vec2.y;
          dyn.set_input(input);
     }         
     
     if(const YAML::Node *pName = node.FindValue("sonar_noise")) {
          *pName >> sonar_noise;
     }
     
     if(const YAML::Node *pName = node.FindValue("sonar_range")) {
          *pName >> sonar_range;
     }         
}

void draw_sonar_lines(cv::Mat &src, cv::Mat &dst, double beam_width)
{
     double img_height = src.cols;
     double img_width = src.rows;
     
     dst = src.clone();
     /////////////////////////////////////////////////////////////////////
     // Draw the sonar's outside polygon
     double start_angle = (PI - beam_width) / 2 ;
     double end_angle = start_angle + beam_width;// * PI / 180.0;//90.0 * PI / 180.0;//1.963495408;
     cv::Point start(img_width/2,img_height);
     cv::Point end_1(start.x+img_height*cos(start_angle), start.y-img_height*sin(start_angle));
     cv::Point end_2(start.x+img_height*cos(end_angle), start.y-img_height*sin(end_angle));
                    
     // Line from sonar head to start of outer-arc
     cv::line(dst, start, end_1, cv::Scalar(0,0,0), 1, 8, 0);
     cv::Point prev = end_1;
          
     // Draw outer-arc polygon
     double num_pts = 25;
     double step = (end_angle - start_angle) / num_pts;
     for (double ang = start_angle; ang <= end_angle; ang += step) {
          //contour.push_back(cv::Point(start.x+img_height*cos(ang), start.y+img_height*sin(ang)));
          cv::Point p(cv::Point(start.x+img_height*cos(ang), start.y-img_height*sin(ang)));
          cv::line(dst, prev, p, cv::Scalar(0,0,0), 1, 8, 0);
          prev = p;
     }
     
     // Line from outer-arc to real final arc position
     cv::line(dst, prev, end_2, cv::Scalar(0,0,0), 1, 8, 0);
     
     // Line from outer-arc back to sonar head
     cv::line(dst, end_2, start, cv::Scalar(0,0,0), 1, 8, 0);          
}

#define USE_HUNGARIAN 0
#define USE_MHT 1
#define USE_SINGLE 0

int main(int argc, char *argv[])
{
     cout << "================================" << endl;
     cout << "  Simple Sonar Simulator" << endl;
     cout << "================================" << endl;

     if (argc < 2) {
          cout << "usage: " << argv[0] << " yaml-file" << endl;
          return -1;
     }
     
     double t0 = 0;
     double dt;
     double tend;
     double sample_period;
     
     std::ifstream fin(argv[1]);
     YAML::Parser parser(fin);     
     YAML::Node doc;
     parser.GetNextDocument(doc);
     
     // Get time information:
     doc["time_step"] >> dt;
     doc["time_end"] >> tend;

     if(const YAML::Node *psampleperiod = doc.FindValue("sensor_sample_period")) {
          *psampleperiod >> sample_period;
     } else {
          sample_period = dt;
     }
     
     int sample_period_count = cvRound(sample_period / dt);

     double traj_threshold;
     doc["traj_threshold"] >> traj_threshold;

     bool display_plot;
     doc["display_plot"] >> display_plot;
     
     // Get entity information:
     std::vector<Dynamics> dyns;
     const YAML::Node& entities = doc["entities"];
     for(unsigned int i = 0; i < entities.size(); i++) {
          Dynamics dyn;
          entities[i] >> dyn;
          dyns.push_back(dyn);          
     }     
          
     // Get reference to the Sonar sensor
     Dynamics *sonar = NULL;
     for (std::vector<Dynamics>::iterator it = dyns.begin(); it != dyns.end();
          it++) {
          it->set_time(0, dt, tend);
          it->set_process_noise(0.0);
          it->set_measurement_noise(sonar_noise);
     
          if (it->type() == "sensor") {
               sonar = &(*it);
          }
     }     
     if (sonar == NULL) { 
          cout << "sensor not found in yaml file." << endl;
          return -1;
     }
     
     // Plot vectors
     std::map< std::string, std::vector<cv::Point3d> > vectors;     
     const std::string title = "Tracks";
     std::vector<std::string> labels;
     std::vector<std::string> styles;               
     
     // Setup plot and basic options
     syllo::Plot plot;     
     std::string options;
     options = "set size ratio -1\n";
     options += "set view equal xy\n"; 
     options += "set size 1,1\n";     

     // Sonar Image Setup
     double max_angle_ = 0.392699082;
     double min_angle_ = -0.392699082;
     double x_min = 0.00;  // R_min
     double x_max = sonar_range; // R_max
     double y_max = sin(max_angle_)*x_max*2;
     double beam_width_ = max_angle_ * 2;
     
     int img_width = 600;
     int img_height = 600;
     cv::Mat frame_img = cv::Mat::zeros(img_height, img_width, CV_8UC3);
     frame_img = cv::Scalar(255,255,255);
     
     cv::Mat history_img = cv::Mat::zeros(img_height, img_width, CV_8UC3);
     history_img = cv::Scalar(255,255,255);     
     
     // Setup rotation matrix
     cv::Point center = cv::Point( frame_img.cols/2, frame_img.rows/2 );
     double angle = 180.0;
     double rot_scale = 1.0;     
     cv::Mat rot_mat(2,3,CV_8UC1);
     rot_mat = cv::getRotationMatrix2D( center, angle, rot_scale );
     
     TrajectoryAnalysis traj;
     traj.set_simulated(true);
     std::map<int, std::list<wb::Blob> > tracks_history;
     
     BlobProcess blob_process;
     std::vector<wb::Blob> tracks;
     // Loop through all generated trajectories
     bool step_flag = true;
     int loop_number = 0;
     int frame_number = 0;
     bool exit_sim = false;
     std::vector<double> tt = Dynamics::time_vector(t0,dt,tend);     
     for (std::vector<double>::iterator it = tt.begin(); 
          it != tt.end() && !exit_sim; it++, loop_number++) {

          bool scan_sample = loop_number % sample_period_count == 0 ? true : false;

          std::map<int, wb::Blob> tracks_frame;
          
          //frame_img = cv::Scalar(255,255,255);
                    
          // Clear out the plot vectors
          //vectors.clear() ; labels.clear(); styles.clear();          

          Dynamics::state_5d_type sonar_state = sonar->state();
          cv::Point2d sonar_pos(sonar_state[0], sonar_state[1]);
          double sonar_heading = sonar_state[2];

          std::vector<wb::Blob> measurements;          
          
          // Run Kalman Filter on all tracks          
          for(std::vector<wb::Blob>::iterator it_tracks = tracks.begin(); 
              it_tracks != tracks.end(); it_tracks++) {
               if (it_tracks->is_confirmed()) {
                    it_tracks->predict_tracker();
               }
          }                   
          
          // Create measurements from sensor's perspective
          for (std::vector<Dynamics>::iterator it_ent = dyns.begin();
               it_ent != dyns.end(); it_ent++) {
               
               // Grab the current state and then step the model
               Dynamics::state_5d_type measurement = it_ent->measurement();
               
               cv::Point2d pos(measurement[0], measurement[1]);
               cv::Point3d pos_3d(measurement[0], measurement[1], 0);

               vectors[syllo::int2str(it_ent->id())].push_back(pos_3d);
               
               if (it_ent->type() == "sensor") {
                    // Skip if this is the sensor
                    continue;
               }               
               
               if (scan_sample) {
                    /////////////////////////////////////////////////
                    // Can the sonar see the object at this point?
                    // Within range and bearing limits?          
                    // Compute bearing from sonar to target          
                    // Position of contact, relative to ownship                              
                    cv::Point2d offset = pos - sonar_pos;
                    cv::Point2d cnt_relative_pos = rotate_2D(offset, -sonar_heading);
                    double bearing_to_cnt = atan2(cnt_relative_pos.y, 
                                                  cnt_relative_pos.x);
               
                    double dist = cv::norm(offset);               
               
                    if ((min_angle_ <= bearing_to_cnt) && (bearing_to_cnt <= max_angle_) && 
                        (dist >= x_min) && dist <= x_max) {
                         // Put target in sonar's image
               
                         //cout << "Bearing: " << bearing_to_cnt * 180 / PI;
                
                         // convert offset to polar coordinates
                         double R = dist;
                         double theta = bearing_to_cnt;
                
                         // x = R*cos(theta), y = R*sin(theta);
                         double x = R*cos(theta);
                         double y = R*sin(theta);                    
                     
                         int x_pos = cvRound(img_width/2 - y / (y_max/2) * img_height*sin(max_angle_));
                         int y_pos = cvRound(img_height - x / x_max * img_height);                                            
                    
                         if (x_pos < 0 || x_pos > frame_img.cols || y_pos < 0 || y_pos > frame_img.rows) {
                              // error, skip
                              cout << "bounds" << endl;
                              continue;
                         }                         

                         cv::Point blob_pos(x_pos, y_pos);

                         wb::Blob blob;
                         blob.set_id(it_ent->id());
                         blob.set_centroid(cv::Point2d(x,y));
                         wb::Point p;
                         p.set_position(blob_pos);
                         p.set_value(255);
                         blob.add_point(p);
                         blob.set_frame(frame_number);                    
                         blob.init();                         
                    
                         tracks_history[it_ent->id()].push_back(blob);
                         tracks_frame[it_ent->id()] = blob;
                    
                         measurements.push_back(blob);
                    
                         wb::drawTriangle(frame_img, blob_pos, cv::Scalar(0,0,0), 10);
                    
                         cv::circle(frame_img,blob_pos,1,cv::Scalar(0,0,0),-1,8,0);
                         cv::circle(history_img,blob_pos,1,cv::Scalar(0,0,0),-1,8,0);
                                        
                         std::ostringstream convert;
                         convert << it_ent->id();               
                         const std::string& text = convert.str();
                         cv::putText(frame_img, text, cv::Point(x_pos+5, y_pos-5), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0,0,0), 1, 8, false);                         
                    } 
                    
                    // Draw sonar lines on images
                    draw_sonar_lines(frame_img, frame_img, beam_width_);          
                    draw_sonar_lines(history_img, history_img, beam_width_);                                                 

#if USE_SINGLE                                        
                    // Assume single track for now
                    if (tracks.size() == 0) {
                         tracks = measurements;   
                         tracks.back().set_age(6); // make it tracked
                    } else {
                         Eigen::MatrixXf Zm;
                         Zm.resize(2,1);
                         Zm << measurements.back().pixel_centroid().x, measurements.back().pixel_centroid().y;

                         if(tracks.back().tracker().is_within_region(Zm,0.99)) {
                              cout << "In region!" << endl;
                         } else {
                              cout << "Out of region!" << endl;
                         }
                         
                         measurements.back().matched_track(tracks.back());
                         tracks = measurements;
                    }     
#elif USE_HUNGARIAN
                    // Send "measurements" to hungarian assignment algorithm
                    std::vector<wb::Blob> fused_tracks;
                    blob_process.assign_hungarian(measurements, tracks, fused_tracks);
                    tracks = fused_tracks;                    
#elif USE_MHT
                    // Send "measurements" to MHT
                    std::vector<wb::Blob> fused_tracks;
                    blob_process.assign_mht(measurements, tracks, fused_tracks);
                    tracks = fused_tracks;                    
#endif                              
               } // if scan_sample             
               blob_process.set_blobs(tracks);
               
               // Draw Tracks on image
               cv::Mat tracks_img(frame_img.size(), CV_8UC3);
               tracks_img = cv::Scalar(255,0,0);
               blob_process.overlay(tracks_img, tracks_img, TRACKS | ERR_ELLIPSE);
               cv::imshow("Tracks", tracks_img);
          
               //// Compute Trajectory Analysis
               //cv::Mat traj_img = frame_img;//img_color;//.clone();
               //traj.trajectory_similarity(tracks_history, frame_number, 
               //                           traj_img, traj_threshold);
               //
               //cv::Mat history_traj_img = history_img;//img_color;//.clone();
               //traj.trajectory_similarity(tracks_history, frame_number, 
               //                           history_traj_img, traj_threshold);
               //cv::imshow("History", history_traj_img);
               //cv::imshow("Frame", traj_img);           
          
               cv::Mat traj_img = frame_img;
               traj.trajectory_similarity_track_diff(tracks_frame, frame_number, 
                                                     traj_img, traj_threshold);
          
               cv::imshow("Frame", traj_img);  

               /////////////////////////////////////////////////////////////////////                    
                    
               // Handle pausing / stepping
               if (step_flag) {
                    int key = cv::waitKey(0);    
                    if (key == 'q' || key == 1048689) { // 'q' key
                         cout << "Ending early." << endl;
                         exit_sim = true;                              
                    } else if (key == 'p' || key == 1048688) {
                         step_flag = 0;
                    }     
               } else {
                    int key = cv::waitKey(30);               
                    if (key == 'q' || key == 1048689) {
                         cout << "Ending early." << endl;
                         exit_sim = true;                              
                    } else if (key == 'p' || key == 1048688) {
                         step_flag = 1;
                    } 
               }
          }

          // Generate Trajectories for all objects that have dynamics
          for (std::vector<Dynamics>::iterator it_ent = dyns.begin();
               it_ent != dyns.end(); it_ent++) {               
               // Grab the current state and then step the model
               it_ent->step_motion_model(dt, *it);
          }
          
          if (display_plot) {
               // Plot the tracks          
               std::vector<std::string> empty_objects;
               plot.plot(vectors, title, labels, styles, options, empty_objects, false);
          }          
          frame_number++;                              
     }     
     cv::waitKey(0);     
     return 0;
}
