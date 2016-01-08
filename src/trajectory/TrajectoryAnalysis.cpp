#include <iostream>
#include <cmath>

#include <opencv_workbench/syllo/syllo.h>

#include "TrajectoryAnalysis.h"

using std::cout;
using std::endl;

TrajectoryAnalysis::TrajectoryAnalysis() : simulated_(false)
{
}

double TrajectoryAnalysis::trajectory_diff(std::list<cv::Point2d> &t1,
                                         std::list<cv::Point2d> &t2)
{
     // Calculate differences in trajectories
     double total_diff = 0;
     std::vector<cv::Point2d> diffs;
     int frame = 0;
     std::list<cv::Point2d>::iterator it1 = t1.begin();
     std::list<cv::Point2d>::iterator it2 = t2.begin();
     for(; it1 != t1.end() && it2 != t2.end(); 
         it1++, it2++) {               
          
          double diff = pow(it1->x - it2->x, 2) + pow(it1->y - it2->y, 2);
          //double diff = sqrt(pow(it1->x - it2->x, 2) + pow(it1->y - it2->y, 2));
          //Polar polar1(it1->x, it1->y);
          //Polar polar2(it2->x, it2->y);
          //double diff = Polar::distance(polar1, polar2);
          
          diffs.push_back(cv::Point2d(frame,diff));               
          total_diff += diff;
          frame++;               
     }
     //vectors.push_back(diffs);
     //labels.push_back("Diff");
     //styles.push_back("linespoints");

     //vectors.push_back(costs);
     //labels.push_back("Cost");
     //styles.push_back("linespoints");
     //plot.plot(vectors, title2, labels, styles);
          
     double mean = total_diff / (double)diffs.size();
     double RMSE = sqrt(mean);
     //double RMSE = mean;
     return RMSE;
}

void TrajectoryAnalysis::trajectory_polar_diff(std::list<wb::Blob> &traj,
                                             std::list<cv::Point2d> &diffs)
{
     std::list<wb::Blob>::reverse_iterator it = traj.rbegin();
     cv::Point2d prev;
     for(; it != traj.rend(); it++) {
          if (it == traj.rbegin()) {
               prev = it->undistorted_centroid();
               continue;
          }
     
          cv::Point2d p = it->undistorted_centroid();          
     
          // Convert to polar
          double range = sqrt( pow(p.x,2) + pow(p.y,2) );
          double range_prev = sqrt( pow(prev.x,2) + pow(prev.y,2) );          
          double theta = atan(p.y / p.x);
          double theta_prev = atan(prev.y / prev.x);
          //
          cv::Point2d polar(range,theta);
          cv::Point2d polar_prev(range_prev, theta_prev);
          
          double range_diff = range - range_prev;
          double theta_diff = theta - theta_prev;
                    
          cv::Point2d temp;
          temp.x = range_diff;
          temp.y = theta_diff;                   
                   
          diffs.push_back(temp);          
               
          prev = it->undistorted_centroid();
     }
}

void TrajectoryAnalysis::trajectory_similarity_track_diff(std::map<int, wb::Blob> &tracks_frame,
                                                          int frame_number, 
                                                          cv::Mat &img, double threshold)
{
     cout << "==========================================" << endl; 

     std::map<std::string, struct Avgs> RMSEs;
     // Store RMSE between each trajectory in a map/hash. A string will be used
     // to hash the values. The string is of the form "n:m", where n and m are
     // the trajectory IDs and n is less than m.
     
     // Reset computed
     for (std::map<std::string, Avgs>::iterator it = traj_avgs_.begin();
          it != traj_avgs_.end(); it++) {
          it->second.computed = false;
     }
     
     for (std::map<int, wb::Blob>::iterator it_1 = tracks_frame.begin(); 
          it_1 != tracks_frame.end(); it_1++) {
          for (std::map<int, wb::Blob>::iterator it_2 = tracks_frame.begin(); 
               it_2 != tracks_frame.end(); it_2++) {
               
               // Determine if this trajectory difference has already been
               // calculated
               std::string key;
               Avgs avg;
               if (it_1->first == it_2->first) {
                    // Don't calculate trajectory diff between two trajectories
                    // with the same ID. This is the same trajectory.
                    continue;
               } else if (it_1->first < it_2->first) {
                    key = syllo::int2str(it_1->first) + ":" + syllo::int2str(it_2->first);
                    avg.ID1 = it_1->first;
                    avg.ID2 = it_2->first;
               } else {
                    key = syllo::int2str(it_2->first) + ":" + syllo::int2str(it_1->first);
                    avg.ID1 = it_2->first;
                    avg.ID2 = it_1->first;
               }                   
               
               // Determine if we already computed this pair's similarity               
               if (traj_avgs_.count(key) != 0 && traj_avgs_[key].computed) {
                    continue;
               }

               traj_avgs_[key].ID1 = avg.ID1;
               traj_avgs_[key].ID2 = avg.ID2;
               
               cv::Point2d p1 = it_1->second.undistorted_centroid();
               cv::Point2d p2 = it_2->second.undistorted_centroid();

               Polar polar1 = Polar::cart2polar(p1);
               Polar polar2 = Polar::cart2polar(p2);
               
               double diff = Polar::distance(polar1,polar2);
               
               traj_avgs_[key].diff_sum += diff;
               traj_avgs_[key].count++;
               traj_avgs_[key].computed = true; 

               
               
               if (simulated_ || (it_1->second.is_tracked() && 
                                  it_2->second.is_tracked())) {
                    
                    double diff_avg = traj_avgs_[key].diff_sum / (double)traj_avgs_[key].count;
               
                    cout << "--------" << endl;
                    cout << key << endl;               
                    cout << "Diff: " << diff << endl;
                    cout << "Diff Avg: " << diff_avg << endl;                                             
                              
                    double abs_diff = std::abs(diff-diff_avg);
                    cout << "Value: " << abs_diff << endl;
                    
                    if (abs_diff < threshold) {
                         cv::Point c1 = it_1->second.centroid();
                         cv::Point c2 = it_2->second.centroid();
                         if (img.channels() == 1) {
                              cv::line(img, c1, c2, 0, 1, 8, 0);
                         } else {
                              cv::line(img, c1, c2, cv::Scalar(0,255,255), 1, 8, 0);
                         }
                    }
               } 
          }
     }     
}

void TrajectoryAnalysis::trajectory_similarity(std::map<int, std::list<wb::Blob> > &tracks_history, int frame_number, cv::Mat &img, double RMSE_threshold)
{
     cout << "==========================================" << endl;          
     std::map<int, std::list<cv::Point2d> > trajectories;
     for (std::map<int, std::list<wb::Blob> >::iterator 
               it = tracks_history.begin(); it != tracks_history.end(); ) {
          
          if (it->second.back().frame() < frame_number) {
               // Clear out track IDs that are dead
               tracks_history.erase(it++);
          } else {
               if (it->second.back().is_tracked() || simulated_) {                    
                    // Compute derivative of each current track's trajectory
                    trajectory_polar_diff(it->second, trajectories[it->first]);                    
               }
               it++;
          }
     }

     std::map<std::string, struct TrajRMSE> RMSEs;
     // Store RMSE between each trajectory in a map/hash. A string will be used
     // to hash the values. The string is of the form "n:m", where n and m are
     // the trajectory IDs and n is less than m.

     // Calculate differences between trajectory derivatives
     std::map<int, std::list<cv::Point2d> >::iterator it_1 = trajectories.begin();
     for(; it_1 != trajectories.end(); it_1++) {
          std::map<int, std::list<cv::Point2d> >::iterator it_2 = trajectories.begin();
          for(; it_2 != trajectories.end(); it_2++) {
               // Determine if this trajectory difference has already been
               // calculated
               std::string key;
               struct TrajRMSE traj;
               if (it_1->first == it_2->first) {
                    // Don't calculate trajectory diff between two trajectories
                    // with the same ID. This is the same trajectory.
                    continue;
               } else if (it_1->first < it_2->first) {
                    key = syllo::int2str(it_1->first) + ":" + syllo::int2str(it_2->first);
                    traj.ID1 = it_1->first;
                    traj.ID2 = it_2->first;
               } else {
                    key = syllo::int2str(it_2->first) + ":" + syllo::int2str(it_1->first);
                    traj.ID1 = it_2->first;
                    traj.ID2 = it_1->first;
               }

               if (RMSEs.count(key) == 0) {               
                    double RMSE = trajectory_diff(it_1->second, it_2->second);
                    traj.RMSE = RMSE;
                    RMSEs[key] = traj;
                    //cout << "--------------" << endl;
                    //cout << it_1->first << " - " << it_2->first << " : " << RMSE << endl;
               }
          }
     }

     // Print out the RMSEs for this frame:
     for(std::map<std::string, struct TrajRMSE>::iterator it = RMSEs.begin();
         it != RMSEs.end(); it++) {
          cout << it->first << ": " << it->second.RMSE << endl;
     
          // If the RMSE between two trajectories is less than a threshold,
          // circle the trajectories
          if (it->second.RMSE < RMSE_threshold) { // 0.017
               // Get the two IDs in the track history and circle them
               cv::Point p1 = tracks_history[it->second.ID1].back().centroid();
               cv::Point p2 = tracks_history[it->second.ID2].back().centroid();
               
               //cv::circle(img, p1, 10, cv::Scalar(0,255,255), 1, 8, 0);
               //cv::circle(img, p2, 10, cv::Scalar(0,255,255), 1, 8, 0);
               if (img.channels() == 1) {
                    cv::line(img, p1, p2, 0, 1, 8, 0);
               } else {
                    cv::line(img, p1, p2, cv::Scalar(0,255,255), 1, 8, 0);
               }
          }          
     }

     //cv::imshow("SimTraj", img);
}
