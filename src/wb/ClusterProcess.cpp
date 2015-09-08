#include <iostream>

#include <opencv_workbench/wb/Point.h>
#include <opencv_workbench/wb/Cluster.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ClusterProcess.h"

using std::cout;
using std::endl;

ClusterProcess::ClusterProcess()
{
     threshold_ = 0;
     gate_ = 15;
     min_cluster_size_ = 30;
     next_id_ = 0;
}

int ClusterProcess::next_available_id()
{
     return next_id_++;
     //return prev_clusters_.back()->id() + 1;
}

void ClusterProcess::cluster_maintenance()
{
     // cull dead.
     std::list<wb::Cluster*>::iterator it = clusters_.begin();
     while(it != clusters_.end()) {
          (*it)->set_matched(false);
          (*it)->set_match(NULL);          
          
          if ((*it)->is_dead()) {
               delete *it;
               it = clusters_.erase(it);
          } else {
               it++;
          }
     }    
}


void ClusterProcess::overlay_clusters(cv::Mat &src, cv::Mat &dst)
{
     //dst = src.clone();
     cv::Mat color;
     cv::cvtColor(src, color, CV_GRAY2BGR);
          
     dst = color;
     std::list<wb::Cluster*>::iterator it = clusters_.begin();
     for (; it != clusters_.end(); it++) {
          std::vector<wb::Point*> points = (*it)->points();

          std::vector<wb::Point*>::iterator it_p = points.begin();                    
          for (; it_p != points.end(); it_p++) {
               dst.at<cv::Vec3b>((*it_p)->position().y, (*it_p)->position().x) = cv::Vec3b(20,255,57);               
          }                                                 
     }
}

void ClusterProcess::overlay_tracks(cv::Mat &src, cv::Mat &dst)
{
     cv::Mat color;
     //cv::cvtColor(src, color, CV_GRAY2BGR);
          
     dst = src;
     std::list<wb::Cluster*>::iterator it = clusters_.begin();
     for (; it != clusters_.end(); it++) {
          if ((*it)->is_visible()) {
               cv::Point centroid_point = (*it)->centroid();
               cv::Point est_centroid = (*it)->estimated_centroid();
               cv::Rect rect = (*it)->rectangle();
               
               std::ostringstream convert;
               convert << (*it)->id();               
               const std::string& text = convert.str();
               
               cv::circle(dst, centroid_point, 1, cv::Scalar(255,255,255), -1, 8, 0);
               cv::rectangle(dst, rect, cv::Scalar(255,255,255), 1, 8, 0);
               cv::putText(dst, text, cv::Point(rect.x-3,rect.y-3), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 1, 8, false);
               wb::drawCross(dst, est_centroid, cv::Scalar(255,255,255), 5);
          }
     }
}

void ClusterProcess::process_frame(cv::Mat &src)
{
     clusters_.clear();
     points_.clear();
     
     CV_Assert(src.depth() != sizeof(uchar));
     int channels = src.channels();
     int nRows = src.rows;
     int nCols = src.cols * channels;          

     // Find all points that are greater than some threshold (zero, usually).
     // Organize into array of points. (cv::Point, value)     
     int i,j;
     uchar* p;                         
     for( i = 0; i < nRows; ++i) {
          p = src.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               if (p[j] > threshold_) {
                    wb::Point point;
                    point.set_position(cv::Point(j,i));
                    point.set_value(p[j]);
                    points_.push_back(point);
               }
          }
     }

     // Outer-loop through all points
     int cluster_id = 1;        
     std::vector<wb::Point>::iterator it1 = points_.begin();
     for (; it1 != points_.end(); it1++) {
          std::vector<wb::Point>::iterator it2 = points_.begin();
          for (; it2 != points_.end(); it2++) {     

               // Skip if this is the same point:
               if (it1->position() == it2->position()) {
                    continue;
               }
                    
               // Calculate distance between all outer-loop points and
               // inner-loop points
               float dist = it1->distance(*it2);
               if ( dist <= gate_) {
                    // Only assign this pixel to the new cluster if it 
                    // hasn't been assigned or if it's old distance is
                    // greater than the new distance
                    if (!(it2->assigned()) || it2->distance() >= dist ) {
                         // If the outer loop's point hasn't been assigned
                         // to a cluster yet, create one.
                         if (!(it1->assigned())) {
                              wb::Cluster *c = new wb::Cluster;
                              c->set_id(cluster_id++);
                                   
                              it1->set_assigned(true);
                              it1->set_distance(dist);
                              it1->set_parent(c);

                              // If it2 is assigned make sure to remove it
                              // from old parent
                              if (it2->assigned()) {
                                   wb::Cluster *c_temp = it2->parent();
                                   if (c_temp != NULL) {
                                        c_temp->remove_point(*it2);
                                   } else {
                                        cout << "Point is assigned, but has no parent" << endl;
                                        cout << "Point: " << it2->position() << endl;
                                   }
                              }
                                   
                              it2->set_assigned(true);
                              it2->set_distance(dist);
                              it2->set_parent(c);
                                   
                              c->add_point(&(*it1));
                              c->add_point(&(*it2));
                              clusters_.push_back(c);
                         } else {
                              // Assign the inner loop's point to the
                              // outer loop's cluster
                              if (dist <= it1->distance()) {
                                   it1->set_distance(dist);
                              }
                                   
                              if (it2->assigned()) {
                                   wb::Cluster *c = it2->parent();
                                   if (c != NULL) {
                                        c->remove_point(*it2);
                                   } else {
                                        cout << "Point is assigned, but has no parent" << endl;
                                        cout << "Point: " << it2->position() << endl;
                                   }
                              }                                    
                                   
                              it2->set_assigned(true);
                              it2->set_distance(dist);
                              it2->set_parent(it1->parent());
                                   
                              it1->parent()->add_point(&(*it2));
                         }
                    } 
               }
          }
     } 
          
     // Loop through all clusters. remove small clusters, draw points in 
     // remaining clusters on image
     //cv::Mat cluster_img = cv::Mat::zeros(src.size(), src.type());
     std::list<wb::Cluster*>::iterator it = clusters_.begin();
     while(it != clusters_.end()) {
          // Remove small clusters
          if ((*it)->size() < min_cluster_size_) {
               delete *it;
               it = clusters_.erase(it);
          } else {
               (*it)->init();
               it++;
          }
          //} else {
          //     std::vector<wb::Point*> points = (*it)->points();
          //     std::vector<wb::Point*>::iterator it_p = points.begin();                                                            
          //          
          //     for (; it_p != points.end(); it_p++) {
          //          cluster_img.at<uchar>((*it_p)->position().y, (*it_p)->position().x) = cluster_color_count;
          //     }                                                 
          //     cluster_color_count += 10;
          //     it++;
          //}
     }    

     // Update all previous cluster's motion models
     std::list<wb::Cluster*>::iterator it_prev = prev_clusters_.begin();
     for(; it_prev != prev_clusters_.end(); it_prev++) {
          (*it_prev)->predict_tracker();
     }
     
     // Greedy correlation between this iteration's clusters and previous
     // iteration's clusters
     it = clusters_.begin();
     for(; it != clusters_.end(); it++) {
          double min_dist = 500;
          wb::Cluster * champ_cluster = NULL;
          
          std::list<wb::Cluster*>::iterator it_prev = prev_clusters_.begin();
          for(; it_prev != prev_clusters_.end(); it_prev++) {
               cv::Point p1 = (*it)->centroid();
               cv::Point p2 = (*it_prev)->centroid();
               //double dist = sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
               double dist = pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2); //sqrt doesn't change comparative distances
               if (dist < min_dist) {
                    min_dist = dist;
                    champ_cluster = *it_prev;
               } 
          }
          
          if (champ_cluster != NULL) {
               // Found a cluster match.
               
               // has the previous cluster been matched already
               if (champ_cluster->matched()) {
                    if (min_dist < champ_cluster->distance()) {
                         // a better match was found
                         
                         // unassign old match, new cluster?
                         wb::Cluster * c = champ_cluster->match();
                         if (c != NULL) {
                              // Possible new cluster
                              c->set_id( next_available_id() );
                              //c->set_age( 1 );
                              c->set_distance(999999);
                              c->set_matched(false);
                              c->set_match(NULL);
                              // tracker change?
                              cout << "Reassign" << endl;
                         } else {
                              cout << "Matched is null" << endl;
                         }
                         
                         // Assign better match                         
                         (*it)->set_id( champ_cluster->id() );
                         (*it)->set_age( champ_cluster->age() );
                         (*it)->inc_age();
                         (*it)->set_tracker(champ_cluster->tracker());
                         (*it)->correct_tracker();
                         
                         champ_cluster->set_distance(min_dist);
                         champ_cluster->set_match(*it);
                         champ_cluster->set_matched(true);
                         
                         
                    } else {
                         // the previous match is better
                         // Possible new cluster
                         (*it)->set_id( next_available_id() ); // what's next id?
                         //(*it)->set_age( 1 );                         
                         (*it)->set_distance(999999);
                         (*it)->set_matched(false);
                    }
               } else {
                    // matched
                    (*it)->set_id( champ_cluster->id() );
                    (*it)->set_age( champ_cluster->age() );
                    (*it)->inc_age();
                    (*it)->set_tracker(champ_cluster->tracker());
                    (*it)->correct_tracker();
                    champ_cluster->set_distance(min_dist);
                    champ_cluster->set_match(*it);
                    champ_cluster->set_matched(true);
               }
               
          } else {
               // Possible new cluster
               (*it)->set_id( next_available_id() ); // what's next id?
               //(*it)->set_age( 1 );
               (*it)->set_distance(999999);
               (*it)->set_matched(false);
          }
     }

     // Copy over the previous clusters that weren't detected.
     it_prev = prev_clusters_.begin();
     for(; it_prev != prev_clusters_.end(); it_prev++) {
          if (!(*it_prev)->matched()) {     
               (*it_prev)->dec_age();
               clusters_.push_back((*it_prev));
          }
     }

     this->cluster_maintenance();          

     //cv::imshow("clusters", cluster_img);          
     //cv::Mat cluster_display;
     //cv::normalize(cluster_img, cluster_display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
     //cv::applyColorMap(cluster_display, cluster_display, cv::COLORMAP_JET);     
           
#define ENABLE_CHECK 0
#if ENABLE_CHECK
     bool wait = false;
     // Do any points in a cluster, belong in another cluster?
     it = clusters_.begin();
     for (; it != clusters_.end(); it++) {                              
          std::list<wb::Cluster*>::iterator it2 = clusters_.begin();
          for (; it2 != clusters_.end(); it2++) {
               // Continue if looking at same cluster
               if (it == it2) {
                    continue;
               }                    
               std::vector<wb::Point*>::iterator it_points1 = (*it)->points().begin();
               for (; it_points1 != (*it)->points().end(); it_points1++) {
                    std::vector<wb::Point*>::iterator it_points2 = (*it2)->points().begin();
                    for (; it_points2 != (*it2)->points().end(); it_points2++) {
                         if ( (*it_points1)->position() == (*it_points2)->position()) {
                              cout << "----------------" << endl;
                              cout << "Cluster IDs: " << (*it)->id() << ", " << (*it2)->id() << endl;
                              cout << "Point1: " << (*it_points1)->position().x << ", " << (*it_points1)->position().y << endl;
                              cout << "Point2: " << (*it_points2)->position().x << ", " << (*it_points2)->position().y << endl;
                              // cluster_display.at<uchar>((*it_points2)->position().y,(*it_points2)->position().x)(0) = 0;
                              cluster_display.at<cv::Vec3b>((*it_points2)->position().y,(*it_points2)->position().x) = cv::Vec3b(0,0,0);
                                   
                              cv::circle(cluster_display, cv::Point((*it_points2)->position().x,(*it_points2)->position().y), 5, cv::Scalar(0,0,255), 1, 8, 0);
                              wait = true;                                   
                         }
                    }
               }
          }
     }
#endif

     //cv::imshow("clusters display", cluster_display);
     //cv::Mat large;
     //cv::resize(cluster_display, large, cv::Size(0,0), 2, 2, cv::INTER_LINEAR );
     //cv::imshow("large", large);
           
#if ENABLE_CHECK
     if (wait) { 
          cv::waitKey(0);
     }
#endif     

     //it = clusters_.begin();
     //for (; it != clusters_.end(); it++) {
     //     std::vector<wb::Point*> points = (*it)->points();
     //
     //     std::vector<wb::Point*>::iterator it_p = points.begin();                    
     //     for (; it_p != points.end(); it_p++) {
     //          if ((*it_p)->position().x > src.cols || (*it_p)->position().x < 0 || (*it_p)->position().y < 0 || (*it_p)->position().y > src.rows) {
     //               cout << "Cluster ID: " << (*it)->id() << endl;
     //               cout << "Point out of bounds: ";
     //               cout << (*it_p)->position().x << ", " << (*it_p)->position().y << endl;
     //          } 
     //     }                                                 
     //}

     //// Save points and clusters for next iteration
     prev_clusters_.clear();
     prev_points_.clear();          
     prev_clusters_ = clusters_;                
     prev_points_ = points_;
}
