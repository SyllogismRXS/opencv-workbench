#include <iostream>

#include "ObjectTracker.h"

#include <opencv_workbench/wb/BlobProcess.h>
#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>

#include <Eigen/Dense>


using std::cout;
using std::endl;

ObjectTracker::ObjectTracker()
{     
     next_id_ = 1;
     covar_tracker_.init(2,2);
}

int ObjectTracker::next_available_id()
{
     return next_id_++;
}

void ObjectTracker::process_frame(cv::Mat &src, std::vector<wb::Blob> &meas)
{
     cv::Mat img = src.clone();
     cv::cvtColor(img,img,CV_GRAY2BGR);
     
     tracks_.clear();

     // Predict each KF
     for (std::vector<wb::Blob>::iterator it_prev = prev_tracks_.begin(); 
               it_prev != prev_tracks_.end(); it_prev++) {
          it_prev->predict_tracker();
     }

     std::vector<wb::Blob*> intermediate;
     
     // Do any of the new blobs match previous tracks. Only count blobs that
     // are of a certain age.
     // Key<int>     : ID of previous track
     // Value <list> : Measurement blob pointers that match track ID
     std::map<int, std::list<wb::Blob*> > track_matches;
     
     for (std::vector<wb::Blob>::iterator it_meas = meas.begin(); 
          it_meas != meas.end(); it_meas++) {

          // Don't use blobs unless they are of a certain age or not occluded
          //if (it_meas->age() < 5 || it_meas->occluded()) {
          //if (it_meas->age() < 2 || it_meas->occluded()) {
          if (it_meas->occluded()) {
               continue;
          }
          
          bool matched = false;
          for (std::vector<wb::Blob>::iterator it_prev = prev_tracks_.begin(); 
               it_prev != prev_tracks_.end(); it_prev++) {                             
               
               // Do any of the blob's points fall within 3-sigma region of the
               // track?
               for (std::vector<wb::Point>::iterator it_point = it_meas->points().begin(); 
                    it_point != it_meas->points().end(); it_point++) {
                    
                    if (it_prev->pixel_tracker().is_within_region(cv::Point2d(it_point->x(),it_point->y()),0.9973)) {
                         track_matches[it_prev->id()].push_back(&(*it_meas));
                         matched = true;                                              
                         break;
                    }
               }                              
          }
          
          // If the measurement doesn't fall within any previous track,
          // initiate a new one.
          if (!matched) {                     
               int id = next_available_id();               
               cout << "New Object" << id << endl;
               it_meas->new_track(id);
               it_meas->pixel_tracker().set_R(1000,0,0,1000);
               it_meas->pixel_tracker().set_P(100);
               it_meas->pixel_tracker().set_Q(10);
               it_meas->set_dead_occluded_age(16);
               intermediate.push_back(&(*it_meas));
          }
     }     
     
     // For each track ID with matches, use kalman filter to integrate position
     for (std::vector<wb::Blob>::iterator it_prev = prev_tracks_.begin(); 
          it_prev != prev_tracks_.end(); it_prev++) {          

          if (track_matches.count(it_prev->id()) > 0) {                                        
               cv::Point2d sum_weighted(0,0); 
               int count_weighted = 0;

               cv::Point2d sum(0,0); 
               int count = 0;
                              
               // Find "centroid" of all measurements
               for (std::list<wb::Blob*>::iterator it_match = track_matches[it_prev->id()].begin();
                    it_match != track_matches[it_prev->id()].end(); it_match++) {                             
                    
                    sum_weighted.x += (*it_match)->estimated_pixel_centroid().x * (*it_match)->age();
                    sum_weighted.y += (*it_match)->estimated_pixel_centroid().y * (*it_match)->age();
                    count_weighted += (*it_match)->age();

                    sum.x += (*it_match)->estimated_pixel_centroid().x;
                    sum.y += (*it_match)->estimated_pixel_centroid().y;
                    count++;
                                        
                    wb::drawCross(img, (*it_match)->estimated_pixel_centroid(), cv::Scalar(0,255,0), 8);
               }               

               cv::Point2d avg_weighted(0,0);
               avg_weighted.x = sum_weighted.x / (double)count_weighted;
               avg_weighted.y = sum_weighted.y / (double)count_weighted;               

               wb::Blob b;
               b.set_pixel_centroid(avg_weighted);
               b.set_centroid(avg_weighted);//todo, not right
               //wb::drawCross(img, avg_weighted, cv::Scalar(255,255,255), 5);               

               cv::Point2d avg;
               avg.x = sum.x / (double)count;
               avg.y = sum.y / (double)count;               
               //wb::drawCross(img, avg, cv::Scalar(0,0,255), 5);               

               // Integrate the matched measurement
               it_prev->set_occluded(false);
               it_prev->inc_age();
               it_prev->copy_meas_info(b);
               it_prev->correct_tracker();                          

               // Adjust measurement noise matrix to accomodate diver's shape
               Eigen::MatrixXd covar_sum = Eigen::MatrixXd::Zero(2,2);
               count = 0;
               
               for (std::list<wb::Blob*>::iterator it_match = track_matches[it_prev->id()].begin();
                    it_match != track_matches[it_prev->id()].end(); it_match++) {
                    
                    Eigen::MatrixXd diff(2,1);
                    diff << (*it_match)->estimated_pixel_centroid().x - avg.x,
                            (*it_match)->estimated_pixel_centroid().y - avg.y;

                    covar_sum += (diff * diff.transpose());
                    
                    count++;                    
               }
               
               Eigen::MatrixXd covar(2,2);
               covar = covar_sum / ((double)count);
               //covar = covar.Eigen::sqrt();               
               //covar *= 10;               
               //covar *= 5;
               covar *= 3;
               
               double eig_scale = 0.1;
               if (count > 1) {
                    // Check to see if smaller eigenvalue is at least
                    // 0.001 times the larger eigenvalue
                    Eigen::VectorXcd eigvals = covar.eigenvalues();
                    double eig1 = eigvals(0).real();
                    double eig2 = eigvals(1).real();

                    if (eig1 < eig2) {
                         if (eig1 < eig2*eig_scale) {
                              eig1 = eig2*eig_scale;

                              Eigen::EigenSolver<Eigen::MatrixXd> es(covar);
                              Eigen::MatrixXcd ev = es.eigenvectors();
                              Eigen::MatrixXd ev_real = ev.real();

                              Eigen::MatrixXd eigs_corrected(2,2);
                              eigs_corrected << eig1, 0 , 0, eig2;
                              covar = ev_real * eigs_corrected * ev_real.inverse();                              
                         }

                    } else {
                         if (eig2 < eig1*eig_scale) {
                              eig2 = eig1*eig_scale;

                              Eigen::EigenSolver<Eigen::MatrixXd> es(covar);
                              Eigen::MatrixXcd ev = es.eigenvectors();
                              Eigen::MatrixXd ev_real = ev.real();

                              Eigen::MatrixXd eigs_corrected(2,2);
                              eigs_corrected << eig1, 0 , 0, eig2;
                              covar = ev_real * eigs_corrected * ev_real.inverse();                              
                         }
                    }

                    //covar_tracker_.predict();
                    //covar_tracker_.set_values(covar);
                    //covar = covar_tracker_.values();
                    //it_prev->pixel_tracker().set_R(covar(0,0), covar(0,1),
                    //                               covar(1,0), covar(1,1));
               } else {
                    // If only a single measurement, decrease size of
                    // measurement, generate measurement that is "circular"
                    //covar_tracker_.predict();                    
                    covar = it_prev->pixel_tracker().R().cast<double>();
                    
                    Eigen::VectorXcd eigvals = covar.eigenvalues();
                    double eig1 = eigvals(0).real();
                    double eig2 = eigvals(1).real();
                    
                    if (eig1 < eig2) {
                         eig1 *= 0.90;
                         eig2 = eig1;

                         Eigen::EigenSolver<Eigen::MatrixXd> es(covar);
                         Eigen::MatrixXcd ev = es.eigenvectors();
                         Eigen::MatrixXd ev_real = ev.real();

                         Eigen::MatrixXd eigs_corrected(2,2);
                         eigs_corrected << eig1, 0 , 0, eig2;
                         covar = ev_real * eigs_corrected * ev_real.inverse();                              
                         
                    } else {
                         eig2 *= 0.90;
                         eig1 = eig2;
                         Eigen::EigenSolver<Eigen::MatrixXd> es(covar);
                         Eigen::MatrixXcd ev = es.eigenvectors();
                         Eigen::MatrixXd ev_real = ev.real();
                         
                         Eigen::MatrixXd eigs_corrected(2,2);
                         eigs_corrected << eig1, 0 , 0, eig2;
                         covar = ev_real * eigs_corrected * ev_real.inverse();                         
                    }                    
                    
                    //covar_tracker_.set_values(covar);

                    //covar = covar_tracker_.values();
                    //it_prev->pixel_tracker().set_R(covar(0,0), covar(0,1),
                    //                               covar(1,0), covar(1,1));
               }

               //// Clamp minimum and maximum size for covar elements:
               //double max_covar = 2000;
               //for (int r = 0; r < 2; r++) {
               //     for (int c = 0; c < 2; c++) {
               //          if (covar(r,c) > max_covar) {
               //               covar(r,c) = max_covar;
               //          }
               //          //if (covar(r,c) < 50) {
               //          //     covar(r,c) = 50;
               //          //}
               //     }
               //}                                             
               
               // TODO: create minimum / maximum covariance possibilities
               covar_tracker_.predict();
               covar_tracker_.set_values(covar);
               covar = covar_tracker_.values();
               it_prev->pixel_tracker().set_R(covar(0,0), covar(0,1),
                                              covar(1,0), covar(1,1));               

               //cout << it_prev->id() << "-covar:" << endl << covar << endl;
               
               // Define maximum and minimum size for object in scene?
               
          } else {                              
               // Missed track measurement?
               it_prev->missed_track();               
          }
          //tracks_.push_back(*it_prev);
          intermediate.push_back(&(*it_prev));
     }

     // Plot intermediate tracks
     cv::Mat intermediate_img;
     this->overlay(intermediate, src, intermediate_img, TRACKS | IDS | ERR_ELLIPSE);
     cv::imshow("Intermediate", intermediate_img);
     
     // Are any of the tracks very similar? If so, keep the oldest one
     // Determine if the centroids of any tracks are within 1 std of each
     // other.
     //Eigen::MatrixXf Zm1, Zm2; Zm1.resize(2,1); Zm2.resize(2,1);
     
     // TODO : We can't integrate the Kalman filters until we find the track
     // with the oldest age and smallest ID. First find tracks that "overlap"
     // and then integrate them.
     
     for (std::vector<wb::Blob*>::iterator it1 = intermediate.begin(); 
          it1 != intermediate.end(); it1++) {
          for (std::vector<wb::Blob*>::iterator it2 = intermediate.begin(); 
               it2 != intermediate.end(); it2++) {
               
               // If it's the same ID, move on to next track
               if ((*it1)->id() == (*it2)->id()) {                    
                    continue;
               }

               // If one of the tracks was marked "matched," it was already
               // integrated into a track
               if ((*it1)->matched() || (*it2)->matched()) {
                    continue;
               }

               //// Are the track's centroids within 3 std of each other?
               if ((*it1)->pixel_tracker().is_within_region((*it2)->estimated_pixel_centroid(),0.9973) && 
                   (*it2)->pixel_tracker().is_within_region((*it1)->estimated_pixel_centroid(),0.9973)) {
                    
                    bool save_it1;
                    if ((*it1)->age() > (*it2)->age()) {
                         save_it1 = true;
                    } else if ((*it1)->age() < (*it2)->age()) {
                         save_it1 = false;
                    } else {
                         // equal ages...
                         if ((*it1)->id() < (*it2)->id()) {
                              save_it1 = true;
                         } else if ((*it1)->id() > (*it2)->id()) {
                              save_it1 = false;
                         } else {
                              save_it1 = true;
                              cout << "WARNING: unexpected object ID" << endl;
                         }
                    }
                    
                    // Found similar tracks. Save the oldest track
                    if (save_it1) {                                                  
                         // Integrate younger track into older track
                         (*it1)->copy_meas_info(*(*it2));
                         (*it1)->set_occluded(false);
                         (*it1)->correct_tracker();
                         
                         // Mark the younger track as matched, so it is overlooked later
                         (*it2)->set_matched(true);
                    } else {                         
                         // Integrate younger track into older track
                         (*it2)->copy_meas_info(*(*it1));
                         (*it2)->set_occluded(false);
                         (*it2)->correct_tracker();
                         
                         // Mark the younger track as matched, so it is overlooked later
                         (*it1)->set_matched(true);
                    }
               }
          }          
     }     

     // Any track that wasn't marked as "matched" was either older or wasn't
     // similar to another track. Copy them to the final fused vector
     for (std::vector<wb::Blob*>::iterator it1 = intermediate.begin(); 
          it1 != intermediate.end(); it1++) {
          if (!(*it1)->matched()) {               
               tracks_.push_back(*(*it1));
          }
     }    

     // Cull dead tracks
     std::vector<wb::Blob>::iterator it = tracks_.begin();
     while(it != tracks_.end()) {
          it->set_matched(false);
          
          if (it->is_dead()) {
               it = tracks_.erase(it);
          } else {
               it++;
          }
     }

     cv::imshow("Used", img);
     prev_tracks_ = tracks_;
}

void ObjectTracker::overlay(std::vector<wb::Blob*> &tracks, cv::Mat &src, cv::Mat &dst, OverlayFlags_t flags)
{
     std::vector<wb::Blob> tracks_temp;
     for (std::vector<wb::Blob*>::iterator it = tracks.begin(); 
          it != tracks.end(); it++) {
          tracks_temp.push_back(*(*it));
     }
     overlay(tracks_temp,src,dst,flags);
}

void ObjectTracker::overlay(cv::Mat &src, cv::Mat &dst, OverlayFlags_t flags)
{
     overlay(tracks_,src,dst,flags);
}

void ObjectTracker::overlay(std::vector<wb::Blob> &tracks, cv::Mat &src, cv::Mat &dst, OverlayFlags_t flags)
{
     cv::Mat color;
     if (src.channels() == 1) {
          cv::cvtColor(src, color, CV_GRAY2BGR);
     } else {
          color = src;
     }
     dst = color;
     
     for(std::vector<wb::Blob>::iterator it = tracks.begin(); 
         it != tracks.end(); it++) {
          
          if ((flags & CONFIRMED_ONLY) && !it->is_confirmed()) {
               // If we are only plotting confirmed tracks and it's not
               // confirmed, ignore this track/blob.
               continue;
          }
          
          if (flags & BLOBS) {
               cv::Vec3b point_color = cv::Vec3b(20,255,57);
               if (it->occluded()) {
                    point_color = cv::Vec3b(0,0,0);
               }

               // Draw all blob points in the image
               std::vector<wb::Point> points = it->points();
               std::vector<wb::Point>::iterator it_points = points.begin();
               for(; it_points != points.end(); it_points++) {
                    dst.at<cv::Vec3b>(it_points->y(), it_points->x()) = point_color;
               }
          }
         
          cv::Rect rect = it->bbox().rectangle();

          if (flags & RECTS) {
               cv::rectangle(dst, rect, cv::Scalar(255,255,255), 1, 8, 0);
          }

          cv::Point est_centroid = it->estimated_pixel_centroid();
          if (flags & IDS) {
               std::ostringstream convert;
               convert << it->id();
               const std::string& text = convert.str();
               cv::putText(dst, text, cv::Point(est_centroid.x-3,est_centroid.y-3), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 2, 8, false);
          }
          
          if (flags & TRACKS) {
               //cv::Point est_centroid = it->estimated_pixel_centroid();
               wb::drawCross(dst, est_centroid, cv::Scalar(255,255,255), 5);
          }          

          if (flags & ERR_ELLIPSE) {
               Ellipse ell = it->error_ellipse(0.9973); // 3 std
               //Ellipse ell = it->error_ellipse(0.9545); // 2 std
               //Ellipse ell = it->error_ellipse(0.6827); // 1 std
               cv::Point center(cvRound(ell.center().x),cvRound(ell.center().y));
               cv::Size axes(cvRound(ell.axes()(0)), cvRound(ell.axes()(1)));
               //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
               //cv::ellipse(dst, center, axes, -cvRound(ell.angle()), 0, 360, cv::Scalar(255,255,255), 1, 8, 0);
               cv::ellipse(dst, center, axes, cvRound(ell.angle()), 0, 360, cv::Scalar(255,255,255), 1, 8, 0);
          }

          if (flags & VELOCITIES) {
               cv::Point pt1, pt2;
               pt1 = est_centroid;
               pt2 = est_centroid + it->estimated_pixel_velocity();

               int thickness = 1;
               double tip_length = 0.1;
               wb::arrowedLine(dst, pt1, pt2, cv::Scalar(0,255,0), 
                               thickness, 8, 0, tip_length);
          }
     }
}
