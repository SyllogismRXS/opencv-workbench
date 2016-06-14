#include <iostream>

#include "ObjectTracker.h"

#include <opencv_workbench/wb/BlobProcess.h>
#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/wb/Parameters.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>
#include <opencv_workbench/syllo/syllo.h>

#include <Eigen/Dense>


using std::cout;
using std::endl;

ObjectTracker::ObjectTracker()
{
     next_id_ = 1;
     covar_tracker_.init(2,2);

     left_side_sign_ = 1;

     left_track_valid_ = false;
     right_track_valid_ = false;

     left_tracker_.set_P(60);
     right_tracker_.set_P(60);

     double T = 0.066666667; // dt
     double q= T * 1;
     left_tracker_.set_Q(q);
     right_tracker_.set_Q(q);
}

int ObjectTracker::next_available_id()
{
     return next_id_++;
}

void ObjectTracker::process_frame(cv::Mat &src, cv::Mat &dst,
                                  std::vector<wb::Blob> &meas,
                                  Parameters *params)
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

     wb::Blob left_blob;
     wb::Blob right_blob;

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

               //// If the left and right fin detectors are strong tracks, add
               //// them to the measurements.
               //double left_P_norm = left_tracker_.P().norm();
               //double right_P_norm = right_tracker_.P().norm();
               //
               //if (left_P_norm < params->covar_threshold && 
               //    right_P_norm < params->covar_threshold) {
               //     
               //     // Transform left and right trackers to diver object's
               //     // frame of reference
               //     cv::Point v = it_prev->estimated_pixel_velocity();
               //     cv::Vec2d vel2d(v.x, v.y);
               //     cv::Vec2d vel_unit = vel2d / sqrt(pow(vel2d[0],2) + pow(vel2d[1],2));                                       
               //
               //     {
               //          left_blob.set_age(1);
               //          left_blob.set_id(-1);
               //
               //          // Get relative position to diver's centroid:
               //          // Rotate point based on object's velocity
               //          cv::Point2d rel_p = left_tracker_.position();
               //          double theta = atan2(vel_unit[1], vel_unit[0]);
               //
               //          cv::Point2d rel_rot_p;
               //          rel_rot_p.x = cos(theta) * rel_p.x - sin(theta) * rel_p.y;
               //          rel_rot_p.y = sin(theta) * rel_p.x + cos(theta) * rel_p.y;
               //
               //          cv::Point2d pos;
               //          pos.x = rel_rot_p.x + it_prev->estimated_pixel_centroid().x;
               //          pos.y = rel_rot_p.y + it_prev->estimated_pixel_centroid().y;
               //
               //          left_blob.set_estimated_pixel_centroid(pos);
               //     }
               //
               //     {
               //          right_blob.set_age(1);
               //          right_blob.set_id(-1);
               //
               //          // Get relative position to diver's centroid:
               //          // Rotate point based on object's velocity
               //          cv::Point2d rel_p = right_tracker_.position();
               //          double theta = atan2(vel_unit[1], vel_unit[0]);
               //
               //          cv::Point2d rel_rot_p;
               //          rel_rot_p.x = cos(theta) * rel_p.x - sin(theta) * rel_p.y;
               //          rel_rot_p.y = sin(theta) * rel_p.x + cos(theta) * rel_p.y;
               //
               //          cv::Point2d pos;
               //          pos.x = rel_rot_p.x + it_prev->estimated_pixel_centroid().x;
               //          pos.y = rel_rot_p.y + it_prev->estimated_pixel_centroid().y;
               //
               //          right_blob.set_estimated_pixel_centroid(pos);
               //     }                    
               //
               //     cout << "Using right and left" << endl;
               //                         
               //     track_matches[it_prev->id()].push_back(&left_blob);
               //     track_matches[it_prev->id()].push_back(&right_blob);
               //}
                              
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

                    std::ostringstream convert;
                    convert << (*it_match)->id();
                    const std::string& text = convert.str();
                    cv::putText(img, text, cv::Point((*it_match)->estimated_pixel_centroid().x-3,(*it_match)->estimated_pixel_centroid().y-3), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 1, 8, false);
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
               covar *= 4;

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
                         //eig2 *= 0.90;
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
     //cv::imshow("Intermediate", intermediate_img);

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
     
     //cv::imshow("Used", img);

     // Overlay object track data
     this->overlay(src, dst, TRACKS | IDS | ERR_ELLIPSE | VELOCITIES);

     this->diver_classification(src, dst, meas, params);

     prev_tracks_ = tracks_;
}

bool ObjectTracker::is_diver(std::vector<wb::Entity> &objects, int id)
{
     for (std::vector<wb::Entity>::iterator it_obj = objects.begin();
          it_obj != objects.end(); it_obj++) {
          if (id == it_obj->id()) {
               if (it_obj->type() == wb::Entity::Diver) {
                    return true;
               }
               return false;
          }
     }
     return false;
}

#define RAD_2_DEG (180.0/3.14159)

void ObjectTracker::diver_classification(cv::Mat &src, cv::Mat &dst,
                                         std::vector<wb::Blob> &meas,
                                         Parameters *params)
{
     estimated_divers_.clear();     
#if 1
     //////////////////////////////////////////////////////////////////////////
     // Leg Trackers:
     // Find blobs that are within the "back half" of the ellipse, where the
     // "back half" is the direction away from forward velocity.
     for (std::vector<wb::Blob>::iterator it_obj = tracks_.begin();
          it_obj != tracks_.end(); it_obj++) {

          //cout << "==================================================" << endl;

          // Predict left and right
          left_tracker_.predict();
          right_tracker_.predict();

          bool classified_as_diver = false;
          
          cv::Point track_centroid = it_obj->estimated_pixel_centroid();
          cv::Point v = it_obj->estimated_pixel_velocity();

          cv::Vec3d v3(v.x,v.y,0);

          cv::Vec2d vel2d(v.x, v.y);
          cv::Vec2d vel_unit = vel2d / sqrt(pow(vel2d[0],2) + pow(vel2d[1],2));

          // TODO: Velocity needed?
          // The velocity vector has to be above length threshold
          double v_norm = sqrt(pow(v3[0],2) + pow(v3[1],2));          
          bool has_velocity = false;
          if (v_norm >= params->min_velocity_threshold_2) {          
               //cout << "Too Slow: " << v_norm << endl;
               has_velocity = true;               
          }

          if (has_velocity) {

               // Get major and minor axes of error ellipse
               Ellipse ell = it_obj->error_ellipse(0.9973); // 3 std
          
#if 0
               // Get rectangle around object
               double theta = atan2(vel_unit[1], vel_unit[0]);
               cv::RotatedRect rrect(cv::Point(track_centroid.x,track_centroid.y), cv::Size2f(2*ell.axes()(0),2*ell.axes()(1)), theta * RAD_2_DEG);

               cv::Point2f vertices[4];
               rrect.points(vertices);
               for (int i = 0; i < 4; i++) {
                    cv::line(dst, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
               }
#endif
          
               // Line separating fins:
               cv::Vec2d sep = vel_unit * -1 * ell.axes()(0);// * 4;

               cv::line(dst,cv::Point(track_centroid.x,track_centroid.y),
                        cv::Point(track_centroid.x + sep[0],
                                  track_centroid.y + sep[1]),
                        cv::Scalar(255,77,77), 1, 8, 0);          

               std::vector<wb::Blob> lefts;
               std::vector<wb::Blob> rights;

               std::vector<wb::Blob> lefts_out;
               std::vector<wb::Blob> rights_out;

               for (std::vector<wb::Blob>::iterator it_blob = meas.begin();
                    it_blob != meas.end(); it_blob++) {

                    if (it_blob->occluded()) {
                         continue;
                    }

                    //// TODO: age needed?
                    //if (it_blob->age() < 3) {
                    //     cout << "Too Young" << endl;
                    //     continue;
                    //}

                    cv::Point blob_centroid = it_blob->estimated_pixel_centroid();

                    // Is the blob within the error ellipse?
                    bool in_ellipse = false;
                    if (it_obj->pixel_tracker().is_within_region(blob_centroid,0.9973)) {
                         in_ellipse = true;
                    }

                    // Is the blob on the side of the ellipse, opposite the track's
                    // estimated velocity?

                    // Find blob centroid relative to the track's centroid
                    cv::Vec2d blob_relative(blob_centroid.x - track_centroid.x,
                                            blob_centroid.y - track_centroid.y);

                    // The blob relative norm distance can't be too small.
                    double blob_relative_norm = sqrt(pow(blob_relative[0],2) + pow(blob_relative[1],2));
                    if (blob_relative_norm < 20) {
                         continue;
                    }

                    cv::Vec2d blob_relative_unit = blob_relative / blob_relative_norm;
                    // The dot product of the velocity and the relative position of
                    // the blob's centroid is negative if they are in "opposite
                    // directions"
                    double dot = vel_unit.dot(blob_relative_unit);
                    if (dot >= 0) {
                         continue;
                    }

                    if (in_ellipse) {
                         // Plot the used points on the image
                         wb::drawCross(dst, it_blob->estimated_pixel_centroid(), cv::Scalar(0,0,255), 8);
                    }

                    cv::Vec3d sep_3d(sep[0],sep[1],0);
                    cv::Vec3d blob_relative_unit_3d(blob_relative_unit[0], blob_relative_unit[1], 0);
                    cv::Vec3d cross_value = sep_3d.cross(blob_relative_unit_3d);
                    int cross_value_sign = syllo::sign(cross_value[2]);

                    cv::Point2d rel_p = it_blob->estimated_pixel_centroid() - it_obj->estimated_pixel_centroid();
                    //cv::Point2d rel_p = it_obj->estimated_pixel_centroid() - it_blob->estimated_pixel_centroid();

                    //// Rotate point based on object's velocity
                    double theta = -atan2(vel_unit[1], vel_unit[0]);
                    cv::Point2d rel_rot_p;
                    rel_rot_p.x = cos(theta) * rel_p.x - sin(theta) * rel_p.y;
                    rel_rot_p.y = sin(theta) * rel_p.x + cos(theta) * rel_p.y;

                    it_blob->set_estimated_pixel_centroid(rel_rot_p);
                    it_blob->set_pixel_centroid(rel_rot_p);
                    it_blob->set_estimated_centroid(rel_rot_p);
                    it_blob->set_centroid(rel_rot_p);

                    // If the blob isn't inside of the ellipse, is it within
                    // rectangle behind the object, with bounds defined by
                    // 4xMajorAxis and 2xMinor Axis, rotated to velocity?
                    cv::Vec2d down_vel(0,1);
                    int dir_vel = syllo::sign((down_vel.dot(vel_unit)));               
                                   
                    // Dot product is greater than 0 if the vectors lie in the same
                    // direction:
                    if (!in_ellipse && dir_vel > 0) {                          
                         // Make a copy of the current object track and offset it behind the object:
                         PositionTracker track_behind = it_obj->pixel_tracker();
                         cv::Point2d pt_behind = track_behind.position();        
                         pt_behind.x = pt_behind.x + vel_unit[0] * ell.axes()(0) * -2;
                         pt_behind.y = pt_behind.y + vel_unit[1] * ell.axes()(0) * -2;                    
                         track_behind.set_position(pt_behind);                    
                    
                         // Draw error ellipse
                         Ellipse ell_offset = track_behind.error_ellipse(0.9973); // 3 std
                         cv::Point center = track_behind.position();                    
                         cv::Size axes(cvRound(ell_offset.axes()(0)), cvRound(ell_offset.axes()(1)));
                         cv::ellipse(dst, center, axes, cvRound(ell_offset.angle()), 0, 360, cv::Scalar(255,0,0), 1, 8, 0);
                    
                         if (track_behind.is_within_region(blob_centroid, 0.9973)) {                         
                              in_ellipse = true;
                         }
                    }
               
                    if (cross_value_sign == left_side_sign_) {
                         if (in_ellipse) {
                              lefts.push_back(*it_blob);
                         }
                    } else {
                         if (in_ellipse) {                         
                              rights.push_back(*it_blob);
                         }
                    }

               } // For each blob

               // Using the collected lefts and rights, determine which measurement
               // in each lefts and rights is "closest" to the left and right
               // tracker

               // Find tracks with smallest mahalanobis distance
               double min_dist = 1e9;          
               std::vector<wb::Blob>::iterator it_left = lefts.begin();
               for (std::vector<wb::Blob>::iterator it = lefts.begin();
                    it != lefts.end(); it++) {

                    double mahal_dist = left_tracker_.mahalanobis(it->estimated_pixel_centroid());
                    if (mahal_dist < min_dist) {
                         mahal_dist = min_dist;
                         it_left = it;
                    }                              
               }
          
               min_dist = 1e9;
               std::vector<wb::Blob>::iterator it_right = rights.begin();
               for (std::vector<wb::Blob>::iterator it = rights.begin();
                    it != rights.end(); it++) {
               
                    double mahal_dist = right_tracker_.mahalanobis(it->estimated_pixel_centroid());
                    if (mahal_dist < min_dist) {
                         mahal_dist = min_dist;
                         it_right = it;
                    }               
               }               

               if (lefts.size() > 0) {
                    left_tracker_.set_measurement(it_left->estimated_pixel_centroid());
               }

               if (rights.size() > 0) {
                    right_tracker_.set_measurement(it_right->estimated_pixel_centroid());
               }

               double left_P_norm = left_tracker_.P().norm();
               double right_P_norm = right_tracker_.P().norm();

               //cout << "----------" << endl;
               //cout << "Left Norm: " << left_P_norm << endl;
               //cout << "Right Norm: " << right_P_norm << endl;
          
               if (left_P_norm < params->covar_threshold && 
                   right_P_norm < params->covar_threshold ) {
                   //&& std::abs(left_P_norm - right_P_norm) < params->covar_norm_threshold) {
               
                    it_obj->inc_class_age();
                    it_obj->set_type(wb::Entity::Diver);
                    estimated_divers_.push_back(*it_obj);
                    classified_as_diver = true;
               }
          }

          if (!classified_as_diver) {
               //if (is_diver(prev_estimated_divers_,it_obj->id()) && it_obj->class_age() > params->class_age_confirmed) {
               if (it_obj->class_age() > params->class_age_confirmed) {
                    //it_obj->dec_class_age();
                    it_obj->set_type(wb::Entity::Diver);
                    estimated_divers_.push_back(*it_obj);                    
               }
          }

          // Overlay left tracker...
          if (left_tracker_.initialized()) {
               // Get relative position to diver's centroid:
               // Rotate point based on object's velocity
               cv::Point2d rel_p = left_tracker_.position();
               double theta = atan2(vel_unit[1], vel_unit[0]);

               cv::Point2d rel_rot_p;
               rel_rot_p.x = cos(theta) * rel_p.x - sin(theta) * rel_p.y;
               rel_rot_p.y = sin(theta) * rel_p.x + cos(theta) * rel_p.y;

               cv::Point2d pos;
               pos.x = rel_rot_p.x + it_obj->estimated_pixel_centroid().x;
               pos.y = rel_rot_p.y + it_obj->estimated_pixel_centroid().y;

               // Draw text
               cv::putText(dst, "L", cv::Point(pos.x-3,pos.y-3), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 2, 8, false);

               // Draw centroid
               wb::drawCross(dst, pos, cv::Scalar(212,173,70), 3);

               // Draw error ellipse
               Ellipse ell = left_tracker_.error_ellipse(0.9973); // 3 std
               cv::Point center = pos;
               cv::Size axes(cvRound(ell.axes()(0)), cvRound(ell.axes()(1)));
               cv::ellipse(dst, center, axes, cvRound(ell.angle()), 0, 360, cv::Scalar(255,255,255), 1, 8, 0);               

               // Draw line from tracker to centroid:
               cv::line(dst, center, it_obj->estimated_pixel_centroid(), cv::Scalar(0,255,0), 1, 8, 0);
          }

          // Overlay right tracker...
          if (right_tracker_.initialized()) {
               // Get relative position to diver's centroid:
               // Rotate point based on object's velocity
               cv::Point2d rel_p = right_tracker_.position();
               double theta = atan2(vel_unit[1], vel_unit[0]);

               cv::Point2d rel_rot_p;
               rel_rot_p.x = cos(theta) * rel_p.x - sin(theta) * rel_p.y;
               rel_rot_p.y = sin(theta) * rel_p.x + cos(theta) * rel_p.y;

               cv::Point2d pos;
               pos.x = rel_rot_p.x + it_obj->estimated_pixel_centroid().x;
               pos.y = rel_rot_p.y + it_obj->estimated_pixel_centroid().y;

               // Draw text
               cv::putText(dst, "R", cv::Point(pos.x-3,pos.y-3), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 2, 8, false);

               // Draw centroid
               wb::drawCross(dst, pos, cv::Scalar(255,173,70), 3);               

               // Draw error ellipse
               Ellipse ell = right_tracker_.error_ellipse(0.9973); // 3 std
               cv::Point center = pos;
               cv::Size axes(cvRound(ell.axes()(0)), cvRound(ell.axes()(1)));
               cv::ellipse(dst, center, axes, cvRound(ell.angle()), 0, 360, cv::Scalar(255,255,255), 1, 8, 0);

               // Draw line from tracker to centroid:
               cv::line(dst, center, it_obj->estimated_pixel_centroid(), cv::Scalar(0,255,0), 1, 8, 0);
          }
     } // For each object
#else
     //////////////////////////////////////////////////////////////////////////
     // Velocity Based Detection:
     // Diver objects have a velocity within a threshold
     for (std::vector<wb::Blob>::iterator it_obj = tracks_.begin();
          it_obj != tracks_.end(); it_obj++) {
          
          bool classified_as_diver = false;
          
          cv::Point v = it_obj->estimated_pixel_velocity();

          // The velocity vector has to be above length threshold
          double v_norm = sqrt(pow(v.x,2) + pow(v.y,2));
          //cout << "ID: " << it_obj->id() << ", v: " << v_norm << endl;
          if (v_norm > params->min_velocity_threshold && v_norm < params->max_velocity_threshold) {
               it_obj->inc_class_age();
               it_obj->set_type(wb::Entity::Diver);
               estimated_divers_.push_back(*it_obj);
               classified_as_diver = true;
          } 
          //else {
          // If it was a diver in the past and it's velocity is below a
          // threshold, it is still a diver:
          //if (v_norm < params->max_velocity_threshold && 
          //    is_diver(prev_estimated_divers_,it_obj->id())) {
          
          if (!classified_as_diver) {
               // If it wasn't classified as a diver
               if (it_obj->class_age() > params->class_age_confirmed) {
                    it_obj->set_type(wb::Entity::Diver);
                    estimated_divers_.push_back(*it_obj);
               }    
          }
     }     
#endif
     prev_estimated_divers_ = estimated_divers_;
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
