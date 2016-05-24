#include <iostream>

#include "FinDetector.h"

#include <opencv_workbench/utils/OpenCV_Helpers.h>
#include <opencv_workbench/syllo/syllo.h>
#include <opencv2/video/video.hpp>

using std::cout;
using std::endl;

FinDetector::FinDetector()
{
     output_file_.open("/home/syllogismrxs/fins.csv");   
     output_file_ << "Frame, ID, X, Y, AGE" << endl;          
     initialized_ = false;
}

#define RAD_2_DEG (180.0/3.14159)

int sign(double v)
{
     if (v >= 0.0) {
          return 1;
     }
     return -1;
}

void FinDetector::process_frame(cv::Mat &gray, cv::Mat &src, cv::Mat &dst, 
                                std::vector<wb::Blob> &tracks, 
                                std::vector<wb::Blob> &blobs,
                                int frame_number,
                                std::vector<wb::Blob> &short_lived)
{               
     dst = src.clone();
     
     if (dst.channels() == 1) {
          cv::cvtColor(dst, dst, CV_GRAY2BGR);     
     }
     
     // Find blobs that are within the "back half" of the ellipse, where the
     // "back half" is the direction away from forward velocity.
     
     for (std::vector<wb::Blob>::iterator it_obj = tracks.begin(); 
          it_obj != tracks.end(); it_obj++) {
          
          cv::Point track_centroid = it_obj->estimated_pixel_centroid();
          cv::Point v = it_obj->estimated_pixel_velocity();          
                    
          cv::Vec3d v3(v.x,v.y,0);
          
          cv::Vec2d vel2d(v.x, v.y);
          cv::Vec2d vel_unit = vel2d / sqrt(pow(vel2d[0],2) + pow(vel2d[1],2));          

          // The velocity vector has to be above length threshold
          double v_norm = sqrt(pow(v3[0],2) + pow(v3[1],0));
          if (v_norm < 2) {
               continue;
          }
          
          cv::Vec3d z_unit(0,0,1);
          cv::Vec3d orth3d = v3.cross(z_unit);
          cv::Vec2d orth2d(cvRound(orth3d[0]),cvRound(orth3d[1]));
          cv::Vec2d orth_unit = orth2d * 1.0 / sqrt(pow(orth2d[0],2) + pow(orth2d[1],2));
          cv::Vec2d orth_20 = orth_unit * 20;

          cv::line(dst,cv::Point(track_centroid.x,track_centroid.y), 
                   cv::Point(track_centroid.x + orth_20[0], 
                             track_centroid.y + orth_20[1]), 
                   cv::Scalar(255,0,0), 1, 8, 0);

          // Get major and minor axes of error ellipse
          Ellipse ell = it_obj->error_ellipse(0.9973); // 3 std
          
          // Get rectangle around object
          double theta = atan2(vel_unit[1], vel_unit[0]);
          cv::RotatedRect rrect(cv::Point(track_centroid.x,track_centroid.y), cv::Size2f(2*ell.axes()(0),2*ell.axes()(1)), theta * RAD_2_DEG);
          
          // Line separating fins:
          cv::Vec2d sep = vel_unit * -1 * ell.axes()(0);

          cv::line(dst,cv::Point(track_centroid.x,track_centroid.y), 
                   cv::Point(track_centroid.x + sep[0], 
                             track_centroid.y + sep[1]), 
                   cv::Scalar(21, 243, 243), 1, 8, 0);
          
          cv::Point2f vertices[4];
          rrect.points(vertices);
          for (int i = 0; i < 4; i++) {
               cv::line(dst, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
          }          
          ////// Rotate image by angle, extract sub image
          //cv::Rect brect = rrect.boundingRect();          
          //cv::Rect rect_mat(0, 0, gray.cols, gray.rows);
          //bool is_inside = (brect & rect_mat) == brect;
          //if (is_inside) {
          //     cv::Mat brect_img(gray, brect);
          //     if (!brect_img.empty()) {
          //          cv::imshow("brect", brect_img);
          //          
          //          // Rotate the image
          //          cv::Point2f pt(brect_img.cols/2.0, brect_img.rows/2.0);    
          //          cv::Mat rot = getRotationMatrix2D(pt, theta * RAD_2_DEG, 1.0);
          //          cv::Mat rot_img;
          //          cv::warpAffine(brect_img, rot_img, rot, cv::Size(brect_img.cols, brect_img.rows));
          //          cv::imshow("Rot", rot_img);
          //     }
          //}
          
          cv::Mat rot = getRotationMatrix2D(track_centroid, theta * RAD_2_DEG, 1.0);
          cv::Mat rot_img;
          cv::warpAffine(gray, rot_img, rot, cv::Size(gray.cols, gray.rows));
          cv::imshow("Rot", rot_img);
          
          cv::RotatedRect rrect2(cv::Point(track_centroid.x,track_centroid.y), cv::Size2f(2*ell.axes()(0),2*ell.axes()(1)), 0);          
          cv::Rect bounding = rrect2.boundingRect();
          if (wb::rect_inside(bounding,rot_img)) {
               cv::Mat rot_roi(rot_img, bounding);
               cv::imshow("rot roi", rot_roi);

               cv::Mat thresh;
               cv::threshold(rot_roi, thresh, 200, 255, cv::THRESH_TOZERO);
               cv::imshow("rot thresh", thresh);               

               if (it_obj->id() == 3) {                    
                    std::ostringstream convert;
                    convert << std::setw(4) << std::setfill('0') << frame_number;        
                    std::string filename = "/home/syllogismrxs/temp/fins/img-" + convert.str() + ".png";
                    cv::imwrite(filename,rot_roi);
               }
          }          
          
          for (std::vector<wb::Blob>::iterator it_blob = blobs.begin();
               it_blob != blobs.end(); it_blob++) {
          
               if (it_blob->occluded()) {
                    continue;
               }
          
               if (it_blob->age() < 3) {
                    continue;
               }
                              
               cv::Point blob_centroid = it_blob->estimated_pixel_centroid();
               
               // Is the blob within the error ellipse?
               if (!it_obj->pixel_tracker().is_within_region(blob_centroid,0.9973)) {
                    continue;
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
          
               // Plot the used points on the image
               wb::drawCross(dst, it_blob->estimated_pixel_centroid(), cv::Scalar(0,0,255), 8);
          
               cv::Vec3d sep_3d(sep[0],sep[1],0);
               cv::Vec3d blob_relative_unit_3d(blob_relative_unit[0], blob_relative_unit[1], 0);
               cv::Vec3d cross_value = sep_3d.cross(blob_relative_unit_3d);
               //cout << cross_value << endl;
               
               int cross_sign = sign(cross_value[2]);
               
               // Is the cross on the left or right side of the segmenting line?
               if (!initialized_) {
                    initialized_ = true;
                    cout << "Side Change" << endl;                    
               } else {
                    if (cross_sign != cross_sign_prev_) {
                         cout << "Side change" << endl;
                    }
               }
               cross_sign_prev_ = cross_sign;
               
               cv::Point rel_p = it_blob->estimated_pixel_centroid() - it_obj->estimated_pixel_centroid();              
               
               //// Rotate point based on object's velocity
               //double theta = -atan2(vel_unit[1], vel_unit[0]);               
               
               double theta = -atan2(vel_unit[1], vel_unit[0]);
               rel_p.x = cos(theta) * rel_p.x - sin(theta) * rel_p.y;
               rel_p.y = sin(theta) * rel_p.x + cos(theta) * rel_p.y;               
               
               if (it_obj->id() == 3) {
                    // Save the points to a text file               
                    output_file_ << syllo::int2str(frame_number) << ", "
                                 << syllo::int2str(it_blob->id()) << ", "
                                 << syllo::double2str(rel_p.x) << ", "
                                 << syllo::double2str(rel_p.y) << ", "
                                 << syllo::int2str(it_blob->age()) << endl;                    
               }
          }                    
     }
}
