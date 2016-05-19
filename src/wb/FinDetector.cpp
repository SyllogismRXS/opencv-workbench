#include <iostream>

#include "FinDetector.h"

#include <opencv_workbench/utils/OpenCV_Helpers.h>

using std::cout;
using std::endl;

FinDetector::FinDetector()
{
     
}

void FinDetector::process_frame(cv::Mat &src, cv::Mat &dst, 
                                std::vector<wb::Blob> &tracks, 
                                std::vector<wb::Blob> &blobs)
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
          
          for (std::vector<wb::Blob>::iterator it_blob = blobs.begin();
               it_blob != blobs.end(); it_blob++) {
          
               if (it_blob->occluded()) {
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
               if (blob_relative_norm < 10) {
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

               wb::drawCross(dst, it_blob->estimated_pixel_centroid(), cv::Scalar(0,0,255), 8);
          }                    
     }
}
