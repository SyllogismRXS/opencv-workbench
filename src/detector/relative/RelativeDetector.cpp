#include <iostream>
#include "RelativeDetector.h"

#include <list>

#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/wb/Cluster.h>
#include <opencv_workbench/wb/ClusterProcess.h>
#include <opencv_workbench/wb/NDT.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>

using std::cout;
using std::endl;

RelativeDetector::RelativeDetector()
{
     //cout << "RelativeDetector Constructor" << endl;
     thresh_value_ = 150;
     //thresh_value_ = 255;
     thresh_value_ = 50;
     //grad_thresh_value_ = 255;

     cluster_process_.set_threshold(0);
     cluster_process_.set_gate(25); // 15, 50, 100
     cluster_process_.set_min_cluster_size(30);     

     int erosionElem = cv::MORPH_ELLIPSE;
     int erosionSize = 1;
     int dilationElem = cv::MORPH_ELLIPSE; // MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
     int dilationSize = 1;          

     erosionConfig_ = cv::getStructuringElement( erosionElem,
                                                 cv::Size(2*erosionSize+1, 2*erosionSize+1),
                                                 cv::Point(erosionSize, erosionSize) );
     
     dilationConfig_ = cv::getStructuringElement( dilationElem,
                                                  cv::Size(2*dilationSize+1, 2*dilationSize+1),
                                                  cv::Point(dilationSize, dilationSize) );

     stream_ = NULL;
}

RelativeDetector::~RelativeDetector()
{
     //cout << "RelativeDetector Destructor" << endl;
}

void RelativeDetector::print()
{
     cout << "I am the Relative Detector" << endl;
}

void RelativeDetector::set_stream(syllo::Stream *stream)
{
     stream_ = stream;
}

void RelativeDetector::color_conversion(const cv::Mat &src, cv::Mat &dst)
{
     if (src.channels() != 1) {           
          Jet2Gray_matlab(src,dst);           
     } else {
          dst = src.clone();
     }
}

void RelativeDetector::thresholding(cv::Mat &src, cv::Mat &dst)
{
     switch (params_->threshold_type) {
     case Parameters::static_type:
          cv::threshold(src, dst, params_->static_threshold, 255, cv::THRESH_TOZERO);
          break;
     case Parameters::ratio_type:
          wb::adaptive_threshold(src, dst, thresh_value_, params_->ratio_threshold, 5, mask_);
          break;
     case Parameters::gradient_type:
          //// Compute estimated gradient
          wb::gradient_sobel(src, dst);
          cv::imshow("gradient", dst);                    
          cv::threshold(dst, dst, params_->gradient_threshold, 255, cv::THRESH_TOZERO);
          cv::imshow("grad_thresh", dst);     
          break;
     default:
          break;
     }               
     thresh_img_ = dst.clone();
}

int RelativeDetector::set_frame(int frame_number, const cv::Mat &original)
{            
     cv::Mat original_w_tracks, original_copy;          
     original_w_tracks = original;
     original_copy = original;           
     // //cv::cvtColor(original_copy, original_copy, CV_RGBA2BGR);
     // //wb::show_nonzero(original_copy);
     //  
     // //cv::Mat bad_gray;
     // //cv::applyColorMap(original, bad_gray, CV_BGR2GRAY);
     // //cv::cvtColor(original, bad_gray, CV_BGR2GRAY);
     // //cv::imshow("bad", bad_gray);         
     // 
     cv::Mat gray;
     color_conversion(original, gray);
     wb::get_sonar_mask(original, mask_);

     if (!hide_windows_) {       
          cv::imshow("Gray", gray);  
     }     
     if (last_stage_ < thresh) {
          return 0;
     }
     
     //cv::Mat range_image;
     //stream_->range_image(range_image);     
     //cv::Mat blend;
     //cv::addWeighted(gray, 0.5, range_image, 0.5, 0, blend, gray.depth());     
     
     //cv::Mat flow_img;
     //flow_.sparse_flow(gray, flow_img);          
     //cv::imshow("Flow", flow_img);          
     
     //cv::imshow("Sonar Mask", mask*255);     
     //wb::showHistogram(gray, mask);         
     //wb::opencv_histogram(original_copy);

     //cv::Mat mean_mat, stddev_mat;
     //double mean, stddev;
     //cv::meanStdDev(gray, mean_mat, stddev_mat, mask);
     //mean = mean_mat.at<double>(0,0);
     //stddev = stddev_mat.at<double>(0,0);
     //cout << "Mean: " << mean << endl;
     //cout << "StdDev: " << stddev << endl;
     
     //std::vector<cv::KeyPoint> keypoints_1;
     //int fast_threshold = 40;
     //bool nonmaxSuppression = true;
     //std::vector<cv::Point2f> points1;
     //cv::FAST(gray, keypoints_1, fast_threshold, nonmaxSuppression);
     //cv::KeyPoint::convert(keypoints_1, points1, vector<int>());
     //
     //cv::Mat key_img = gray.clone();
     //cv::cvtColor(key_img, key_img, CV_GRAY2BGR);
     //for (std::vector<cv::Point2f>::iterator it = points1.begin(); 
     //     it != points1.end() ;it++) {
     //     cv::circle(key_img, *it, 1, cv::Scalar(0,255,0), -1, 8, 0);
     //}
     //cv::imshow("key points", key_img);
      
     // Compute median
     cv::Mat median;
     cv::medianBlur(gray, median,5);     
      
     //// Compute estimated gradient
     //cv::Mat grad;     
     //wb::gradient_sobel(median, grad);
     //cv::imshow("gradient", grad);
     //
     //cv::Mat first_level;
     //cv::Mat grad_thresh;
     //wb::multi_otsu(grad, grad_thresh, mask, first_level);     
     //cv::imshow("grad_thresh", grad_thresh);
     
     // 
     // cv::Mat grad_thresh;
     // cv::threshold(grad, grad_thresh, 100, 255, cv::THRESH_TOZERO);
     // cv::imshow("grad_thresh", grad_thresh);     
      
     ///cv::Mat thresh_grad;
     ///wb::adaptive_threshold(grad, thresh_grad, grad_thresh_value_, 0.001, 0.002, 10, 5);
     /// cv::imshow("grad thresh", thresh_grad);        
     
     cv::Mat thresh_amp;      
     thresholding(median, thresh_amp);

     if (!hide_windows_) {       
          cv::imshow("Median", median);
          cv::imshow("Thresh", thresh_amp);
     }     
     if (last_stage_ < cluster) {
          return 0;
     }
     
     //wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.001, 0.002, 1, 5);
     //wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.003, 5, mask);      
     //wb::adaptive_threshold(median, thresh_amp, thresh_value_, params_->ratio_threshold, 5, mask);      
     //wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.003, 0.004, 1, 5, mask);     
     //thresh_img_ = thresh_amp.clone();
     
     //cv::adaptiveThreshold(median, thresh_amp, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 31, 20);
     //cv::threshold(median, thresh_amp, 0, 255, cv::THRESH_TOZERO | cv::THRESH_OTSU);     

     //double val = wb::getThreshVal_Otsu_8u(median, mask);
     //cv::threshold(median, thresh_amp, val, 255, cv::THRESH_TOZERO);

     //cv::Mat first_level;
     //wb::multi_otsu(median, thresh_amp, mask, first_level);     
     //cv::imshow("First", first_level);
     
     /// wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.003, 5, mask);


     //cout << "thresh_value: " << thresh_value_ << endl;     
     
     //cv::Mat grad_plus_thresh = grad_thresh + thresh_amp;
     //cv::imshow("grad_plus_thresh", grad_plus_thresh);
      
     /// cv::Mat thresh_and_grad = thresh_amp + thresh_grad;
     /// cv::imshow("thresh and grad", thresh_and_grad);     
     cv::Mat erode;
     cv::erode(thresh_amp, erode, erosionConfig_);
     //cv::erode(grad_plus_thresh, erode, erosionConfig_);     
      
     cv::Mat dilate;
     cv::dilate(erode, dilate, dilationConfig_);     
     
     cv::Mat ndt_img;
     //ndt_.set_frame(dilate, ndt_img, stream_);
     //ndt_.set_frame(range_image, ndt_img, stream_);
     //cv::imshow("ndt", ndt_img);
      
     blob_process_.set_mask(mask_);
     blob_process_.process_frame(dilate, median, thresh_value_);
     //blob_process_.process_frame(ndt_img, median, thresh_value_);
          
     // Get the current frame's blobs and save to frame's entities
     blob_process_.frame_ents(frame_ents_);
     
     cv::Mat blob_img;     
     blob_process_.overlay(gray, blob_img, BLOBS | RECTS | TRACKS | IDS | ERR_ELLIPSE);
     //blob_process_.overlay(gray, blob_img, BLOBS | RECTS | IDS | ERR_ELLIPSE);     
     
     cv::Mat short_lived;
     blob_process_.overlay_short_lived(gray, short_lived);     
      
     cv::Mat blob_consolidate;
     blob_process_.consolidate_tracks(gray, blob_consolidate);     
     
     cv::Mat original_rects = original.clone();
     blob_process_.overlay(original_rects, original_rects, RECTS | IDS);     
      
     // Add undistorted centroids and compute trajectory analysis
     std::map<int, wb::Blob> tracks_frame;
     std::vector<wb::Blob> blobs = blob_process_.blobs();
     std::vector<wb::Blob>::iterator it = blobs.begin();
     for (; it != blobs.end(); it++) {      
          // Have to transform tracks from distorted cartesian
          // to polar, then to undistorted cartesian
          cv::Point p = it->estimated_centroid();
           
          double x, y;
          if (stream_ != NULL && stream_->type() == syllo::SonarType) {           
               double range = stream_->pixel_range(p.y, p.x); //row, col
               double bearing = stream_->pixel_bearing(p.y, p.x) * 0.017453293; // pi/180               
               x = range*sin(bearing);
               y = -range*cos(bearing);
          } else {
               x = p.x;
               y = p.y;
          }
                      
          it->set_undistorted_centroid(cv::Point2f(x,y));
          it->set_frame(frame_number);
           
          tracks_history_[it->id()].push_back(*it);
          tracks_frame[it->id()] = *it;                     
     }    
     
     // Calculate Similarity between current tracks
     traj_.trajectory_similarity(tracks_history_, frame_number, 
                                 blob_consolidate, 0.017);
     //traj_.trajectory_similarity_track_diff(tracks_frame, frame_number, 
     //                                       blob_consolidate, 0.02);          
     
     //////////////////////////////////////////////////////////////
     /// Diver Detections / Object Labelling
     ////////////////////////////////////////////////////
     blob_process_.displace_detect(params_->history_length,
                                   params_->history_distance);
     
     // Copy track detections to tracks_ object
     tracks_.clear(); // clear out the tracks from previous loop      
     blob_process_.blobs_to_entities(tracks_);
      
     ///////////////////////////////////////////////////
     // Display images
     ///////////////////////////////////////////////////
     if (!hide_windows_) {       
          //cv::imshow("Range", range_image);
          //cv::imshow("blend", blend);
          cv::imshow("erode", erode);
          cv::imshow("Dilate", dilate);      
          cv::imshow("Blobs", blob_img);        
          cv::imshow("Tracking Tracks",short_lived);
          cv::imshow("Consolidate", blob_consolidate);      
          cv::imshow("Tracks", original_rects);          
          cv::imshow("Traj", blob_consolidate);
     }
     
     return 0;
}

extern "C" {
     Detector *maker(){
          return new RelativeDetector;
     }

     class proxy {
     public:
          proxy(){
               // register the maker with the factory
               // causes static initialization error plugin_manager_.factory["blank_detector"] = maker;
               plugin_manager_.factory["relative_detector"] = maker;
          }
     };
     // our one instance of the proxy
     proxy p;
}

// // "GOLDEN" set_frame
// int RelativeDetector::set_frame(int frame_number, const cv::Mat &original)
// {               
//      cv::Mat original_w_tracks, original_copy;          
//      original_w_tracks = original;
//      original_copy = original;
//       
//      //cv::cvtColor(original_copy, original_copy, CV_RGBA2BGR);
//      //wb::show_nonzero(original_copy);
//       
//      //cv::Mat bad_gray;
//      //cv::applyColorMap(original, bad_gray, CV_BGR2GRAY);
//      //cv::cvtColor(original, bad_gray, CV_BGR2GRAY);
//      //cv::imshow("bad", bad_gray);         
//      
//      cv::Mat gray;
//      if (original.channels() != 1) {           
//           Jet2Gray_matlab(original,gray);           
//      } else {
//           gray = original.clone();
//      }
//      cv::imshow("Gray", gray);   
//      
//      //cv::Mat flow_img;
//      //flow_.sparse_flow(gray, flow_img);          
//      //cv::imshow("Flow", flow_img);     
// 
//      cv::Mat mask;
//      wb::get_sonar_mask(original, mask);
//      //cv::imshow("Sonar Mask", mask*255);     
//      //wb::showHistogram(gray, mask);
//            
//      //cv::Mat ndt_img;
//      //ndt_.set_frame(gray, ndt_img);
//      //cv::imshow("ndt", ndt_img);
//       
//      // Compute median
//      cv::Mat median;
//      cv::medianBlur(gray, median,5);
//      cv::imshow("median", median);          
//       
//      // // Compute estimated gradient
//      // cv::Mat grad;     
//      // wb::gradient_sobel(median, grad);
//      // cv::imshow("gradient", grad);
//      // 
//      // cv::Mat grad_thresh;
//      // cv::threshold(grad, grad_thresh, 100, 255, cv::THRESH_TOZERO);
//      // cv::imshow("grad_thresh", grad_thresh);     
//       
//      ///cv::Mat thresh_grad;
//      ///wb::adaptive_threshold(grad, thresh_grad, grad_thresh_value_, 0.001, 0.002, 10, 5);
//      /// cv::imshow("grad thresh", thresh_grad);        
//      
//      cv::Mat thresh_amp;      
//      wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.001, 0.002, 1, 5);
//      //cout << "thresh_value: " << thresh_value_ << endl;
//      cv::imshow("thresh amp", thresh_amp);
// 
//      //cv::Mat grad_plus_thresh = grad_thresh + thresh_amp;
//      //cv::imshow("grad_plus_thresh", grad_plus_thresh);
//       
//      /// cv::Mat thresh_and_grad = thresh_amp + thresh_grad;
//      /// cv::imshow("thresh and grad", thresh_and_grad);     
//      cv::Mat erode;
//      cv::erode(thresh_amp, erode, erosionConfig_);
//      //cv::erode(grad_plus_thresh, erode, erosionConfig_);
//      cv::imshow("erode", erode);
//       
//      cv::Mat dilate;
//      cv::dilate(erode, dilate, dilationConfig_);
//      cv::imshow("Dilate", dilate);      
//       
//      blob_process_.process_frame(dilate, median, thresh_value_);
//       
//      cv::Mat blob_img;
//      //blob_process_.overlay_blobs(gray, blob_img);            
//      //blob_process_.overlay_tracks(blob_img, blob_img);
//      blob_process_.overlay(gray, blob_img, BLOBS | RECTS | TRACKS | IDS | ERR_ELLIPSE);
//      //blob_process_.overlay(gray, blob_img, BLOBS | RECTS | IDS | ERR_ELLIPSE);
//      cv::imshow("Blobs", blob_img);        
// 
//      cv::Mat short_lived;
//      blob_process_.overlay_short_lived(gray, short_lived);
//      cv::imshow("Tracking Tracks",short_lived);
//       
//      cv::Mat blob_consolidate;
//      blob_process_.consolidate_tracks(gray, blob_consolidate);
//      cv::imshow("Consolidate", blob_consolidate);      
//      
//      cv::Mat original_rects = original.clone();
//      blob_process_.overlay(original_rects, original_rects, RECTS | IDS);
//      cv::imshow("Tracks", original_rects);
//      
//      //////////////////////////////////////////////////////////////
//      /// Tracking     
//      ////////////////////////////////////////////////////
//      tracks_.clear(); // clear out the tracks from previous loop
//       
//      std::map<int, wb::Blob> tracks_frame;
//      std::vector<wb::Blob> blobs = blob_process_.blobs();
//      std::vector<wb::Blob>::iterator it = blobs.begin();
//      for (; it != blobs.end(); it++) {
//       
//           // Have to transform tracks from distorted cartesian
//           // to polar, then to undistorted cartesian
//           cv::Point p = it->estimated_centroid();
//            
//           double x, y;
//           if (stream_ != NULL && stream_->type() == syllo::SonarType) {           
//                double range = stream_->pixel_range(p.y, p.x); //row, col
//                double bearing = stream_->pixel_bearing(p.y, p.x) * 0.017453293; // pi/180
//                y = -range*cos(bearing);
//                x = range*sin(bearing);
//           } else {
//                x = p.x;
//                y = p.y;
//           }
//                       
//           it->set_undistorted_centroid(cv::Point2f(x,y));
//           it->set_frame(frame_number);
//            
//           tracks_history_[it->id()].push_back(*it);
//           tracks_frame[it->id()] = *it;
//            
//           tracks_.push_back(*it);
//      }
//       
//      // Calculate Similarity between current tracks
//      traj_.trajectory_similarity(tracks_history_, frame_number, 
//                                  blob_consolidate, 0.017);
//      //traj_.trajectory_similarity_track_diff(tracks_frame, frame_number, 
//      //                                       blob_consolidate, 0.02);
//      
//      cv::imshow("Traj", blob_consolidate);
//       
//      ///////////////////////////////////////////////////
//      // Display images
//      ///////////////////////////////////////////////////
//      if (!hide_windows_) {          
//      }
//      
//      return 0;
// }

// Contours and gradients
// // Compute median
//      cv::Mat median;
//      cv::medianBlur(gray, median,5);
//      cv::imshow("median", median);          
//      
//      // Compute estimated gradient
//      cv::Mat grad;     
//      wb::gradient_sobel(median, grad);
//      cv::imshow("sobel gradient", grad);
// 
//      cv::Mat grad_thresh;
//      cv::threshold(grad, grad_thresh, 150, 255, cv::THRESH_TOZERO);
//      cv::imshow("grad thresh", grad_thresh);          
//      
//      cv::Mat thresh_amp;      
//      wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.001, 0.002, 1, 5);
//      cv::imshow("thresh amp", thresh_amp);
//      
//      //cv::Mat grad_plus_thresh = grad_thresh + thresh_amp;
//      cv::Mat grad_plus_thresh;
//      cv::addWeighted( thresh_amp, 0.5, grad_thresh, 0.5, 0, grad_plus_thresh );
//      cv::imshow("grad_plus_thresh", grad_plus_thresh);
//       
//      cv::Mat erode;
//      cv::erode(grad_plus_thresh, erode, erosionConfig_);
//      cv::imshow("erode", erode);
// 
//      cv::Mat dilate;
//      cv::dilate(erode, dilate, dilationConfig_);
//      cv::imshow("Dilate", dilate);      
// 
//      blob_process_.process_frame(dilate, median, thresh_value_);
//       
//      cv::Mat blob_img;
//      blob_process_.overlay(gray, blob_img, RECTS | TRACKS | IDS);
//      cv::imshow("Blobs", blob_img);        
// 
//      cv::Mat blob_consolidate;
//      blob_process_.consolidate_tracks(gray, blob_consolidate);
//      cv::imshow("Consolidate", blob_consolidate);      
//      
//      ///// Apply Laplace function
//      //cv::Mat abs_dst;
//      //int kernel_size = 3;
//      //int scale = 1;
//      //int delta = 0;
//      //int ddepth = CV_16S;
//      //cv::Laplacian( median, abs_dst, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT );
//      //cv::convertScaleAbs( abs_dst, abs_dst );
//      //cv::imshow("laplacian gradient", abs_dst);
//      // 
//      //cv::Mat lapl_thresh;
//      //cv::threshold(abs_dst, lapl_thresh, 100, 255, cv::THRESH_TOZERO);
//      //cv::imshow("lapl_thresh", lapl_thresh);          
//      //
//      //cv::Mat grad_simple;
//      //wb::gradient_simple(median,grad_simple);
//      //cv::imshow("simple gradient", grad_simple);
// 
//      cv::Mat canny_output;
//      std::vector< std::vector<cv::Point> > contours;
//      std::vector<cv::Vec4i> hierarchy;
//      
//      /// Detect edges using canny
//      int thresh = 150;
//      cv::Canny( median, canny_output, thresh, thresh*2, 3 );
//      /// Find contours
//      cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
//      
//      /// Draw contours
//      cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
//      for( unsigned int i = 0; i< contours.size(); i++ )
//      {
//           //cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//           cv::Scalar color = cv::Scalar(255,255,255);
//           cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
//      }
//      
//      /// Show in a window
//      cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//      cv::imshow( "Contours", drawing );

/// Color map tests
///cv::Mat grad;
///create_gradient(grad, 100, 255);
///cv::imshow("gradient", grad);
///
///cv::Mat jet_matlab;
///Gray2Jet_matlab(grad,jet_matlab);
///cv::imshow("jet matlab", jet_matlab);
///
///cv::Mat gray_2;
///Jet2Gray_matlab(jet_matlab,gray_2);
///cv::imshow("gray_2",gray_2);
///
///cv::Mat diff = gray_2 - grad;
///cv::imshow("diff",diff);
///
///if (!equal(gray_2, grad)) {
///     cout << "Grays are unequal" << endl;
///}     
