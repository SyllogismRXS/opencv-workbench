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
     cout << "RelativeDetector Constructor" << endl;
     thresh_value_ = 150;
     thresh_value_ = 255;
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
     cout << "RelativeDetector Destructor" << endl;
}

void RelativeDetector::print()
{
     cout << "I am the Relative Detector" << endl;
}

void RelativeDetector::set_stream(syllo::Stream *stream)
{
     stream_ = stream;
}

int RelativeDetector::set_frame(int frame_number, const cv::Mat &original)
{               
     cv::Mat original_w_tracks, original_copy;          
     original_w_tracks = original;
     original_copy = original;
      
     //cv::cvtColor(original_copy, original_copy, CV_RGBA2BGR);
     //wb::show_nonzero(original_copy);
      
     //cv::Mat bad_gray;
     //cv::applyColorMap(original, bad_gray, CV_BGR2GRAY);
     //cv::cvtColor(original, bad_gray, CV_BGR2GRAY);
     //cv::imshow("bad", bad_gray);         
     
     cv::Mat gray;
     if (original.channels() != 1) {           
          Jet2Gray_matlab(original,gray);           
     } else {
          gray = original.clone();
     }
     cv::imshow("Gray", gray);

     cv::Mat mask;
     wb::get_sonar_mask(original, mask);
     //cv::imshow("Sonar Mask", mask*255);     
     //wb::showHistogram(gray, mask);
           
     //cv::Mat ndt_img;
     //ndt_.set_frame(gray, ndt_img);
     //cv::imshow("ndt", ndt_img);
      
     // Compute median
     cv::Mat median;
     cv::medianBlur(gray, median,5);
     cv::imshow("median", median);          
      
     /// // Compute estimated gradient
     /// cv::Mat grad;     
     /// wb::gradient_sobel(median, grad);
     /// cv::imshow("gradient", grad);
     /// 
     ///cv::Mat thresh_grad;
     ///wb::adaptive_threshold(grad, thresh_grad, grad_thresh_value_, 0.001, 0.002, 10, 5);
     /// cv::imshow("grad thresh", thresh_grad);        
     
     cv::Mat thresh_amp;      
     wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.001, 0.002, 1, 5);
     //cout << "thresh_value: " << thresh_value_ << endl;
     cv::imshow("thresh amp", thresh_amp);
      
     /// cv::Mat thresh_and_grad = thresh_amp + thresh_grad;
     /// cv::imshow("thresh and grad", thresh_and_grad);     
     cv::Mat erode;
     cv::erode(thresh_amp, erode, erosionConfig_);
     cv::imshow("erode", erode);
      
     cv::Mat dilate;
     cv::dilate(erode, dilate, dilationConfig_);
     cv::imshow("Dilate", dilate);      
      
     blob_process_.process_frame(dilate, median, thresh_value_);
      
     cv::Mat blob_img;
     //blob_process_.overlay_blobs(gray, blob_img);            
     //blob_process_.overlay_tracks(blob_img, blob_img);
     blob_process_.overlay(gray, blob_img, BLOBS | RECTS | TRACKS | IDS | ERR_ELLIPSE);
     cv::imshow("Blobs", blob_img);        
      
     cv::Mat blob_consolidate;
     blob_process_.consolidate_tracks(gray, blob_consolidate);
     cv::imshow("Consolidate", blob_consolidate);      
     
     cv::Mat original_rects = original.clone();
     blob_process_.overlay(original_rects, original_rects, RECTS | IDS);
     cv::imshow("Tracks", original_rects);
     
     //////////////////////////////////////////////////////////////
     /// Tracking     
     ////////////////////////////////////////////////////
     tracks_.clear(); // clear out the tracks from previous loop
      
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
               y = -range*cos(bearing);
               x = range*sin(bearing);
          } else {
               x = p.x;
               y = p.y;
          }
                      
          it->set_undistorted_centroid(cv::Point2f(x,y));
          it->set_frame(frame_number);
           
          tracks_history_[it->id()].push_back((wb::Entity)*it);
           
          tracks_.push_back((wb::Entity)*it);
     }
      
     // Calculate Similarity between current tracks
     //this->trajectory_similarity(frame_number, blob_consolidate);
     traj_.trajectory_similarity(tracks_history_, frame_number, 
                                 blob_consolidate, 0.017);
      
     ///////////////////////////////////////////////////
     // Display images
     ///////////////////////////////////////////////////
     if (!hide_windows_) {          
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
