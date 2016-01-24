#include <iostream>
#include "StationarySonarDetector.h"

#include <list>

#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/wb/Cluster.h>
#include <opencv_workbench/wb/ClusterProcess.h>
#include <opencv_workbench/wb/NDT.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>

using std::cout;
using std::endl;

StationarySonarDetector::StationarySonarDetector()
{
     cout << "StationarySonarDetector Constructor" << endl;
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

     //bgs = new MultiLayerBGS;
     
     stream_ = NULL;
}

StationarySonarDetector::~StationarySonarDetector()
{
     cout << "StationarySonarDetector Destructor" << endl;
}

void StationarySonarDetector::print()
{
     cout << "I am the StationarySonar Detector" << endl;
}

void StationarySonarDetector::set_stream(syllo::Stream *stream)
{
     stream_ = stream;
}

int StationarySonarDetector::set_frame(int frame_number, const cv::Mat &original)
{               
     cv::Mat original_w_tracks, original_copy;          
     original_w_tracks = original;
     original_copy = original;           
     
     cv::Mat gray;
     if (original.channels() != 1) {           
          Jet2Gray_matlab(original,gray);           
     } else {
          gray = original.clone();
     }
     cv::imshow("Gray", gray);  

     cv::Mat gray_low_thresh;
     cv::threshold(gray, gray_low_thresh, 50, 255, cv::THRESH_TOZERO);
     cv::imshow("Low Thresh", gray_low_thresh);
     
     cv::Mat rg_img;
     rg_.process(gray_low_thresh, rg_img);
     cv::imshow("RG", rg_img);                         
     
     cv::Mat mask;
     wb::get_sonar_mask(original, mask);               
      
     //// Compute median
     //cv::Mat median;
     //cv::medianBlur(gray, median,5);
     //cv::imshow("median", median);                
     // // Compute estimated gradient
     // cv::Mat grad;     
     // wb::gradient_sobel(median, grad);
     // cv::imshow("gradient", grad);
     // 
     // cv::Mat grad_thresh;
     // cv::threshold(grad, grad_thresh, 100, 255, cv::THRESH_TOZERO);
     // cv::imshow("grad_thresh", grad_thresh);           
     ///cv::Mat thresh_grad;
     ///wb::adaptive_threshold(grad, thresh_grad, grad_thresh_value_, 0.001, 0.002, 10, 5);
     /// cv::imshow("grad thresh", thresh_grad);             
     //cv::Mat thresh_amp;      
     //wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.001, 0.002, 1, 5);
     ////cout << "thresh_value: " << thresh_value_ << endl;
     //cv::imshow("thresh amp", thresh_amp);         
           
     cv::Mat erode;
     cv::erode(rg_img, erode, erosionConfig_);
     //cv::erode(grad_plus_thresh, erode, erosionConfig_);
     cv::imshow("erode", erode);
      
     cv::Mat dilate;
     cv::dilate(erode, dilate, dilationConfig_);
     cv::imshow("Dilate", dilate);      
     // 
     // //cv::Mat ndt_img;
     // ////ndt_.set_frame(dilate, ndt_img, stream_);
     // //ndt_.set_frame(range_image, ndt_img, stream_);
     // //cv::imshow("ndt", ndt_img);
     //  
     blob_process_.process_frame(dilate, rg_img, thresh_value_);
       
     cv::Mat blob_img;
     //blob_process_.overlay(gray, blob_img, BLOBS | RECTS | TRACKS | IDS | ERR_ELLIPSE);     
     blob_process_.overlay(gray, blob_img, BLOBS);     
     cv::imshow("Blobs", blob_img);        
     
     // TODO: LOOK AT DISPLACE DETECTOR'S SETTINGS.
     // Maybe use clustering?

     // 
     // cv::Mat short_lived;
     // blob_process_.overlay_short_lived(gray, short_lived);
     // cv::imshow("Tracking Tracks",short_lived);
     //  
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
           
          tracks_.push_back(*it);
     }
      
     // // Calculate Similarity between current tracks
     // traj_.trajectory_similarity(tracks_history_, frame_number, 
     //                             blob_consolidate, 0.017);
     // //traj_.trajectory_similarity_track_diff(tracks_frame, frame_number, 
     // //                                       blob_consolidate, 0.02);
     
     //cv::imshow("Traj", blob_consolidate);
      
     ///////////////////////////////////////////////////
     // Display images
     ///////////////////////////////////////////////////
     if (!hide_windows_) {          
     }
     
     return 0;
}

extern "C" {
     Detector *maker(){
          return new StationarySonarDetector;
     }

     class proxy {
     public:
          proxy(){
               // register the maker with the factory
               // causes static initialization error plugin_manager_.factory["blank_detector"] = maker;
               plugin_manager_.factory["stationarysonar_detector"] = maker;
          }
     };
     // our one instance of the proxy
     proxy p;
}

