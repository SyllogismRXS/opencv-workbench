#include <iostream>
#include "RelativeDetector.h"

#include <list>

#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/wb/Cluster.h>
#include <opencv_workbench/wb/ClusterProcess.h>
#include <opencv_workbench/wb/NDT.h>

using std::cout;
using std::endl;

RelativeDetector::RelativeDetector()
{
     cout << "RelativeDetector Constructor" << endl;
     thresh_value_ = 255;
     grad_thresh_value_ = 255;

     cluster_process_.set_threshold(0);
     cluster_process_.set_gate(50); // 15, 100
     cluster_process_.set_min_cluster_size(30);     
}

RelativeDetector::~RelativeDetector()
{
     cout << "RelativeDetector Destructor" << endl;
}

void RelativeDetector::print()
{
     cout << "I am the Relative Detector" << endl;
}

int RelativeDetector::set_frame(int frame_number, const cv::Mat &original)
{         
     cv::Mat original_w_tracks, original_copy;          
     original_w_tracks = original;
     original_copy = original;
     
     //cv::Mat bad_gray;
     ////cv::applyColorMap(original, bad_gray, CV_BGR2GRAY);
     //cv::cvtColor(original, bad_gray, CV_BGR2GRAY);
     //cv::imshow("bad", bad_gray);
     
     cv::Mat gray;
     Jet2Gray_matlab(original,gray);
     cv::imshow("gray",gray); 

     //cv::Mat ndt_img;
     //ndt_.set_frame(gray, ndt_img);
     //cv::imshow("ndt", ndt_img);
     
     // Compute median
     cv::Mat median;
     cv::medianBlur(gray, median,5);
     cv::imshow("median", median);          
     
     //// Compute estimated gradient
     //cv::Mat grad;     
     //wb::gradient_sobel(median, grad);
     //cv::imshow("gradient", grad);

     cv::Mat thresh_amp;
     wb::adaptive_threshold(median, thresh_amp, thresh_value_, 0.001, 0.002, 10, 5);
     cv::imshow("thresh amp", thresh_amp);
     
     //cv::Mat grad;     
     //wb::gradient_simple(median, grad);
     //cv::imshow("gradient", grad);
     //
     //// Threshold
     //cv::Mat thresh;
     //wb::adaptive_threshold(grad, thresh, grad_thresh_value_, 0.001, 0.002, 10, 5);
     ////wb::adaptive_threshold(median, thresh, thresh_value_, 0.003, 0.005, 10, 5);
     ////wb::adaptive_threshold(median, thresh, thresh_value_, 0.002, 0.003, 10, 5);
     //cv::imshow("thresh",thresh);    

     //cv::Mat thresh_plus_grad = thresh_amp + thresh;
     //cv::imshow("thrush_plus_grad", thresh_plus_grad);
     
     cluster_process_.process_frame(thresh_amp);

     cv::Mat cluster_img;
     cluster_process_.overlay_clusters(gray, cluster_img);
     cv::imshow("clusters", cluster_img);
     
     // cv::Mat cluster_tracks_img;
     // cluster_process_.overlay_tracks(cluster_img, cluster_tracks_img);
     // cv::imshow("tracks", cluster_tracks_img);
     
     //int gate = 15;
     //int min_cluster_size = 30;
     //std::list<wb::Cluster*> clusters;
     //wb::cluster_points(thresh_amp, clusters, 0, gate, min_cluster_size);  
     //
     //cv::Mat cluster_img;
     //wb::draw_clusters(original_copy, cluster_img, clusters);
     //cv::imshow("clusters", cluster_img);
     
     //////////////////////////////////////////////////////////////
     /// Tracking     
     tracks_.clear(); // clear out the tracks from previous loop
     
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
