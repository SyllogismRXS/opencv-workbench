#include <iostream>
#include "DisplaceDetector.h"

using std::cout;
using std::endl;

DisplaceDetector::DisplaceDetector()
{
     cout << "DisplaceDetector Constructor" << endl;

     int erosionElem = cv::MORPH_ELLIPSE;
     int erosionSize = 2;
     int dilationElem = cv::MORPH_ELLIPSE; // MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
     int dilationSize = 1;          

     erosionConfig_ = cv::getStructuringElement( erosionElem,
                                                 cv::Size(2*erosionSize+1, 2*erosionSize+1),
                                                 cv::Point(erosionSize, erosionSize) );
     
     dilationConfig_ = cv::getStructuringElement( dilationElem,
                                                  cv::Size(2*dilationSize+1, 2*dilationSize+1),
                                                  cv::Point(dilationSize, dilationSize) );
}

DisplaceDetector::~DisplaceDetector()
{
     cout << "DisplaceDetector Destructor" << endl;
}

void DisplaceDetector::print()
{
     cout << "I am the Displace Detector" << endl;
}

int DisplaceDetector::set_frame(int frame_number, const cv::Mat &original)
{
     cv::Mat original_w_tracks;
     cv::Mat gray;
     cv::Mat threshold;
     cv::Mat erode;
     cv::Mat dilate;     
     
     original_w_tracks = original;
     
     cv::cvtColor(original, gray, CV_BGR2GRAY);          
     cv::threshold(gray, threshold, 200, 255, cv::THRESH_TOZERO);
     syllo::RunningGaussian(0.5, threshold, rg_, avgs_, variance_);
     cv::erode(rg_, erode, erosionConfig_);
     cv::dilate(erode, dilate, dilationConfig_);
     
     ////////////////////////////
     // Blob Detector / Tracker
     ////////////////////////////	  
     cv::Mat blob_img;

     std::map<int,syllo::Blob> blobs;
     syllo::DetectBlobs(dilate, blob_img, blobs);
	  
     syllo::CentroidsOfBlobs(blob_img, blobs);
	  
     std::map<int,syllo::Blob> new_blobs;
     syllo::BlobMatch(prev_blobs_, blobs, new_blobs, frame_number);
     prev_blobs_ = new_blobs;
	  
     cv::normalize(blob_img, blob_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
     cv::applyColorMap(blob_img, blob_img, cv::COLORMAP_JET);
	  
     std::map<int,syllo::Cluster> clusters;
     syllo::formClusters(new_blobs, clusters);
	  
     std::map<int,syllo::Cluster> new_clusters;
     syllo::ClusterMatch(prev_clusters_, clusters, new_clusters, 
                         all_clusters_, frame_number, track_ids_); 
     prev_clusters_ = new_clusters;
	  
     drawClusters(blob_img, blob_img, new_clusters, 20);
     drawClusters(original_w_tracks, original_w_tracks, new_clusters, 20);                    

     /////////////////////////////////////////////////////
     // Determine which track is diver based on which track has moved
     // the farthest
     ////////////////////////////////////////////////////
     tracks_.clear(); // clear out the tracks from previous loop

     syllo::Cluster farthest;
     int ID = syllo::getFarthestTravel(new_clusters, farthest);
     std::map<int,syllo::Cluster>::iterator it = all_clusters_.begin();
     for (; it != all_clusters_.end(); it++) {
          syllo::Track track;
          cv::Point point2d = it->second.getCentroid();          
          track.set_position(cv::Point3d(point2d.x, point2d.y, 0));
          track.set_id(it->first);
          track.set_age(it->second.getAge());
          
          if (it->first == ID) {
               // This is the predicted diver
               track.set_type(syllo::Diver);
          } else {
               // Not the predicted diver
               track.set_type(syllo::Unknown);
          }
          tracks_.push_back(track);
     }
     
     ///////////////////////////////////////////////////
     // Display images
     ///////////////////////////////////////////////////
     //cv::imshow("Original", original);
     //cv::imshow("Gray", gray);
     cv::imshow("Threshold", threshold);
     cv::imshow("Running", rg_);
     cv::imshow("Clusters", blob_img);
     cv::imshow("Tracks", original_w_tracks);
     
     return 0;
}

extern "C" {
     Detector *maker(){
          return new DisplaceDetector;
     }

     class proxy {
     public:
          proxy(){
               // register the maker with the factory
               // causes static initialization error plugin_manager_.factory["blank_detector"] = maker;
               plugin_manager_.factory["displace_detector"] = maker;
          }
     };
     // our one instance of the proxy
     proxy p;
}
