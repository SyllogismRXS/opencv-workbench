#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/syllo.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     cout << "Diver Detection" << endl;     

     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input-file>" << endl;
          return -1;
     }
     
     syllo::Stream stream;
     syllo::Status status = stream.open(argv[1]);

     if (status != syllo::Success) {
          cout << "Failed to open: " << argv[1] << endl;
          return -1;
     }
     
     cv::Mat original;
     cv::Mat gray;
     cv::Mat threshold;     
     
     // Running Gaussian Frames
     cv::Mat rg;
     cv::Mat avgs;
     cv::Mat variance;

     // Erosion / dilation
     cv::Mat erode;
     cv::Mat dilate;

     int erosionElem = cv::MORPH_ELLIPSE;
     int erosionSize = 2;
     int dilationElem = cv::MORPH_ELLIPSE; // MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
     int dilationSize = 1;          

     cv::Mat erosionConfig = cv::getStructuringElement( erosionElem,
							cv::Size(2*erosionSize+1, 2*erosionSize+1),
							cv::Point(erosionSize, erosionSize) );

     cv::Mat dilationConfig = cv::getStructuringElement( dilationElem,
							 cv::Size(2*dilationSize+1, 2*dilationSize+1),
							 cv::Point(dilationSize, dilationSize) );
     
     // Blob / Cluster tracking
     cv::Mat blob_img;    
     std::map<int,syllo::Blob> prev_blobs;
     std::map<int,syllo::Cluster> prev_clusters;
     std::map<int,syllo::Cluster> all_clusters;
     std::map<int,bool> track_ids;
     
     int frame_num = 0;
     while (stream.read(original)) {          

          cv::cvtColor(original, gray, CV_BGR2GRAY);          
          //cv::threshold(gray, threshold, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
          //cv::threshold(gray, threshold, 200, 255, cv::THRESH_TOZERO | CV_THRESH_OTSU);
          cv::threshold(gray, threshold, 200, 255, cv::THRESH_TOZERO);
          
          syllo::RunningGaussian(0.5, threshold, rg, avgs, variance);	  
          
          cv::erode(rg, erode, erosionConfig);	 
          cv::dilate(erode, dilate, dilationConfig);	  
                              
          //cv::imshow("Erode", erode);
          //cv::imshow("Dilate", dilate);          
          
          ////////////////////////////
	  // Blob Detector / Tracker
	  ////////////////////////////	  
	  std::map<int,syllo::Blob> blobs;
	  int blob_clount = syllo::DetectBlobs(rg, blob_img, blobs);
	  
	  syllo::CentroidsOfBlobs(blob_img, blobs);
	  
	  std::map<int,syllo::Blob> new_blobs;
	  syllo::BlobMatch(prev_blobs, blobs, new_blobs, frame_num);
	  prev_blobs = new_blobs;
	  
	  cv::normalize(blob_img, blob_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	  cv::applyColorMap(blob_img, blob_img, cv::COLORMAP_JET);
	  
          std::map<int,syllo::Cluster> clusters;
	  syllo::formClusters(new_blobs, clusters);
	  
	  std::map<int,syllo::Cluster> new_clusters;
	  syllo::ClusterMatch(prev_clusters, clusters, new_clusters, 
                              all_clusters, frame_num, track_ids); 
	  prev_clusters = new_clusters;
	  
	  drawClusters(blob_img, blob_img, new_clusters, 20);	 
          
          // Display images
          cv::imshow("Original", original);
          cv::imshow("Gray", gray);
          cv::imshow("Threshold", threshold);
          cv::imshow("Running", rg);
          cv::imshow("Clusters", blob_img);

          if(cv::waitKey(10) >= 0) break;
          
          frame_num++;
     }
     
     return 0;
}
