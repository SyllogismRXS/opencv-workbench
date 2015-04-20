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
     cout << "Diver Detection - Segmentation" << endl;     

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
     cv::Mat original_w_tracks;
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
          original_w_tracks = original.clone();                    
                    
          ////////////////////////////
          // Original Preprocessing
          ////////////////////////////
          cv::cvtColor(original, gray, CV_BGR2GRAY);          
          cv::threshold(gray, threshold, 200, 255, cv::THRESH_TOZERO);
                    
          cv::dilate(threshold, dilate, dilationConfig);
                              
          ////////////////////////////
	  // Blob Detector / Tracker
	  ////////////////////////////	  
	  std::map<int,syllo::Blob> blobs;
	  syllo::DetectBlobs(dilate, blob_img, blobs);
          
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
          drawClusters(original_w_tracks, original_w_tracks, new_clusters, 20);
          
          
          ///////////////////////////////////////////////
          /// Image Segmentation
          ///////////////////////////////////////////////
          cv::Mat1b markers(original.rows, original.cols);
          // let's set all of them to possible background first
          markers.setTo(cv::GC_PR_BGD);

          //// cut out a small area in the middle of the image          
          //int m_rows = 0.1 * original.rows;
          //int m_cols = 0.1 * original.cols;
          //// of course here you could also use cv::Rect() instead of cv::Range to select 
          //// the region of interest
          //cv::Mat1b fg_seed = markers(cv::Range(original.rows/2 - m_rows/2, original.rows/2 + m_rows/2), 
          //                            cv::Range(original.cols/2 - m_cols/2, original.cols/2 + m_cols/2));
          // cut out a small area in the middle of the image

          int fg_rows = 20;
          int fg_cols = 20;
          if (new_clusters.size() > 1 && new_clusters[1].getCentroid().y > fg_rows && new_clusters[1].getCentroid().x > fg_cols) {
               
               int fg_center_row = new_clusters[1].getCentroid().y;
               int fg_center_col = new_clusters[1].getCentroid().x;

               // of course here you could also use cv::Rect() instead of cv::Range to select 
               // the region of interest
               cv::Mat1b fg_seed = markers(cv::Range(fg_center_row - fg_rows/2, fg_center_row + fg_rows/2),
                                           cv::Range(fg_center_col - fg_cols/2, fg_center_col + fg_cols/2));
          
               // mark it as foreground
               fg_seed.setTo(cv::GC_FGD);
          
               // select first 1 row of the image as background
               cv::Mat1b bg_seed = markers(cv::Range(0, 1),cv::Range::all());
               bg_seed.setTo(cv::GC_BGD);

               cv::Mat bgd, fgd;
               int iterations = 3;
               cv::grabCut(original, markers, cv::Rect(), bgd, fgd, iterations, cv::GC_INIT_WITH_MASK);          
          
               // let's get all foreground and possible foreground pixels
               cv::Mat1b mask_fgpf = ( markers == cv::GC_FGD) | ( markers == cv::GC_PR_FGD);
               // and copy all the foreground-pixels to a temporary image
               cv::Mat3b tmp = cv::Mat3b::zeros(original.rows, original.cols);
               original.copyTo(tmp, mask_fgpf);
               // show it
               cv::imshow("foreground", tmp);
          }
          
          ////////////////////////////////////////////////
          // Display images
          ////////////////////////////////////////////////
          cv::imshow("Original", original);
          cv::imshow("Gray", gray);
          cv::imshow("Threshold", threshold);
          cv::imshow("Clusters", blob_img);
          cv::imshow("Tracks", original_w_tracks);

          if(cv::waitKey(10) >= 0) break;
          
          frame_num++;
     }
     
     return 0;
}
