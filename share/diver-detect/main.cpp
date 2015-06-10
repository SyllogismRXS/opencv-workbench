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

#include <opencv_workbench/utils/AnnotationParser.h>

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

     // Setup Annotation Parser
     AnnotationParser parser;
     parser.CheckForFile(argv[1], AnnotationParser::track);
     parser.clear();
     parser.set_width(stream.width());
     parser.set_height(stream.height());
     parser.set_depth(3);
     parser.set_type("video");
     parser.set_number_of_frames(stream.get_frame_count());
     
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
     
     int frame_number = 0;
     while (stream.read(original)) {          

          original_w_tracks = original;
          
          cv::cvtColor(original, gray, CV_BGR2GRAY);          
          cv::threshold(gray, threshold, 200, 255, cv::THRESH_TOZERO);
          
          syllo::RunningGaussian(0.5, threshold, rg, avgs, variance);

          cv::erode(rg, erode, erosionConfig);	 
          cv::dilate(erode, dilate, dilationConfig);	  
                              
          ////////////////////////////
	  // Blob Detector / Tracker
	  ////////////////////////////	  
	  std::map<int,syllo::Blob> blobs;
	  syllo::DetectBlobs(dilate, blob_img, blobs);
	  
	  syllo::CentroidsOfBlobs(blob_img, blobs);
	  
	  std::map<int,syllo::Blob> new_blobs;
	  syllo::BlobMatch(prev_blobs, blobs, new_blobs, frame_number);
	  prev_blobs = new_blobs;
	  
	  cv::normalize(blob_img, blob_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	  cv::applyColorMap(blob_img, blob_img, cv::COLORMAP_JET);
	  
          std::map<int,syllo::Cluster> clusters;
	  syllo::formClusters(new_blobs, clusters);
	  
	  std::map<int,syllo::Cluster> new_clusters;
	  syllo::ClusterMatch(prev_clusters, clusters, new_clusters, 
                              all_clusters, frame_number, track_ids); 
	  prev_clusters = new_clusters;
	  
	  drawClusters(blob_img, blob_img, new_clusters, 20);
          drawClusters(original_w_tracks, original_w_tracks, new_clusters, 20);
          
          ///////////////////////////////////////////////////
          // Save Track Information in AnnotationParser
          ///////////////////////////////////////////////////
          Frame frame;
          frame.set_frame_number(frame_number);

          // Loop through all clusters
          
          //std::map<int,bool>::iterator it = track_ids.begin();
          std::map<int,syllo::Cluster>::iterator it = new_clusters.begin();
          for (; it != new_clusters.end(); it++) {
               Object object;
               object.set_name(syllo::int2str(it->first));               

               syllo::Cluster c = it->second;               
               object.bbox = BoundingBox(c.getCentroid().x, 
                                         c.getCentroid().x,
                                         c.getCentroid().y, 
                                         c.getCentroid().y);
               
               // Save object to current frame
               frame.objects[object.name()] = object;           
          }
          // Save frame to parser
          parser.frames[frame_number] = frame;
          
          ///////////////////////////////////////////////////
          // Display images
          ///////////////////////////////////////////////////
          cv::imshow("Original", original);
          cv::imshow("Gray", gray);
          cv::imshow("Threshold", threshold);
          cv::imshow("Running", rg);
          cv::imshow("Clusters", blob_img);
          cv::imshow("Tracks", original_w_tracks);

          if(cv::waitKey(10) >= 0) break;
          
          frame_number++;
     }

     cout << "Saving tracks to xml file" << endl;
     parser.write_annotation();
     
     return 0;
}
