#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <boost/graph/adjacency_list.hpp> 
#include <boost/graph/graphviz.hpp>

#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/wb/Blob.h>
#include <opencv_workbench/wb/BlobProcess.h>
#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>

using std::cout;
using std::endl;

using namespace boost; 

int main(int argc, char * argv[]) {
     cv::Mat img = cv::Mat::zeros(200,300,CV_8UC3);
     
     std::vector<wb::Blob> measurements;
     std::vector<wb::Blob> tracks;
     std::vector<wb::Blob> fused_tracks;
     
#if 1
     // Using two "established" tracks
     wb::Blob t1;
     t1.set_id(1);
     wb::Point p1;
     p1.set_position(cv::Point(100,100));
     p1.set_value(255);
     t1.add_point(p1);
     t1.set_frame(10);
     t1.init();
     t1.set_age(10);     
     t1.set_R(0);
     t1.set_P(1500);
     t1.set_prob(1);

     tracks.push_back(t1);          

     // Using one "measurements"
     wb::Blob m1;
     m1.set_id(11);     
     wb::Point p3;
     p3.set_position(cv::Point(150,100));
     p3.set_value(255);
     m1.add_point(p3);
     m1.set_frame(10);
     m1.init();
     m1.set_age(1);

     measurements.push_back(m1);

#else
     // Using two "established" tracks
     wb::Blob t1;
     t1.set_id(1);
     wb::Point p1;
     p1.set_position(cv::Point(100,100));
     p1.set_value(255);
     t1.add_point(p1);
     t1.set_frame(10);
     t1.init();
     t1.set_age(10);     
     t1.set_R(0);
     t1.set_P(1500);
     t1.set_prob(1);

     wb::Blob t2;
     t2.set_id(2);
     wb::Point p2;
     p2.set_position(cv::Point(200,150));
     p2.set_value(255);
     t2.add_point(p2);
     t2.set_frame(10);
     t2.init();
     t2.set_age(10);
     t2.set_R(0);
     t2.set_P(1500);  
     t1.set_prob(2);

     tracks.push_back(t1);
     tracks.push_back(t2);         

     // Using three "measurements"
     wb::Blob m1;
     m1.set_id(11);     
     wb::Point p3;
     p3.set_position(cv::Point(150,100));
     p3.set_value(255);
     m1.add_point(p3);
     m1.set_frame(10);
     m1.init();
     m1.set_age(1);

     wb::Blob m2;
     m2.set_id(12);     
     wb::Point p4;
     p4.set_position(cv::Point(250,100));
     p4.set_value(255);
     m2.add_point(p4);
     m2.set_frame(10);
     m2.init();
     m2.set_age(1);

     wb::Blob m3;
     m3.set_id(13);     
     wb::Point p5;
     p5.set_position(cv::Point(225,125));
     p5.set_value(255);
     m3.add_point(p5);
     m3.set_frame(10);
     m3.init();
     m3.set_age(1);

     measurements.push_back(m1);
     measurements.push_back(m2);
     measurements.push_back(m3);
     
#endif         
     
     // Plot the measurements
     for (std::vector<wb::Blob>::iterator it = measurements.begin();
          it != measurements.end(); it++) {
          cv::Point center = it->centroid();
          img.at<cv::Vec3b>(center.y,center.x) = cv::Vec3b(255,0,0);
          wb::drawTriangle(img, center, cv::Scalar(0,255,0), 7);
     }

     BlobProcess blob_process;
     blob_process.assign_mht(measurements, tracks, fused_tracks);

     // Plot the tracks 
     BlobProcess blob_plot;
     blob_plot.set_blobs(tracks);
     blob_plot.overlay(img, img, IDS|TRACKS|ERR_ELLIPSE);     

     cv::resize(img, img, cv::Size(0,0),3,3,cv::INTER_NEAREST);     
     cv::imshow("Tracks", img);
     cv::waitKey(0);
     
     return 0;
}
