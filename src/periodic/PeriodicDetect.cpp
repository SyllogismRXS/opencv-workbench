#include <iostream>

#include "PeriodicDetect.h"

using std::cout;
using std::endl;

PeriodicDetect::PeriodicDetect()
{
     
}

void PeriodicDetect::add_frame(const cv::Mat &img)
{
     cv::Mat img_mod;
     cv::cvtColor(img, img_mod, CV_BGR2GRAY);
     cv::normalize(img_mod, img_mod, 0, 255, CV_MINMAX, CV_8UC1);
     frames_.push_back(img_mod);
}

int PeriodicDetect::similarity(cv::Mat &img1, cv::Mat &img2)
{
     int sum = 0;

     //cv::Mat output;
     //cv::filter2D(img1, output, -1, img2, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
     //
     //int channels = output.channels();
     //int nRows = output.rows;
     //int nCols = output.cols * channels;
     //if (output.isContinuous()) {
     //	  nCols *= nRows;
     //	  nRows = 1; 
     //}
     //int i,j;
     //uchar *p1, *p2;
     //for( i = 0; i < nRows; ++i) {
     //	  p1 = output.ptr<uchar>(i);
     //	  for ( j = 0; j < nCols; ++j) {
     //	       sum += p1[j];
     //	  }
     //}

     int channels = img1.channels();
     int nRows = img1.rows;
     int nCols = img1.cols * channels;
     if (img1.isContinuous()) {
     	  nCols *= nRows;
     	  nRows = 1; 
     }
     int i,j;
     uchar *p1, *p2;
     for( i = 0; i < nRows; ++i) {
     	  p1 = img1.ptr<uchar>(i);
     	  p2 = img2.ptr<uchar>(i);
     	  for ( j = 0; j < nCols; ++j) {
     	       sum += abs( p1[j] - p2[j] );
     	  }
     }
     return sum;
}

void PeriodicDetect::recurrence()
{
     int num_frames = frames_.size();
     
     // Now calculate Recurrence plot
     cv::Mat rp = cv::Mat::zeros(num_frames, num_frames, CV_8UC1);
     uchar *p;
     for (int i = 0 ; i < num_frames ; i++) {
     	  p = rp.ptr<uchar>(i);
     	  for (int j = 0 ; j < num_frames ; j++) {
     	       int sum = similarity(frames_[i], frames_[j]);
     	       p[j] = sum;
     	  }
     }
     cv::imshow("Recurrence Plot",rp);
     
     cv::Mat rp_norm = rp.clone();
     cv::normalize(rp_norm, rp_norm, 0, 255, CV_MINMAX, CV_8UC1);
     cv::imshow("Normalized", rp_norm);
     
     cv::Mat rp_norm_thresh = rp_norm.clone();
     cv::threshold( rp_norm_thresh, rp_norm_thresh, 230, 255, cv::THRESH_TOZERO);
     cv::imshow("Norm,Thresh", rp_norm_thresh);
     cv::waitKey();
}
