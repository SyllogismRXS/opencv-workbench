#include <iostream>

// OpenCV headers
#include <cv.h>
#include <highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "RunningGaussian.h"

using std::cout;
using std::endl;

RunningGaussian::RunningGaussian()
{
     
}

int RunningGaussian::process(cv::Mat &img1, cv::Mat &dst)
{

if (avg_img_.empty()) {
avg_img_ = cv::Mat::zeros(img1.size(), CV_8UC1);
var_img_ = cv::Mat::zeros(img1.size(), CV_8UC1);
}

dst = cv::Mat::zeros(img1.size(), CV_8UC1);
     double alpha = 0.2;
     double k = 5;
     
     cv::Mat tempImg = img1.clone();               

     int channels = tempImg.channels();
     int nRows = tempImg.rows;
     int nCols = tempImg.cols * channels;
     if (tempImg.isContinuous()) {
          nCols *= nRows;
          nRows = 1; 
     }
     
     int i,j;
     uchar *p1, *p2, *p3, *p4;
     uchar diff;
     for( i = 0; i < nRows; ++i) {
          p1 = tempImg.ptr<uchar>(i);
          p2 = avg_img_.ptr<uchar>(i);
          p3 = dst.ptr<uchar>(i);
          p4 = var_img_.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               p2[j] = alpha*p1[j] + (1-alpha)*p2[j];

               diff = abs(p1[j] - p2[j]);
               p4[j] = alpha*diff*diff + (1-alpha)*p4[j];

               if ( abs(p1[j] - p2[j]) > 50) {
                    //if ( abs(p1[j] - p2[j]) > 25) {
                    //if ( abs(p1[j] - p2[j]) > k*sqrt(p4[j])) {
                    //p3[j] = p1[j];
                    p3[j] = 255;
               }
          }
     }          
     return 0;
}
