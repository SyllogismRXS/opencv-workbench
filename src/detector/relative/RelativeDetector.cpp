#include <iostream>
#include "RelativeDetector.h"

#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>

using std::cout;
using std::endl;

RelativeDetector::RelativeDetector()
{
     cout << "RelativeDetector Constructor" << endl;
     thresh_value_ = 255;
     grad_thresh_value_ = 255;
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
     cv::Mat original_w_tracks;          
     original_w_tracks = original;
     
     cv::Mat gray;
     Jet2Gray_matlab(original,gray);
     cv::imshow("gray",gray);        
     
     // Threshold
     cv::Mat thresh;
     //cv::threshold(gray, thresh, 150, 255, cv::THRESH_TOZERO);
     syllo::adaptive_threshold(gray, thresh, thresh_value_, 0.001, 0.002, 10, 5);
     cv::imshow("thresh",thresh);     
     
     // Compute median
     cv::Mat median;
     cv::medianBlur(gray,median,5);
     cv::imshow("median", median);
     
     //cv::Mat grad;
     //cv::Mat kernel;
     //int kernel_size = 10;
     //kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);     
     //cv::filter2D(median, grad, -1, kernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);
     //cv::imshow("filter2d",grad);
     
     // Compute estimated gradient
     cv::Mat grad;     
     syllo::gradient_sobel(median, grad);
     cv::imshow("gradient", grad);
     
     // Only select gradients above a certain threshold
     cv::Mat grad_thresh;
     syllo::adaptive_threshold(grad, grad_thresh, grad_thresh_value_, 0.001, 0.002, 10, 5);
     cv::imshow("gradient_thresh", grad_thresh);
     
     ////// cluster the points
     ////cv::Mat src = grad_thresh;
     ////cv::Mat samples(src.rows * src.cols, 1, CV_32F);
     ////for( int y = 0; y < src.rows; y++ ) {
     ////     for( int x = 0; x < src.cols; x++ ) {
     ////          //for( int z = 0; z < 3; z++) {
     ////          samples.at<float>(y + x*src.rows, 0) = src.at<uchar>(y,x); //src.at<cv::Vec3b>(y,x)[z];
     ////          //}
     ////     }
     ////}
     ////
     ////int clusterCount = 25;
     ////cv::Mat labels;
     ////int attempts = 3;
     ////cv::Mat centers;
     ////cv::kmeans(samples, clusterCount, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers );
     ////
     ////cv::Mat new_image( src.size(), src.type() );
     ////for( int y = 0; y < src.rows; y++ ) {
     ////     for( int x = 0; x < src.cols; x++ ) { 
     ////          int cluster_idx = labels.at<int>(y + x*src.rows,0);
     ////          new_image.at<uchar>(y,x) = centers.at<float>(cluster_idx);
     ////          //new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
     ////          //new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
     ////          //new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
     ////     }
     ////}     
     
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
