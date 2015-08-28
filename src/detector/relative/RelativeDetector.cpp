#include <iostream>
#include "RelativeDetector.h"

#include <opencv_workbench/utils/ColorMaps.h>

using std::cout;
using std::endl;

RelativeDetector::RelativeDetector()
{
     cout << "RelativeDetector Constructor" << endl;
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
     cv::threshold(gray, thresh, 150, 255, cv::THRESH_TOZERO);
     cv::imshow("thresh",thresh);
     
     // Compute median
     cv::Mat median;
     cv::medianBlur(gray,median,3);
     cv::imshow("median", median);
     
     // Compute estimated gradient
     cv::Mat grad_x, grad_y;
     cv::Mat abs_grad_x, abs_grad_y;
     cv::Mat grad;
     cv::Mat grad_thresh;
     
     int scale = 1;
     int delta = 0;
     int ddepth = CV_16S;
     
     /// Gradient X
     cv::Sobel( median, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
     /// Gradient Y
     cv::Sobel( median, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
     
     cv::convertScaleAbs( grad_x, abs_grad_x );
     cv::convertScaleAbs( grad_y, abs_grad_y );
     
     cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
     cv::imshow("gradient", grad);
     
     // Only select gradients above a certain threshold
     cv::threshold(grad, grad_thresh, 150, 255, cv::THRESH_TOZERO);
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
