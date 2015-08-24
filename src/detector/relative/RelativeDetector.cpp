#include <iostream>
#include "RelativeDetector.h"

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
     cv::Mat gray;
     cv::Mat median;
     
     original_w_tracks = original;
     
     cv::cvtColor(original, gray, CV_BGR2GRAY);          
     //cv::threshold(gray, threshold, 200, 255, cv::THRESH_TOZERO);

     // Compute median
     cv::medianBlur(gray,median,5);
     
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
     
     // Only select gradients above a certain threshold
     cv::threshold(grad, grad_thresh, 175, 255, cv::THRESH_TOZERO);
     
     
     //////////////////////////////////////////////////////////////
     /// Tracking     
     tracks_.clear(); // clear out the tracks from previous loop
     
     ///////////////////////////////////////////////////
     // Display images
     ///////////////////////////////////////////////////
     if (!hide_windows_) {
          cv::imshow("Original", original);
          cv::imshow("gray", gray);
          cv::imshow("median", median);
          cv::imshow("gradient", grad);
          cv::imshow("gradient threshold", grad_thresh);
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
