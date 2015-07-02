#include <iostream>
#include "BlankDetector.h"

using std::cout;
using std::endl;

BlankDetector::BlankDetector()
{
     cout << "BlankDetector Constructor" << endl;
}

BlankDetector::~BlankDetector()
{
     cout << "BlankDetector Destructor" << endl;
}

void BlankDetector::print()
{
     cout << "I am the Blank Detector" << endl;
}

int BlankDetector::set_frame(int frame_number, const cv::Mat &img)
{
     //cv::imshow("Original", img);
     //cv::waitKey(1);
     return 0;
}

extern "C" {
     Detector *maker(){
          return new BlankDetector;
     }

     class proxy {
     public:
          proxy(){
               // register the maker with the factory
               // causes static initialization error plugin_manager_.factory["blank_detector"] = maker;
               plugin_manager_.factory["blank_detector"] = maker;
          }
     };
     // our one instance of the proxy
     proxy p;
}
