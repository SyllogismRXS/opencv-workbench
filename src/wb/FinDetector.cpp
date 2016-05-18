#include <iostream>

#include "FinDetector.h"

using std::cout;
using std::endl;

FinDetector::FinDetector()
{
     
}

void FinDetector::process_frame(cv::Mat &src, cv::Mat &dst, 
                                std::vector<wb::Blob> &tracks, 
                                std::vector<wb::Blob> &blobs)
{
     dst = src.clone();
     
     // Find blobs that are within the "back half" of the ellipse, where the
     // "back half" is the direction away from forward velocity.
     
     for (std::vector<wb::Blob>::iterator it_obj = tracks.begin(); 
          it_obj != tracks.end(); it_obj++) {
          // Determine the back half of the object's ellipse
          
     }
}
