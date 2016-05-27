#include <iostream>

#include "DiverClassifier.h"

#include <opencv_workbench/utils/OpenCV_Helpers.h>
#include <opencv_workbench/syllo/syllo.h>

using std::cout;
using std::endl;

DiverClassifier::DiverClassifier()
{
     
}

void DiverClassifier::process_frame(cv::Mat &gray, 
                                    std::vector<wb::Blob> &tracks, 
                                    std::vector<wb::Blob> &blobs,
                                    int frame_number)
{
     estimated_divers_.clear();
     
     // Simple detection: Diver objects have a velocity within a threshold
     for (std::vector<wb::Blob>::iterator it_obj = tracks.begin(); 
          it_obj != tracks.end(); it_obj++) {

          cv::Point v = it_obj->estimated_pixel_velocity();          
                              
          // The velocity vector has to be above length threshold
          double v_norm = sqrt(pow(v.x,2) + pow(v.y,2));
          cout << "ID: " << it_obj->id() << ", v: " << v_norm << endl;
          if (v_norm > 9 && v_norm < 20) {
               it_obj->set_type(wb::Entity::Diver);
               estimated_divers_.push_back(*it_obj);
          }
     }
}
