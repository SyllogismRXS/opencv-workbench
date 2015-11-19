#include <iostream>
#include <fstream>

#include <opencv_workbench/utils/Stream.h>

#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/data_io.h>

#include <dlib/opencv.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <dlib/image_processing/frontal_face_detector.h>
//#include <dlib/image_processing/render_face_detections.h>
//#include <dlib/image_processing.h>
//#include <dlib/gui_widgets.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <hog-model.svm>" << endl;
          return -1;
     }

     // Load HOG model
     typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<6> > image_scanner_type; 
     dlib::object_detector<image_scanner_type> detector;
     dlib::deserialize(argv[1]) >> detector;
          
     dlib::image_window hogwin(draw_fhog(detector), "Learned fHOG detector");
     
     std::string str;
     std::cin >> str;
     
     return 0;
}
