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
     syllo::Stream stream;

     if (argc < 3) {
          cout << "Usage: " << argv[0] << " <hog-model.svm> <video-file>" 
               << endl;
          return -1;
     }

     // Load HOG model
     typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<6> > image_scanner_type; 
     dlib::object_detector<image_scanner_type> detector;
     dlib::deserialize(argv[1]) >> detector;
          
     // Open video stream and process each frame
     syllo::Status status = stream.open(argv[2]);    
     if (status != syllo::Success) {
          cout << "Failed to open: " << argv[2] << endl;
          return -1;
     }
     
     bool single_step = false;
     
     cv::VideoWriter out;     
     
     cv::Mat original;     
     //stream.set_frame_number(5380); // for: 2014_05_02_16_01_53.avi
     while (stream.read(original)) {
     
          if (!out.isOpened()) {
               out.open("/home/syllogismrxs/output.avi",CV_FOURCC('M','J','P','G'),stream.get_fps(),original.size(),true);
               if (!out.isOpened()) {
                    cout << "Unable to open output video" << endl;
                    return -1;
               }
          }
          
          cv::pyrUp( original, original, cv::Size( original.cols*2, original.rows*2 ));
          
          dlib::cv_image<dlib::bgr_pixel> cimg(original);
          //dlib::pyramid_up(cimg);
          //dlib::pyramid_up(cimg);
          
          std::vector<dlib::rectangle> dets = detector(cimg);
          std::vector<dlib::rectangle>::iterator it = dets.begin();
          for (; it != dets.end(); it++) {
               cv::rectangle(original, cv::Point(it->left(),it->top()), 
                             cv::Point(it->right(), it->bottom()), 
                             cv::Scalar(0,0,0), 4, 8, 0);
          }

          cv::pyrDown(original, original, cv::Size(original.cols/2,original.rows/2));
          cv::imshow("Original", original);
          out << original;
          
          int wait = 1;
          if (single_step) {
               wait = 0;
          }

          int key = cv::waitKey(wait);
          if (key == 'q') {
               break;
          } else if(key == 'p') {
               single_step = !single_step;
          }
     }
     
     cout << "Clean exit." << endl;
     
     return 0;
}
