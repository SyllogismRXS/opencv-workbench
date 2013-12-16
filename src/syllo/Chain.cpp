#include <iostream>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "Chain.h"

using std::cout;
using std::endl;

namespace syllo {     

     Chain::Chain()
     {          
     }

     Chain::Chain(const std::string &filename)
     {
          this->LoadFile(filename);
     }

     int Chain::LoadFile(const std::string &filename)
     {
#if 0
          config_ = YAML::LoadFile(filename);


          YAML::Node algorithms = config_["Algorithms"];
          if (algorithms.IsSequence()) {
               cout << "A sequence was found!" << endl;               
               for(std::size_t i = 0; i < algorithms.size(); i++) {
                    cout << algorithms[i]["name"].as<std::string>() << endl;
                    if (algorithms[i]["name"].as<std::string>() == "Threshold") {
                         cout << "\tThresh: " << algorithms[i]["value"].as<int>() << endl;
                    }
               }
          } else {
               cout << "No seq" << endl;
          }
#endif

          //-- 1. Load the cascades
          if( !face_cascade.load( "../data/haarcascades/haarcascade_frontalface_alt.xml" ) ) { 
               printf("--(!)Error loading\n"); 
               return -1;
          }
          
          if( !eyes_cascade.load( "../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ) ) { 
               printf("--(!)Error loading\n"); 
               return -1; 
          }

          rng = new cv::RNG(12345);

          return 0;
     }

     void Chain::DetectFace( cv::Mat &frame, cv::Mat &dest )
     {
          std::vector<cv::Rect> faces;
          cv::Mat frame_gray;

          cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
          cv::equalizeHist( frame_gray, frame_gray );

          //-- Detect faces
          face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

          for( unsigned int i = 0; i < faces.size(); i++ )
          {
               cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
               cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
          
               cv::Mat faceROI = frame_gray( faces[i] );
               std::vector<cv::Rect> eyes;
               
               //-- In each face, detect eyes
               eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
               
               for( unsigned int j = 0; j < eyes.size(); j++ )
               {
                    cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
                    int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
                    cv::circle( frame, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
               }
          }
          dest = frame;
          ////-- Show what you got
          //imshow( window_name, frame );
     }


     int Chain::Process(const cv::Mat &src, cv::Mat &dest)
     {
          //cv::threshold(src, dest, 200, 255, cv::THRESH_TOZERO);
          cv::Mat input = src;
          this->DetectFace(input, dest);
          return 0;
     }
}
