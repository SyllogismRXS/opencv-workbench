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

          //cascade = cvLoadHaarClassifierCascade( "/home/syllogismrxs/Documents/scuba-faces-db/processing/haarcascade.xml", cvSize( width, height ) );
          cascade = cvLoadHaarClassifierCascade( "/home/syllogismrxs/Documents/scuba-faces-db/processing/haarcascade.xml", cvSize( 0,0 ) );
          if( cascade == NULL )
          {
               printf( "Unable to load classifier\n" );

               return 1;
          }

          int* numclassifiers = new int[cascade->count];
          numclassifiers[0] = cascade->stage_classifier[0].count;
          for( int i = 1; i < cascade->count; i++ )
          {
               numclassifiers[i] = numclassifiers[i-1] + cascade->stage_classifier[i].count;
          }

          storage = cvCreateMemStorage();

          int nos = -1, nos0;
          nos0 = cascade->count;
          if( nos <= 0 ){
               nos = nos0;
          }

          printf( "Number of stages: %d\n", nos );
          printf( "Number of weak classifiers: %d\n", numclassifiers[nos - 1] );
          
          
          //if( !face_cascade.load( "/home/syllogismrxs/Documents/scuba-faces-db/processing/haarcascade.xml" ) ) { 
          //     printf("--(!)Error loading\n"); 
          //     return -1;
          //} else {
          //     printf("Loaded cascade file.\n");
          //}

          ////-- 1. Load the cascades
          //if( !face_cascade.load( "../data/haarcascades/haarcascade_frontalface_alt.xml" ) ) { 
          //     printf("--(!)Error loading\n"); 
          //     return -1;
          //}
          //
          //if( !eyes_cascade.load( "../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ) ) { 
          //     printf("--(!)Error loading\n"); 
          //     return -1; 
          //}

          rng = new cv::RNG(12345);

          return 0;
     }

     void Chain::DetectFace( cv::Mat &frame, cv::Mat &dest )
     {
          std::vector<cv::Rect> faces;
          cv::Mat frame_gray;
          
          cv::Mat original_mat = frame.clone();
          //IplImage orig = original_mat;

          cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
          cv::equalizeHist( frame_gray, frame_gray );

          cvClearMemStorage( storage );                   

          IplImage img = frame_gray;
          double scale_factor = 1.2;
          CvSeq* objects = cvHaarDetectObjects( &img, cascade, storage, scale_factor, 1 );
          int detcount;
          ObjectPos* det;
          detcount = ( objects ? objects->total : 0);
          cout << "Objects: " << detcount << endl;
          det = (detcount > 0) ?
               ( (ObjectPos*)cvAlloc( detcount * sizeof( *det )) ) : NULL;

          for( int i = 0; i < detcount; i++ )
          {
               CvAvgComp r = *((CvAvgComp*) cvGetSeqElem( objects, i ));
               det[i].x = 0.5F * r.rect.width  + r.rect.x;
               det[i].y = 0.5F * r.rect.height + r.rect.y;
               det[i].width = sqrtf( 0.5F * (r.rect.width * r.rect.width +
                                             r.rect.height * r.rect.height) );
               det[i].neghbors = r.neighbors;

               //cvRectangle( &orig, cvPoint( r.rect.x, r.rect.y ),
               //             cvPoint( r.rect.x + r.rect.width, r.rect.y + r.rect.height ),
               //             CV_RGB( 255, 0, 0 ), 3 );
               //cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
               //cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
               cv::rectangle(original_mat,cv::Point( r.rect.x, r.rect.y ),cv::Point( r.rect.x + r.rect.width, r.rect.y + r.rect.height ),cv::Scalar(0,0,255),1,8,0);
               
          }
          dest = original_mat;
          //dest = &img;

          // //-- Detect faces
          // face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
          // 
          // cout << "Faces Detected: " << faces.size() << endl;
          // for( unsigned int i = 0; i < faces.size(); i++ )
          // {
          //      cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
          //      cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
          // 
          //      cv::Mat faceROI = frame_gray( faces[i] );
          //      std::vector<cv::Rect> eyes;
          //      
          //      //-- In each face, detect eyes
          //      //eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
          //      
          //      //for( unsigned int j = 0; j < eyes.size(); j++ )
          //      //{
          //      //     cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
          //      //     int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
          //      //     cv::circle( frame, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
          //      //}
          // }
          // dest = frame;
          // ////-- Show what you got
          // //imshow( window_name, frame );
     }


     int Chain::process(const cv::Mat &src, cv::Mat &dest)
     {
          //cv::threshold(src, dest, 200, 255, cv::THRESH_TOZERO);
          //double scale = 0.15;
          double scale = 1;
          cv::resize(src,dest,cv::Size(0,0),scale,scale,cv::INTER_NEAREST);

          this->DetectFace(dest,dest);
          
          //cv::Mat input = src;
          //this->DetectFace(input, dest);
          return 0;
     }

     int Chain::draw_rectangle(const cv::Mat &src, cv::Mat &dest, 
                               cv::Point pt1, cv::Point pt2)
     {
          dest = src;
          cv::rectangle(dest,pt1,pt2,cv::Scalar(0,0,255),1,8,0);
          return 0;
     }
     
     int Chain::crop_image(const cv::Mat &src, cv::Mat &dest, cv::Rect rect)
     {
          dest = cv::Mat(src, rect);
          return 0;
     }

     int Chain::save_image(const std::string &fn, const cv::Mat &img)
     {
          std::vector<int> compression_params;
          compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
          compression_params.push_back(9);

          cv::imwrite(fn,img, compression_params);
          return 0;
     }
     
     int Chain::gray_and_equalize(const cv::Mat &src, cv::Mat &dest)
     {
          /// Convert to grayscale
          cv::cvtColor( src, dest, CV_BGR2GRAY );

          /// Apply Histogram Equalization
          cv::equalizeHist( dest, dest );

          return 0;
     }

}
