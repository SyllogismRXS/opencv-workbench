#ifndef STREAM_H_
#define STREAM_H_

#include <cv.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <algorithm>
#include <string>

using std::cout;
using std::endl;

#include "Sonar.h"

namespace syllo
{
     enum Status{
          Success = 0,
          Failure
     };

     typedef enum{
	  NoneType = 0,
	  CameraType,
	  MovieType,
	  SonarType,
          ImageType
     }Stream_t;

     typedef enum{
          Recorded = 0,
          Live = 1
     }Live_t;

     typedef enum{
          AVI = 0,
          MP4,
          MPEG_1,
          MOTION_JPG
     }Codec_t;

     class Stream {
     protected:
	  cv::VideoCapture vcap;
          cv::VideoWriter *output_;
          int output_count_;
          int end_frame_;
	  sonar::Sonar sonar;
	  Stream_t type;
          Live_t live_;
          cv::Mat frame_;          
          std::string img_fn_;
          bool valid_img_;
     public:

          int select_codec(Codec_t codec)
          {
               switch(codec) {
               case AVI:
                    return CV_FOURCC('M','J','P','G');
                    //return cv::VideoWriter::fourcc('M','J','P','G');
                    //return cv::VideoWriter::fourcc('I', 'Y', 'U', 'V');
                    break;
               case MP4:
                    //return cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
                    return CV_FOURCC('X', 'V', 'I', 'D');
                    break;
               case MPEG_1:
                    //return cv::VideoWriter::fourcc('P','I','M','1');
                    return CV_FOURCC('P','I','M','1');
                    break;
               case MOTION_JPG:
                    //return cv::VideoWriter::fourcc('M','J','P','G');
                    return CV_FOURCC('M','J','P','G');
                    break;
               default:
                    //return cv::VideoWriter::fourcc('I', 'Y', 'U', 'V');
                    return CV_FOURCC('I', 'Y', 'U', 'V');
               }
          }

          Status open(int num)
	  {
	       vcap.open(num);
	       //vcap.set(cv::CAP_PROP_FRAME_WIDTH, 480);
	       //vcap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
               vcap.set(CV_CAP_PROP_FRAME_WIDTH, 480);
	       vcap.set(CV_CAP_PROP_FRAME_HEIGHT, 640);
	       type = CameraType;
               live_ = Live;

               if (vcap.isOpened()) {
                    return Success;
               } else {                    
                    return Failure;
               }
	  }
	  
	  Status open(std::string fn) 
	  {
               Status status = Failure;

               std::string ext = fn.substr(fn.find_last_of(".") + 1);
               std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

               if ( ext == "SON") {
		    sonar.setSonarFile(fn);
		    sonar.setRange(0,45);
		    sonar.init();
		    type = SonarType;
                    live_ = Recorded;
                    status = Success; //TODO: need check
               } else if ( ext == "AVI" || 
                           ext == "WMV" ||
                           ext == "MOV" || 
                           ext == "MP4" ||
                           ext == "MPG") {
		    vcap.open( fn );
		    type = MovieType;
                    live_ = Recorded;

                    if (vcap.isOpened()) {
                         status = Success;
                    } else {
                         cout << "Failed to open: " << fn  << endl;
                         status = Failure;
                    }
               } else if ( ext == "JPG") {
                    type = ImageType;
                    img_fn_ = fn;

                    cv::Mat check;
                    check = cv::imread(img_fn_, CV_LOAD_IMAGE_COLOR);

                    // imread returns cv::Mat.data == NULL if invalid image
                    if (check.data != NULL) {                         
                         valid_img_ = true;
                         status = Success;
                    } else {
                         valid_img_ = false;
                         status = Failure;
                    }
               } else {
                    cout << "File: " << fn << endl;
		    cout << "Invalid file extension: " << ext << endl;
		    status = Failure;
	       }
               return status;                  
	  }
       
          bool isLive()
          {
               if ( live_ == Live) {
                    return true;
               } else {
                    return false;
               }
          }

	  bool isOpened()
	  {
               bool status = false;
               
	       if (type == SonarType) {
		    status = true;
	       } else if (type == MovieType){
		    status = vcap.isOpened();
	       } else if (type == ImageType) {
                    status = valid_img_;
               }
               return status;
	  }

	  bool read(cv::Mat &frame) 
	  {
               bool status = false;
	       if(type == SonarType) {
		    status = sonar.getNextSonarImage(frame);
               } else if (type == ImageType) {
                    frame = cv::imread(img_fn_, CV_LOAD_IMAGE_COLOR);
		    status = true;
               } else if (type == MovieType){
                    status = vcap.read(frame_);
                    frame = frame_;		    
	       }
               return status;
	  }

          int get_frame_count()
          {
               switch (type) {
	       case MovieType:
		    //return vcap.get(cv::CAP_PROP_FRAME_COUNT);
                    return vcap.get(CV_CAP_PROP_FRAME_COUNT);
		    break;
	       case CameraType:
		    return 0;
		    break;
               case SonarType:
                    return sonar.getNumPings();
	       default:
		    return -1;
	       }
          }

          int get_fps()
          {
               switch (type) {
	       case MovieType:
		    //return vcap.get(cv::CAP_PROP_FPS);
                    return vcap.get(CV_CAP_PROP_FPS);
		    break;
	       case CameraType:
                    return 15;
		    //return vcap.get(CV_CAP_PROP_FPS);
		    break;
               case SonarType:
                    return 0;
               case ImageType:
                    return 10000; // update image every 10 seconds
	       default:
		    return -1;
	       }
          }

          int get_frame_number()
          {
               switch (type) {
	       case MovieType:
                    //return vcap.get(cv::CAP_PROP_POS_FRAMES);
                    return vcap.get(CV_CAP_PROP_POS_FRAMES);
		    break;
	       case CameraType:
		    return 0;
		    break;
               case SonarType:
                    return sonar.getCurrentPingNum();
	       default:
		    return -1;
	       }
          }

          void set_frame_number(int frame_num)
          {
               switch (type) {
	       case MovieType:
                    vcap.set(CV_CAP_PROP_POS_FRAMES, frame_num);
                    break;
	       case CameraType:
                    break;
               case SonarType:
                    sonar.setFrameNum(frame_num);
                    break;
	       default:
                    break;
               }
          }          

          void release()
          {
               switch (type) {
	       case MovieType:
                    vcap.release();
                    break;
	       case CameraType:
                    vcap.release();
                    break;
               case SonarType:
                    break;
	       default:
                    break;
               }
          }


	  int width()
	  {
	       switch (type) {
	       case MovieType:
		    return vcap.get(CV_CAP_PROP_FRAME_WIDTH);
		    break;
	       case CameraType:
		    return vcap.get(CV_CAP_PROP_FRAME_WIDTH);
		    break;
               case SonarType:
                    return sonar.width();
                    break;
	       default:
		    return -1;
	       }
	  }

	  int height()
	  {
	       switch (type) {
	       case MovieType:
		    return vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
		    break;
	       case CameraType:
		    return vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
		    break;
               case SonarType:
                    return sonar.height();
                    break;
	       default:
		    return -1;
	       }
	  }

          int get_codec()
	  {
	       switch (type) {
	       case MovieType:
		    return vcap.get(CV_CAP_PROP_FOURCC);
		    break;
	       case CameraType:
		    return CV_FOURCC('M','J','P','G');
		    break;
               case SonarType:
                    return CV_FOURCC('M','J','P','G');
                    break;
	       default:
		    return -1;
	       }
	  }

          Status export_video(std::string filename, int start_frame, int end_frame)
          {
               cv::VideoWriter output;
               
               // Get the video size
               cv::Size size = cv::Size((int) this->width(),    // Acquire input size
                                        (int) this->height());
               
               std::string ext = filename.substr(filename.find_last_of(".") + 1);
               std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

               int codec;
               if (ext == "MPG") {
                    codec = select_codec(MP4);
               } else if (ext == "AVI") {
                    codec = select_codec(AVI);
               } else if (ext == "MP4") {
                    codec = select_codec(MP4);
               } else {
                    codec = select_codec(AVI);
               }

               // Open the output video file:
               output.open(filename, codec, this->get_fps(), size, true);
               
               if (!output.isOpened()) {
                    cout << "Error: Couldn't open output file: " << filename << endl;
                    return Failure;
               }

               // Set starting frame:
               this->set_frame_number(start_frame);
               
               // Loop through video until the defined end frame;
               for(int i = start_frame; i <= end_frame ; i++) {
                    // Get the next frame
                    cv::Mat frame;
                    this->read(frame);
                    
                    // Check if the frame is empty
                    if (frame.empty()) { 
                         break;
                    }

                    // Write the frame
                    output.write(frame);
               }               
               return Success;
          }

          Status setup_export_video(std::string filename, int start_frame, int end_frame)
          {
               // Get the video size
               cv::Size size = cv::Size((int) this->width(),    // Acquire input size
                                        (int) this->height());
               
               std::string ext = filename.substr(filename.find_last_of(".") + 1);
               std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

               int codec;
               if (ext == "MPG") {
                    codec = select_codec(MP4);
               } else if (ext == "AVI") {
                    codec = select_codec(AVI);
               } else if (ext == "MP4") {
                    codec = select_codec(MP4);
               } else {
                    codec = select_codec(AVI);
               }

               // Open the output video file:
               //output_ = new VideoWriter;
               //output_->open(filename, codec, this->get_fps(), size, true);
               output_ = new cv::VideoWriter(filename, codec, this->get_fps(), size, true);
               
               if (!output_->isOpened()) {
                    cout << "Error: Couldn't create output file: " << filename << endl;
                    return Failure;
               }

               // Set starting frame:
               this->set_frame_number(start_frame);               
               output_count_ = start_frame;
               end_frame_ = end_frame;
               return Success;
          }

          int step_video_export()
          {
               if(output_count_ >= end_frame_) {
                    delete output_;
                    return 0;
               }
               
               // Get the next frame
               cv::Mat frame;
               this->read(frame);
                    
               // Check if the frame is empty
               if (frame.empty()) { 
                    delete output_;
                    return 0;
               }

               // Write the frame
               output_->write(frame);

               output_count_++;

               return 1;
          }

          int step_camera_record()
          {
               if (!frame_.empty() && output_ != NULL) {
                    output_->write(frame_);
                    return 1;
               }
               return 0;
          }

          int stop_camera_record()
          {
               if (output_ != NULL) {
                    delete output_;
                    return 0;
               } else {
                    cout << "Tried to delete output_ when it wasn't instantiated" << endl;
                    return 1;
               }               
               
          }
     };
    
}
#endif 
