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
	  cv::VideoCapture *vcap_;
          cv::VideoWriter *output_;
          int output_count_;
          int end_frame_;
	  Sonar sonar;
	  Stream_t type_;
          Live_t live_;
          cv::Mat frame_;          
          std::string img_fn_;
          bool valid_img_;

          int curr_frame_number_;
          int next_frame_number_;

     public:

          Stream()
          {
               output_ = NULL;
               vcap_ = NULL;
               curr_frame_number_ = -1;
               next_frame_number_ = 0;
          }
          
          Stream_t type() { return type_; }

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
                    //return CV_FOURCC('I', 'Y', 'U', 'V');
                    return CV_FOURCC('M','J','P','G');
               }
          }

          Status open(int num)
	  {           
               if (vcap_ != NULL) {
                    delete vcap_;
               }
               vcap_ = new cv::VideoCapture;                
               vcap_->open(num);
	       //vcap_->set(cv::CAP_PROP_FRAME_WIDTH, 480);
	       //vcap_->set(cv::CAP_PROP_FRAME_HEIGHT, 640);
               vcap_->set(CV_CAP_PROP_FRAME_WIDTH, 480);
	       vcap_->set(CV_CAP_PROP_FRAME_HEIGHT, 640);
	       type_ = CameraType;
               live_ = Live;

               if (vcap_->isOpened()) {
                    return Success;
               } else {                    
                    return Failure;
               }
	  }
	  
	  Status open(std::string fn) 
	  {
               Status status = Failure;

               //if (this->isOpened()) {
               //     this->release();
               //}
               
               std::string ext = fn.substr(fn.find_last_of(".") + 1);
               std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);
               
               if ( ext == "SON") {
                    sonar.set_mode(Sonar::sonar_file);
                    sonar.set_input_son_filename(fn);
                    sonar.set_data_mode(Sonar::image);
                    // TODO : this needs to be in a configuration file
                    sonar.set_color_map("/home/syllogismrxs/repos/sonar-processing/bvtsdk/colormaps/jet.cmap");
                    sonar.set_save_directory("/tmp/workbench_sonar_log");                    
                    sonar.init();
                    sonar.set_range(0,45);
                    
                    type_ = SonarType;
                    live_ = Recorded;
                    status = Success; //TODO: need check
               } else if ( ext == "AVI" || 
                           ext == "WMV" ||
                           ext == "MOV" || 
                           ext == "MP4" ||
                           ext == "MPG") {
                    
                    if (vcap_ != NULL) {
                         delete vcap_;
                    } 
                    vcap_ = new cv::VideoCapture; 
                    
		    vcap_->open( fn );
		    type_ = MovieType;
                    live_ = Recorded;

                    if (vcap_->isOpened()) {
                         status = Success;
                    } else {
                         cout << "Failed to open: " << fn  << endl;
                         status = Failure;
                    }
               } else if ( ext == "JPG"  || 
                           ext == "JPEG" || 
                           ext == "PNG") {
                    type_ = ImageType;
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

          void close()
          {               
               switch (type_) {
	       case MovieType:
                    break;
	       case CameraType:
                    break;
               case SonarType:
                    break;
	       default:
                    break;
               }
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
               
	       if (type_ == SonarType) {
		    status = true;
	       } else if (type_ == MovieType){
                    if (vcap_ == NULL) {
                         status = false;
                    } else {
                         status = vcap_->isOpened();
                    }
	       } else if (type_ == ImageType) {
                    status = valid_img_;
               }
               return status;
	  }

	  bool read(cv::Mat &frame) 
	  {               
               bool status = false;
	       if(type_ == SonarType) {
                    Sonar::Status_t sonar_stat;
		    sonar_stat = sonar.getNextSonarImage(frame);
                    if (sonar_stat == Sonar::Success) {
                         status = true;
                    }
               } else if (type_ == ImageType) {
                    frame = cv::imread(img_fn_, CV_LOAD_IMAGE_COLOR);
		    status = true;
               } else if (type_ == MovieType){
                    status = vcap_->read(frame_);
                    frame = frame_;		    
	       }
               
               curr_frame_number_ = next_frame_number_;               

               // Check for end of video
               //cout << curr_frame_number_ << " / " << get_frame_count() << endl;
               if (curr_frame_number_ >= get_frame_count()-1) {
                    return false;
               }

               return status;
	  }

          int get_frame_count()
          {
               switch (type_) {
	       case MovieType:                    
                    return vcap_->get(CV_CAP_PROP_FRAME_COUNT);
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
               switch (type_) {
	       case MovieType:
		    //return vcap_->get(cv::CAP_PROP_FPS);                    
                    return vcap_->get(CV_CAP_PROP_FPS);
		    break;
	       case CameraType:
                    return 15;
		    //return vcap_->get(CV_CAP_PROP_FPS);
		    break;
               case SonarType:
                    return 15;
               case ImageType:
                    return 15; // update image every 10 seconds
	       default:
		    return -1;
	       }
          }

          int frame_number()
          {
               switch (type_) {
	       case MovieType:
                    //return vcap_->get(CV_CAP_PROP_POS_FRAMES)-1;
                    return curr_frame_number_;
		    break;
	       case CameraType:
		    return 0;
		    break;
               case SonarType:
                    return sonar.getCurrentPingNum();
               case ImageType:
                    return 0;
	       default:
		    return -1;
	       }
          }

          int next_frame_number()
          {
               return next_frame_number_;
          }

          void set_frame_number(int frame_num)
          {
               switch (type_) {
	       case MovieType:                    
                    vcap_->set(CV_CAP_PROP_POS_FRAMES, frame_num);
                    next_frame_number_ = frame_num;
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

          void step_forward()
          {                  
               vcap_->set(CV_CAP_PROP_POS_FRAMES, ++next_frame_number_);
          }

          void step_backward()
          {
               // Make sure we can't get below zero
               if (next_frame_number_ <= 0) {
                    next_frame_number_ = 0;                    
               } else {
                    next_frame_number_--;                    
               }
               vcap_->set(CV_CAP_PROP_POS_FRAMES, next_frame_number_);
          }

          void release()
          {
               switch (type_) {
	       case MovieType:                    
                    vcap_->release();
                    delete vcap_;
                    vcap_ = NULL;
                    break;
	       case CameraType:  
                    vcap_->release();
                    delete vcap_;
                    vcap_ = NULL;
                    break;
               case SonarType:
                    break;
	       default:
                    break;
               }
          }


	  int width()
	  {
	       switch (type_) {
	       case MovieType:                    
		    return vcap_->get(CV_CAP_PROP_FRAME_WIDTH);
		    break;
	       case CameraType:                    
		    return vcap_->get(CV_CAP_PROP_FRAME_WIDTH);
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
	       switch (type_) {
	       case MovieType:                    
		    return vcap_->get(CV_CAP_PROP_FRAME_HEIGHT);
		    break;
	       case CameraType:                    
		    return vcap_->get(CV_CAP_PROP_FRAME_HEIGHT);
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
	       switch (type_) {
	       case MovieType:                    
		    return vcap_->get(CV_CAP_PROP_FOURCC);
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

               cout << "==========================================" << endl;
               cout << "Filename: " << filename << endl;
               cout << "Using codec: " << codec << endl;
               cout << "Size: " << this->height() << "x" << this->width() << endl;               
               cout << "FPS: " << this->get_fps() << endl;

               // Open the output video file:
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
                    cv::destroyWindow("export");
                    delete output_;
                    return 0;
               }
               
               // Get the next frame
               cv::Mat frame;
               this->read(frame);

#if 0
               cout << "Frame info: " << endl;
               cout << "type: " << frame.type() << endl;
               cout << "depth: " << frame.depth() << endl;
               cout << "channels: " << frame.channels() << endl;
#endif
                   
               cv::imshow("export", frame);
               cv::waitKey(1);
 
               // Check if the frame is empty
               if (frame.empty()) { 
                    cout << "Warning: Frame empty." << endl;
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
