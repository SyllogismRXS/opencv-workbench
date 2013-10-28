#ifndef STREAM_H_
#define STREAM_H_

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
	  CameraType = 1,
	  MovieType = 2,
	  SonarType = 3
     }Stream_t;

     class Stream {
     protected:
	  cv::VideoCapture vcap;
	  sonar::Sonar sonar;
	  Stream_t type;
     public:
          Status open(int num)
	  {
	       vcap.open(num);
	       vcap.set(CV_CAP_PROP_FRAME_WIDTH, 480);
	       vcap.set(CV_CAP_PROP_FRAME_HEIGHT, 640);
	       type = CameraType;

               if (vcap.isOpened()) {
                    return Success;
               } else {
                    return Failure;
               }
	  }
	  
	  Status open(std::string fn) 
	  {
               std::string ext = fn.substr(fn.find_last_of(".") + 1);
               std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

               if ( ext == "SON") {
		    sonar.setSonarFile(fn);
		    sonar.setRange(0,45);
		    sonar.init();
		    type = SonarType;
                    return Success;
	       } else if ( ext == "AVI" || 
                           ext == "WMV" ||
                           ext == "MOV" || 
                           ext == "MP4") {
		    vcap.open( fn );
		    type = MovieType;
                    return Success;
	       } else {
                    cout << "File: " << fn << endl;
		    cout << "Invalid file extension: " << ext << endl;
		    return Failure;
	       }   
	  }
       
	  bool isOpened()
	  {
	       if (type == SonarType) {
		    return true;
	       } else {
		    return vcap.isOpened();
	       }
	  }

	  bool read(cv::Mat &frame) 
	  {
	       if(type == SonarType) {
		    return sonar.getNextSonarImage(frame);
	       } else {
		    return vcap.read(frame);
	       }
	  }

          int get_frame_count()
          {
               switch (type) {
	       case MovieType:
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
		    return vcap.get(CV_CAP_PROP_FPS);
		    break;
	       case CameraType:
		    return 0;
		    break;
               case SonarType:
                    return 0;
	       default:
		    return -1;
	       }
          }

          int get_frame_number()
          {
               switch (type) {
	       case MovieType:
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
                    sonar.width();
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
     };
    
}
#endif 
