#ifndef _SONAR_H_
#define _SONAR_H_

#define ENABLE_SONAR 0

#include <cv.h>

#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

namespace sonar {

     //typedef enum {
     //	  real = 0,
     //	  simulated = 0
     //}SonarMode;


     class Sonar {
     protected:
	  //SonarMode mMode;
	  std::string fn;
	  int mMode;

#if ENABLE_SONAR == 1
	  BVTHead head;
	  BVTSonar son;

          BVTMagImage img;
	  BVTColorImage cimg;
          BVTColorMapper mapper;
#endif

	  int heads;
	  int pings;

	  int mCurPing;

	  int mMinRange;
	  int mMaxRange;
	  
	  int height_;
	  int width_;
	  
     public:
	  Sonar();
	  Sonar(int mode, std::string sonarFile, int minRange, int maxRange);
	  ~Sonar();
	  int getNumPings();
          int getCurrentPingNum();
          void setFrameNum(int num);
	  int getSonarImage(cv::Mat &image, int index);
	  int getNextSonarImage(cv::Mat &image);
	  int reset();
	  int init();
	  void setRange(int minRange, int maxRange);
	  void setSonarFile(std::string fn);

          int height();
          int width();
     };

}

#endif
