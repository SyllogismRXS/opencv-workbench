#ifndef _SONAR_H_
#define _SONAR_H_

#define ENABLE_SONAR 1

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

class Sonar {
public:
     typedef enum SonarMode{
          net = 0,
          sonar_file
     }SonarMode_t;
     
     typedef enum DataMode{
          image = 0,
          range
     }DataMode_t;

     typedef enum Status{
          Success = 0,
          Failure
     }Status_t;

     Sonar();
     ~Sonar();
     int getNumPings();
     int getCurrentPingNum();
     void setFrameNum(int num);
     Status_t getSonarImage(cv::Mat &image, int index);
     Status_t getNextSonarImage(cv::Mat &image);
     int reset();
     Status_t init();
     
     void set_mode(SonarMode_t mode);
     void set_data_mode(DataMode_t data_mode);
     void set_ip_addr(const std::string &ip_addr);
     void set_input_son_filename(const std::string &fn);
     void set_range(double min_range, double max_range);
     void set_max_range(double max_range);
     void set_min_range(double min_range);
     void set_color_map(const std::string &color_map);
     void set_save_directory(const std::string &save_directory);

     Status_t SonarLogEnable(bool enable);
     const std::string& current_sonar_file();


     int height();
     int width();

protected:
     bool initialized_;     
     std::string fn_;
     std::string ip_addr_;    
     bool logging_;

     SonarMode_t mode_;
     DataMode_t data_mode_;
     
     std::string cur_log_file_;
     std::string save_directory_;


#if ENABLE_SONAR == 1
     BVTHead head_;
     BVTSonar son_;

     BVTMagImage img_;
     BVTColorImage cimg_;
     BVTColorMapper mapper_;
     std::string color_map_;

     // Sonar file save / logger members
     BVTSonar son_logger_;
     BVTHead out_head_;

#endif

     int heads_;
     int pings_;

     int cur_ping_;

     double min_range_;
     double max_range_;
	  
     int height_;
     int width_;	 
};

#endif
