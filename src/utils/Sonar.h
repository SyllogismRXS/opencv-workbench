#ifndef _SONAR_H_
#define _SONAR_H_

//#define ENABLE_SONAR 0

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Set in top-level CMakeLists.txt
#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

#define MAX_SONAR_HEADS 5

class Sonar {
public:
     typedef enum SonarMode{
          net = 0,
          sonar_file
     }SonarMode_t;          

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
     void set_ip_addr(const std::string &ip_addr);
     void set_input_son_filename(const std::string &fn);
     void set_range(double min_range, double max_range);
     void set_max_range(double max_range);
     void set_min_range(double min_range);
     void set_color_map(const std::string &color_map);
     void set_save_directory(const std::string &save_directory);
     
     double pixel_range(int row, int col);
     double pixel_bearing(int row, int col);

     //double get_range_resolution();
     //double get_bearing_resolution();
     //cv::Point get_origin();
     cv::Point get_pixel(double range, double bearing);

     Status_t SonarLogEnable(bool enable);
     const std::string& current_sonar_file();


     int height();
     int width();

     void range_image(cv::Mat &img) { img = range_image_; }

protected:

     cv::Mat range_image_;
     
     bool initialized_;     
     std::string fn_;
     std::string ip_addr_;    
     bool logging_;

     SonarMode_t mode_;
          
     float stop_range_;
     float start_range_;
     
     std::string cur_log_file_;
     std::string save_directory_;

     std::string color_map_;
     
#if ENABLE_SONAR == 1
     BVTHead heads_[MAX_SONAR_HEADS];
     BVTSonar son_;

     BVTMagImage img_;
     BVTColorImage cimg_;
     BVTColorMapper mapper_;     

     // Sonar file save / logger members
     BVTSonar son_logger_;
     BVTHead out_head_;
#endif

     int num_heads_;
     int num_pings_;

     int cur_head_;
     int cur_ping_;
     int next_ping_;

     double min_range_;
     double max_range_;
	  
     int height_;
     int width_;	 
};

#endif
