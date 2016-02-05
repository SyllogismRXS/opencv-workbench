#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include "Sonar.h"
//#include <syllo_common/Utils.h>

#define PI 3.14159265359

#if ENABLE_SONAR == 1
#include <bvt_sdk.h>
#endif

using std::cout;
using std::endl;

Sonar::Sonar()
     : initialized_(false), fn_(""), ip_addr_(""), logging_(false), 
       mode_(Sonar::net), save_directory_("./"), 
       color_map_(""), min_range_(0), max_range_(40)
{     
}

Sonar::~Sonar()
{
#if ENABLE_SONAR == 1
     // TODO: Segfaults (problem with bvt sdk?)
     //if (cimg_ != NULL) {
     //     BVTColorImage_Destroy(cimg_);
     //}    
     if (img_ != NULL) {
          BVTMagImage_Destroy(img_);
     }
     // TODO: Segfaults
     ///if (mapper_) {
     ///     BVTColorMapper_Destroy(mapper_);
     ///}    
     //if (son_ != NULL) {
     //     BVTSonar_Destroy(son_);
     //}

     // Close logging file
     this->SonarLogEnable(false);
#endif
}
     
Sonar::Status_t Sonar::init()
{
#if ENABLE_SONAR == 1
     logging_ = false;
     cur_ping_ = 0;
     next_ping_ = 0;

     son_ = BVTSonar_Create();
     if (son_ == NULL ) {
          printf("BVTSonar_Create: failed\n");
          return Sonar::Failure;
     }

     int ret;
     if (mode_ == Sonar::net) {
          /////////////////////////////////////////
          // Reading from physical sonar
          /////////////////////////////////////////
          
          // If the ip address string is set, try to manually connect 
          // to sonar first.
          bool manual_sonar_found = false;
          if (ip_addr_ != "" && ip_addr_ != "0.0.0.0") {
               ret = BVTSonar_Open(son_, "NET", ip_addr_.c_str());
               if( ret != 0 ) {
                    printf("Couldn't find sonar at defined IP address: %s", 
                           ip_addr_.c_str());                    
               } else {
                    manual_sonar_found = true;
               }
          }

          if (!manual_sonar_found) {
               //Create the discovery agent
               BVTSonarDiscoveryAgent agent = BVTSonarDiscoveryAgent_Create();
               if( agent == NULL ) {
                    printf("BVTSonarDiscoverAgent_Create: failed\n");
                    return Sonar::Failure;
               }
          
               // Kick off the discovery process
               ret = BVTSonarDiscoveryAgent_Start(agent);
          
               //Let the discovery process run for a short while (5 secs)
               //cout << "Searching for available sonars..." << endl;
               sleep(5);
          
               // See what we found
               int numSonars = 0;
               numSonars = BVTSonarDiscoveryAgent_GetSonarCount(agent);
          
               char SonarIPAddress[20];
          
               for(int i = 0; i < numSonars; i++) {
                    ret = BVTSonarDiscoveryAgent_GetSonarInfo(agent, i, &SonarIPAddress[0], 20);
                    //printf("Found Sonar: %d, IP address: %s\n", i, SonarIPAddress);
               }
          
               if(numSonars == 0) {
                    printf("No Sonars Found\n");
                    return Sonar::Failure;
               }    

               // Open the sonar
               //ret = BVTSonar_Open(son_, "NET", "192.168.1.45");
               ret = BVTSonar_Open(son_, "NET", SonarIPAddress);
               if( ret != 0 ) {
                    printf("BVTSonar_Open: ret=%d\n", ret);
                    return Sonar::Failure;
               }
          }
          
     } else {
          /////////////////////////////////////////
          // Reading from sonar file
          /////////////////////////////////////////
          
          // Open the sonar
          ret = BVTSonar_Open(son_, "FILE", fn_.c_str());
          if (ret != 0 ) {
               printf("BVTSonar_Open: ret=%d\n", ret);
               return Sonar::Failure;
          }
     }

     // Make sure we have the right number of heads_
     cur_head_ = -1;
     num_heads_ = -1;
     num_heads_ = BVTSonar_GetHeadCount(son_);
     //printf("BVTSonar_GetHeadCount: %d\n", num_heads_);

     for (int i = 0; i < num_heads_; i++) {
          //cout << "--------------------------------" << endl;
          //cout << "Head Number: " << i << endl;
          heads_[i] = NULL;
          //ret = BVTSonar_GetHead(son_, 0, &heads_[i]);
          ret = BVTSonar_GetHead(son_, i, heads_+i);
          if (ret != 0) {
               printf("BVTSonar_GetHead for head %d: ret=%s\n", i, BVTError_GetString(ret) );
               break;
          } else {
               //BVTHead_SetRange(heads_[i], 1, 40);
               // Check the ping count
               num_pings_ = -1;
               num_pings_ = BVTHead_GetPingCount(heads_[i]);
               //printf("BVTHead_GetPingCount: %d\n", num_pings_);
               if (num_pings_ > 0) {
                    cur_head_ = i;
               }
          }
     }
     
     if (cur_head_ < 0) {
          cout << "Can't find valid head" << endl;
          return Sonar::Failure;
     } else {
          //cout << "--------------------------------" << endl;
          //cout << "Using head: " << cur_head_ << endl;
     }

     // Set the stop and start ranges;
     stop_range_ = BVTHead_GetStopRange(heads_[cur_head_]);
     start_range_ = BVTHead_GetStartRange(heads_[cur_head_]);
                        
     // Check the ping count
     num_pings_ = -1;
     num_pings_ = BVTHead_GetPingCount(heads_[cur_head_]);
     //printf("BVTHead_GetPingCount: %d\n", num_pings_);
     
     // Set the range window
     this->set_range(min_range_, max_range_);
     
     // Build a color mapper
     mapper_ = BVTColorMapper_Create();
     if (mapper_ == NULL) {
          printf("BVTColorMapper_Create: failed\n");
          return Sonar::Failure;
     }

     // Load the colormap
     ret = BVTColorMapper_Load(mapper_, color_map_.c_str());
     if(ret != 0) {
          if (color_map_ == "") {
               printf("Color map not set.\n");
          }
          printf("BVTColorMapper_Load: ret=%d\n", ret);          
          return Sonar::Failure;
     }     

     initialized_ = true;

     // Need to read a single frame to get the image height and width
     cv::Mat temp;
     this->getNextSonarImage(temp);
     cur_ping_ = 0; // reset back to first frame
     next_ping_ = 0;

     return Sonar::Success;
#else
     return Sonar::Failure;
#endif
          
}

int Sonar::getNumPings()
{
     return num_pings_;
}

int Sonar::getCurrentPingNum()
{
     return cur_ping_;
}

void Sonar::setFrameNum(int num)
{
     next_ping_ = num;
}

int Sonar::reset()
{
     cur_ping_ = 0;
     next_ping_ = 0;
     return 0;
}

Sonar::Status_t Sonar::SonarLogEnable(bool enable)
{
#if ENABLE_SONAR == 1
     // Whether enable is true or false, if we enter the function here,
     // we should properly close the current file if currently logging
     if (logging_ && son_logger_) {
          BVTSonar_Destroy(son_logger_);
     }

     if (!enable) {
          // If logging is disabled, exit.
          logging_ = false;
          return Sonar::Success;
     }

     son_logger_ = BVTSonar_Create();
     if (son_logger_ == NULL) {
          printf("BVTSonar_Create: failed\n");
          return Sonar::Failure;
     }

     // Create the sonar file
     //cur_log_file_ = save_directory_ + "/" + syllo::get_time_string() + ".son";
     cur_log_file_ = "/tmp/fake.son"; // TODO: fix this
     
     int ret = BVTSonar_CreateFile(son_logger_, cur_log_file_.c_str(), 
                                   son_, "");
     
     if (ret != 0) {
          printf("BVTSonar_CreateFile: ret=%d\n", ret);
          return Sonar::Failure;
     }
     
     // Get the first head of the file output
     out_head_ = NULL ;
     ret = BVTSonar_GetHead(son_logger_, 0, &out_head_);
     if (ret != 0) {
          printf("BVTSonar_GetHead: ret=%d\n" ,ret);
          return Sonar::Failure;
     }

     logging_ = true;

#endif
     return Sonar::Success;
}

Sonar::Status_t Sonar::getNextSonarImage(cv::Mat &image)
{     
     Status_t status = Sonar::Failure;
     if (mode_ == Sonar::net) {
          status = getSonarImage(image, -1);
     } else if ( (next_ping_) < num_pings_) {
          cur_ping_ = next_ping_;
          status = getSonarImage(image, next_ping_++);
     } else {
          status = Sonar::Failure;
     }
     return status;
}

Sonar::Status_t Sonar::getSonarImage(cv::Mat &image, int index)
{
#if ENABLE_SONAR == 1

     if (!initialized_) {
          cout << "Sonar wasn't initialized." << endl;
          return Sonar::Failure;
     }

     BVTPing ping = NULL;
     int ret = BVTHead_GetPing(heads_[cur_head_], index, &ping);
	  
     if(ret != 0) {
          printf("BVTHead_GetPing: ret=%d\n", ret);
          return Sonar::Failure;
     }
	
     // Logging is enabled, write to file
     if (logging_) {
          ret = BVTHead_PutPing(out_head_, ping);
          if (ret != 0) {
               printf("BVTHead_PutPing: ret=%d\n", ret);
               return Sonar::Failure;
          }
     }

     
// Grayscale
#if 0
     //BVTMagImage img ;
     BVTPing_GetImageXY ( ping , &img_ ) ;
     height_ = BVTMagImage_GetHeight ( img_ ) ;
     width_ = BVTMagImage_GetWidth ( img_ ) ;
     
     IplImage * gray_img = cvCreateImageHeader ( cvSize ( width_ , height_ ) , IPL_DEPTH_16U , 1 ) ;
     cvSetImageData ( gray_img , BVTMagImage_GetBits ( img_ ) , width_*2 ) ;
     
     image = gray_img;
     
     cvReleaseImageHeader(&gray_img);     
     BVTMagImage_Destroy ( img_ );
     
// Color
#else
     
     ret = BVTPing_GetImage(ping, &img_);
     if (ret != 0) {
          printf("BVTPing_GetImage: ret=%d\n", ret);
          return Sonar::Failure;
     }     
     
     // Perform the colormapping
     ret = BVTColorMapper_MapImage(mapper_, img_, &cimg_);
     if (ret != 0) {
          printf("BVTColorMapper_MapImage: ret=%d\n", ret);
          return Sonar::Failure;
     }
     
     height_ = BVTColorImage_GetHeight(cimg_);
     width_ = BVTColorImage_GetWidth(cimg_);
     
     IplImage* sonarImg;
     sonarImg = cvCreateImageHeader(cvSize(width_,height_), IPL_DEPTH_8U, 4);
        
     // And set it's data
     cvSetImageData(sonarImg,  BVTColorImage_GetBits(cimg_), width_*4);
     
     image = cv::cvarrToMat(sonarImg);    
     cv::cvtColor(image, image, cv::COLOR_BGRA2BGR, 3);
     
     cvReleaseImageHeader(&sonarImg);
     
     //////////////////////////////////////////////////////
     // Generate image from range data
     BVTRangeData rangeData;
     ret=BVTPing_GetRangeData(ping, &rangeData);
     if(ret!=0) {
          printf("BVTPing_GetRangeData:ret=%d\n",ret);
          return Sonar::Failure;
     }

     int range_count = BVTRangeData_GetCount(rangeData);
       
     // Use height/width from above image
     //height_ = 600;
     //width_ = 600;
     float rotate = PI/2;
     range_image_ = cv::Mat::zeros(height_, width_, CV_8UC1);
     
     int origin_row = BVTColorImage_GetOriginRow(cimg_);
     int origin_col = BVTColorImage_GetOriginCol(cimg_);
     double range_resolution = BVTColorImage_GetRangeResolution(cimg_);
     
     cv::Point origin(origin_col, origin_row);
     
     double min_intensity = 1e9;
     double max_intensity = -1e9;
     
     for (int i = 0; i < range_count; i++) {
          double range = BVTRangeData_GetRangeValue(rangeData,i);
          if (range < 1000) {
               double intensity = BVTRangeData_GetIntensityValue(rangeData, i);
               
               if (intensity > max_intensity) {
                    max_intensity = intensity;
               } else if (intensity < min_intensity) {
                    min_intensity = intensity;
               }
                    
               double theta = BVTRangeData_GetBearingValue(rangeData,i) * PI / 180.0;
                    
               double x = range * cos(theta+rotate);
               double y = range * sin(theta+rotate);

               double x_res = x / range_resolution;
               double y_res = y / range_resolution;
               cv::Point p(origin.x - cvRound(x_res), origin.y - cvRound(y_res));
               
               //x =  x / stop_range_ * width_;
               //y =  y / stop_range_ * height_;
                    
               //cv::Point p2(origin.x + x, origin.y + y);
                                   
               //cout << "-------------" << endl;
               //cout << "origin: " << origin << endl;
               //cout << "x: " << x << endl;
               //cout << "y: " << y << endl;
               //cout << "x_res: " << x_res << endl;
               //cout << "y_yes: " << y_res << endl;
               //cout << "Intensity: " << intensity << endl;
               //cout << "Theta: " << theta << endl;
               //cout << "Range: " << range << endl;
               //cout << "Point: " << endl;
               //cout << "resolution: " << range_resolution << endl;
               //cout << "\t(" << p.x << "," << p.y << ")" << endl;
               
               //cv::circle(img, p, 4, cv::Scalar(0,0,255), -1, 8, 0);
               //cv::circle(img, p2, 4, cv::Scalar(255,0,0), -1, 8, 0);
               //cv::line(img, p, p2, cv::Scalar(255,255,255), 1, 8, 0);
                    
               //image.at<uchar>(p2) = intensity;
               range_image_.at<uchar>(p) = intensity;
          }               
     }
     
     // Normalize range elements that are greater than zero.
     // Norm from 1 to 255;
     int nRows = range_image_.rows;
     int nCols = range_image_.cols;
          
     int i,j;
     uchar* p;                         
     for( i = 0; i < nRows; ++i) {
          p = range_image_.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               if (p[j] > 0) {
                    p[j] = round((p[j] - min_intensity) / (max_intensity - min_intensity) * 255.0);                    
               }
          }
     }
          
     //////////////////////////////////////////////
     // Rotate the sonar image to pointing "up"
     //////////////////////////////////////////////
     //cv::Point center = cv::Point( image.cols/2, image.rows/2 );
     //double angle = 180.0;
     //double rot_scale = 1.0;
     //     
     //cv::Mat rot_mat(2,3,CV_8UC1);
     //rot_mat = cv::getRotationMatrix2D( center, angle, rot_scale );
     //cv::warpAffine(image, image, rot_mat, image.size());

#endif

     BVTPing_Destroy(ping);

     return Sonar::Success;
#else
     return Sonar::Failure;
#endif
}

double Sonar::pixel_range(int row, int col)
{
     return BVTColorImage_GetPixelRange(cimg_, row, col);
}

// returns bearing in degrees
double Sonar::pixel_bearing(int row, int col)
{
     return BVTColorImage_GetPixelRelativeBearing(cimg_, row, col);
}

cv::Point Sonar::get_pixel(double range, double bearing)
{
     int origin_row = BVTColorImage_GetOriginRow(cimg_);
     int origin_col = BVTColorImage_GetOriginCol(cimg_);
     double range_resolution = BVTColorImage_GetRangeResolution(cimg_);          
     
     double x,y;
     // These are flipped on-purpose
     x = range*sin(bearing)/range_resolution;
     y = range*cos(bearing)/range_resolution;     

     cv::Point result;     
     result.x = origin_col + cvRound(y); 
     result.y = origin_row - cvRound(x);          
          
     return result;
}


int Sonar::width() 
{ 
     return width_;
}

int Sonar::height()
{
     return height_;
}

void Sonar::set_mode(SonarMode_t mode)
{
     mode_ = mode;
}

void Sonar::set_ip_addr(const std::string &ip_addr)
{
     ip_addr_ = ip_addr;
}

void Sonar::set_input_son_filename(const std::string &fn)
{
     fn_ = fn;
}

void Sonar::set_range(double min_range, double max_range)
{
     min_range_ = min_range;
     max_range_ = max_range;

     if (min_range_ < 0 || min_range_ > max_range_ ) {
          min_range = 0;
     }

     if (max_range_ < 0 || max_range_ <= min_range_+1) {
          max_range_ = min_range_ + 2;
     }
     
#if ENABLE_SONAR == 1
     BVTHead_SetRange(heads_[cur_head_], min_range_, max_range_);
#endif
}

void Sonar::set_min_range(double min_range)
{
     this->set_range(min_range, max_range_);     
}

void Sonar::set_max_range(double max_range)
{
     this->set_range(min_range_, max_range);     
}

void Sonar::set_color_map(const std::string &color_map)
{
     color_map_ = color_map;
}

void Sonar::set_save_directory(const std::string &save_directory)
{
     save_directory_ = save_directory;
}

const std::string& Sonar::current_sonar_file()
{
     return cur_log_file_;
}
