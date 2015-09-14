#include <iostream>

#include <string.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include <bvt_sdk.h>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//char DataFile[]="../../data/RangeData.son";

#define MAX_SONAR_HEADS 5

using std::cout;
using std::endl;

int main(int argc, char*argv[])
{
     int ret;
     //Create a new BVTSonarObject
     BVTSonar son=BVTSonar_Create();
     if(son==NULL)
     {
          printf("BVTSonar_Create:failed\n");
          return 1;
     }

     //Open the sonar
     char DataFile[512];
     if(argc==2) {
          strcpy(DataFile,argv[1]);
     }

     ret=BVTSonar_Open(son,"FILE",DataFile);
     if(ret!=0)
     {
          printf("BVTSonar_Open:ret=%d\n",ret);
          return 1;
     }
     //////
     
     BVTHead heads_[MAX_SONAR_HEADS];

     // Make sure we have the right number of heads_
     int cur_head_ = -1;
     int num_heads_ = -1;
     int num_pings_ = -1;
     num_heads_ = BVTSonar_GetHeadCount(son);
     printf("BVTSonar_GetHeadCount: %d\n", num_heads_);     
     
     for (int i = 0; i < num_heads_; i++) {
          cout << "--------------------------------" << endl;
          cout << "Head Number: " << i << endl;
          
          heads_[i] = NULL;
          ret = BVTSonar_GetHead(son, i, heads_+i);
          if (ret != 0) {
               printf("BVTSonar_GetHead for head %d: ret=%s\n", i, BVTError_GetString(ret) );
               break;
          } else {
               num_pings_ = -1;
               num_pings_ = BVTHead_GetPingCount(heads_[i]);
               printf("BVTHead_GetPingCount: %d\n", num_pings_);
               if (num_pings_ > 0) {
                    cur_head_ = i;
               }
          }
     }
     
     if (cur_head_ < 0) {
          cout << "Can't find valid head" << endl;
          return -1;
     } else {
          cout << "--------------------------------" << endl;
          cout << "Using head: " << cur_head_ << endl;
     }

     // Get stop range
     float stop_range_ = BVTHead_GetStopRange(heads_[cur_head_]);
     
     for (int k = 0; k < num_pings_; k++) {
          //cout << "======================================" << endl;
          //cout << "Ping Number: " << k << endl;
          //cout << "======================================" << endl;
     
          //Now,getaping!
          BVTPing ping=NULL;
          ret=BVTHead_GetPing(heads_[cur_head_],k,&ping);
          if(ret!=0) {
               printf("BVTHead_GetPing:ret=%d\n",ret);
               return 1;
          }
      
          //Gettherangedatafromthisping
          BVTRangeData rangeData;
          ret=BVTPing_GetRangeData(ping, &rangeData);
          if(ret!=0) {
               printf("BVTPing_GetRangeData:ret=%d\n",ret);
               return 1;
          }
      
          //Howmanyrangevaluesarethere?#ofrangevalues=#ofbeams
          int numberOfRanges=BVTRangeData_GetCount(rangeData);
          //printf("BVTRangeData_GetCount:%d\n",numberOfRanges);
      
          int img_width = 600;
          int img_height = 600;
          float rotate = 3.14159265359/2;
          cv::Mat img = cv::Mat::zeros(img_height, img_width, CV_8UC1);
          cv::Point origin(img_width/2,0);
          
          //double max_angle_ = 0.392699082;
          //double y_max = sin(max_angle_)*x_max*2;
          
          double min_intensity = 1e9;
          double max_intensity = -1e9;
          
          for (int i = 0; i < numberOfRanges; i++) {
               //cout << "------------------------" << endl;
               //cout << "Bearing: " << BVTRangeData_GetBearingValue(rangeData,i) << endl;
               //cout << "Range: " << BVTRangeData_GetRangeValue(rangeData,i) << endl;
               //cout << "Intensity: " << BVTRangeData_GetIntensityValue(rangeData,i) << endl;               
               double range = BVTRangeData_GetRangeValue(rangeData,i);               
               if (range < 1000) {
                    double intensity = BVTRangeData_GetIntensityValue(rangeData, i);

                    if (intensity > max_intensity) {
                         max_intensity = intensity;
                    } else if (intensity < min_intensity) {
                         min_intensity = intensity;
                    }
                    
                    double theta = BVTRangeData_GetBearingValue(rangeData,i) * 3.14159265359 / 180.0;
                    
                    double x = range * cos(theta+rotate);
                    double y = range * sin(theta+rotate);
                    cv::Point p(origin.x + x, origin.y + y);
                    
                    x =  x / stop_range_ * img_height;
                    y =  y / stop_range_ * img_height;
                    
                    cv::Point p2(origin.x + x, origin.y + y);
                    
                    //cout << "-------------" << endl;
                    //cout << "Intensity: " << intensity << endl;
                    //cout << "Theta: " << theta << endl;
                    //cout << "Range: " << range << endl;
                    //cout << "Point before: " << endl;
                    //cout << "\t(" << p.x << "," << p.y << ")" << endl;
                    //cout << "Point transform: " << endl;
                    //cout << "\t(" << p2.x << "," << p2.y << ")" << endl;

                    //cv::circle(img, p, 4, cv::Scalar(0,0,255), -1, 8, 0);
                    //cv::circle(img, p2, 4, cv::Scalar(255,0,0), -1, 8, 0);
                    //cv::line(img, p, p2, cv::Scalar(255,255,255), 1, 8, 0);
                    
                    img.at<uchar>(p2) = intensity;
               }               
          }
          
          {
               // Normalize range elements that are greater than zero.
               // Norm from 1 to 255;
               int nRows = img.rows;
               int nCols = img.cols;

               int i,j;
               uchar* p;                         
               for( i = 0; i < nRows; ++i) {
                    p = img.ptr<uchar>(i);
                    for ( j = 0; j < nCols; ++j) {
                         if (p[j] > 0) {
                              p[j] = round((p[j] - min_intensity) / (max_intensity - min_intensity) * 255.0);
                              //printf("p[j]: %d\n", p[j]);
                         }
                    }
               }
          }
          
          //cv::normalize(img, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);

          // Rotate the sonar image to pointing "up"
          cv::Point center = cv::Point( img.cols/2, img.rows/2 );
          double angle = 180.0;
          double rot_scale = 1.0;
     
          cv::Mat rot_mat(2,3,CV_8UC1);
          rot_mat = cv::getRotationMatrix2D( center, angle, rot_scale );
          cv::warpAffine(img, img, rot_mat, img.size());
     
          cv::imshow("ranges", img);
          int key = cv::waitKey(1);
          if (key == 'q') { 
               break;
          }
     }
     
     //std::vector<float>m_ranges;
     //std::vector<float>m_angles;
     //std::vector<float>m_intensities;
      
     ////getthebeamangles
     //m_angles.reserve(numberOfRanges);
     //for(int j=0;j<numberOfRanges;j++)
     //     m_angles.push_back(BVTRangeData_GetBearingValue(rangeData,j));
     // 
     //m_ranges.reserve(numberOfRanges);
     //for(int j=0;j<numberOfRanges;j++)
     //     m_ranges.push_back(BVTRangeData_GetRangeValue(rangeData,j));
     // 
     //m_intensities.reserve(numberOfRanges);
     //
     //for(int j=0;j<numberOfRanges;j++)
     //     printf("\tAngle\tRange\tIntensity\n");
     // 
     //for(int i=0;i<numberOfRanges;++i)
     //{
     //     printf("\t%04f\t%04f\t%04f\n",
     //            m_angles.at(i),m_ranges.at(i),m_intensities.at(i));
     //}
      
     printf("\n");

     return 0;
}

