#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

#include <vector>
#include <algorithm> 

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>
//
#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/track/Dynamics.h>
#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/track/EKF.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


using std::cout;
using std::endl;

#include <boost/filesystem.hpp>
namespace fs = ::boost::filesystem;


using namespace cv;

bool path_compare (fs::path i, fs::path j) 
{ 
     return (i.string() < j.string());     
}

void make_same_size(std::vector<cv::Mat> &original_imgs, 
                    std::vector<cv::Mat> &adjusted_imgs, int depth)
{
     // Look back 'depth' frames and make sure all images are the same size as
     // the current img.
     adjusted_imgs.clear();

     // First compute the average frame size
     int count = 0;
     int rows_sum = 0;
     int cols_sum = 0;
     for (std::vector<cv::Mat>::reverse_iterator rit= original_imgs.rbegin(); 
          rit != original_imgs.rend() && count < depth; ++rit, ++count) {
          rows_sum += rit->rows;
          cols_sum += rit->cols;          
     }

     int rows_avg = cvRound((double)rows_sum / (double) count);
     int cols_avg = cvRound((double)cols_sum / (double) count);     
     
     //cout << "-----------" << endl;
     //cout << "Rows Avg: " << rows_avg << endl;
     //cout << "Cols Avg: " << cols_avg << endl;
     
     // Adjust size of images
     // First compute the average frame size
     count = 0;
     for (std::vector<cv::Mat>::reverse_iterator rit= original_imgs.rbegin(); 
          rit != original_imgs.rend() && count < depth; ++rit, ++count) {
          
          cv::Mat adjusted_img = rit->clone();
          
          //cout << "Count: " << count << endl;
          //cout << "Frame rows: " << rit->rows << endl;
          //cout << "Frame cols: " << rit->cols << endl;

          int diff = rows_avg - rit->rows;
          if (diff < 0) {                  
               // Frame rows is larger than average, crop rows
               cv::Rect roi(0, cvRound(abs(diff) / 2.0), adjusted_img.cols, adjusted_img.rows - cvRound(abs(diff) / 2.0));
               adjusted_img = cv::Mat(adjusted_img, roi);
          } else if (diff > 0) {               
               // Frame rows is smaller than average, make border
               cv::copyMakeBorder(adjusted_img, adjusted_img, cvRound(diff / 2.0), cvRound(diff / 2.0), 0, 0, cv::BORDER_REPLICATE);
          }
          
          diff = cols_avg - rit->cols;
          if (diff < 0) {                                      
               // Frame cols is larger than average, crop cols               
               cv::Rect roi(cvRound(abs(diff)/2.0), 0, adjusted_img.cols - cvRound(abs(diff)/2.0), adjusted_img.rows);
               adjusted_img = cv::Mat(adjusted_img, roi);
          } else if (diff > 0) {               
               // Frame cols is smaller than average, make border
               cv::copyMakeBorder(adjusted_img, adjusted_img, 0, 0, cvRound(diff / 2.0), cvRound(diff / 2.0), cv::BORDER_REPLICATE);
          }                   
          
          //cv::imshow("Adjusted", adjusted_img);
          //cv::waitKey(0);
          adjusted_imgs.push_back(adjusted_img);
     }     
}

int similarity(cv::Mat &img1, cv::Mat &img2)
{
     int sum = 0;
     
     int channels = img1.channels();
     int nRows = img1.rows;
     int nCols = img1.cols * channels;
     if (img1.isContinuous()) {
     	  nCols *= nRows;
     	  nRows = 1; 
     }
     int i,j;
     uchar *p1, *p2;
     for( i = 0; i < nRows; ++i) {
     	  p1 = img1.ptr<uchar>(i);
     	  p2 = img2.ptr<uchar>(i);
     	  for ( j = 0; j < nCols; ++j) {
     	       sum += abs( p1[j] - p2[j] );
     	  }
     }
     return sum;
}

void recurrence_plot(std::vector<cv::Mat> &imgs, cv::Mat &rp_dst)
{
     rp_dst = cv::Mat::zeros(imgs.size(), imgs.size(), CV_8UC1);
     uchar *p;
     for (unsigned int i = 0 ; i < imgs.size() ; i++) {
     	  p = rp_dst.ptr<uchar>(i);
     	  for (unsigned int j = 0 ; j < imgs.size() ; j++) {
     	       int sum = similarity(imgs[i], imgs[j]);
     	       p[j] = sum;
     	  }
     }     
}


int main(int argc, char *argv[])
{
     if (argc < 2) {
          cout << "usage: " << argv[0] << " <input-dir>" << endl;
          return -1;
     }

     std::vector<fs::path> files;
     syllo::get_files_with_ext(fs::path(std::string(argv[1])), ".png",
                               files, false);

     std::sort(files.begin(), files.end(), path_compare);      
     
     std::vector<cv::Mat> imgs;     
     
     for (std::vector<fs::path>::iterator it1 = files.begin();
          it1 != files.end(); it1++) {                   
          
          cv::Mat img = cv::imread(it1->string());
          imgs.push_back(img);
          cv::imshow("Current Image", img);

          std::vector<cv::Mat> adjusted_imgs;
          make_same_size(imgs, adjusted_imgs, 20);

          int i = 0;
          for (std::vector<cv::Mat>::iterator it2 = adjusted_imgs.begin();
               it2 != adjusted_imgs.end(); it2++, i++) {
               //std::string win_name = std::string("Adjusted: ") + syllo::int2str(i);
               //cv::imshow(win_name, *it2);               
               cv::imshow("Adjusted", *it2);
               cv::waitKey(25);                                            
          }

          //cv::Mat rp;
          //recurrence_plot(adjusted_imgs, rp);         
          //cv::imshow("Recurrence Plot",rp);
          //
          //cv::Mat rp_norm = rp.clone();
          //cv::normalize(rp_norm, rp_norm, 0, 255, CV_MINMAX, CV_8UC1);
          //cv::imshow("Normalized", rp_norm);
          //
          //cv::Mat rp_norm_thresh = rp_norm.clone();
          //cv::threshold( rp_norm_thresh, rp_norm_thresh, 230, 255, cv::THRESH_TOZERO);
          //cv::imshow("Norm,Thresh", rp_norm_thresh);
                    
          int key = cv::waitKey(0);          
          if (key == 'q' || key == 1048689) {
               break;
          }
     }          
}


// class Tracker {
//      vector<Point2f> trackedFeatures;
//      Mat             prevGray;
// public:
//      bool            freshStart;
//      Mat_<float>     rigidTransform;
// 
//      Tracker():freshStart(true) {
//           rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
//      }
// 
//      void processImage(Mat& img) {
//           Mat gray; cvtColor(img,gray,CV_BGR2GRAY);
//           vector<Point2f> corners;
//           if(trackedFeatures.size() < 200) {
//                goodFeaturesToTrack(gray,corners,300,0.01,10);
//                cout << "found " << corners.size() << " features\n";
//                for (unsigned int i = 0; i < corners.size(); ++i) {
//                     trackedFeatures.push_back(corners[i]);
//                }
//           }
// 
//           if(!prevGray.empty()) {
//                
//                int min_row = gray.rows;               
//                int diff = gray.rows - prevGray.rows;
//                if (diff < 0) {                    
//                     min_row = gray.rows;
//                     cv::copyMakeBorder(gray, gray, cvRound(-diff / 2.0), cvRound(-diff / 2.0), 0, 0, cv::BORDER_REPLICATE);
//                } else if (diff > 0) {
//                     min_row = prevGray.rows;
//                     cv::copyMakeBorder(prevGray, prevGray, cvRound(diff / 2.0), cvRound(diff / 2.0), 0, 0, cv::BORDER_REPLICATE);
//                }
// 
//                int min_col = gray.cols;
//                diff = gray.cols - prevGray.cols;
//                if (diff < 0) {                    
//                     min_col = gray.cols;
//                     cv::copyMakeBorder(gray, gray, 0, 0, cvRound(-diff / 2.0), cvRound(-diff / 2.0), cv::BORDER_REPLICATE);
//                } else if (diff > 0) {
//                     min_col = prevGray.cols;
//                     cv::copyMakeBorder(prevGray, prevGray, 0, 0, cvRound(diff / 2.0), cvRound(diff / 2.0), cv::BORDER_REPLICATE);
//                }
// 
//                cv::Mat valid_size(min_row, min_col, CV_8UC1);
//                
//                // Filter new and old features based on minimize size               
//                {
//                     std::vector<cv::Point2f>::iterator it = trackedFeatures.begin();
//                     while (it != trackedFeatures.end()) {                         
//                          if (!wb::point_inside<cv::Point2f>(*it,valid_size)) {
//                               it = trackedFeatures.erase(it);
//                          } else {
//                               it++;
//                          }
//                     }
//                }
//                {
//                     std::vector<cv::Point2f>::iterator it = corners.begin();
//                     while (it != corners.end()) {                         
//                          if (!wb::point_inside<cv::Point2f>(*it,valid_size)) {
//                               it = corners.erase(it);
//                          } else {
//                               it++;
//                          }
//                     }
//                }               
// 
//                cv::imshow("Prev", prevGray);
//                cv::imshow("Gray", gray);
//                
//                std::vector<uchar> status; std::vector<float> errors;
//                cv::calcOpticalFlowPyrLK(prevGray,gray,trackedFeatures,corners,status,errors,Size(20,20));
// 
//                if(countNonZero(status) < status.size() * 0.8) {
//                     cout << "cataclysmic error \n";
//                     rigidTransform = Mat::eye(3,3,CV_32FC1);
//                     trackedFeatures.clear();
//                     prevGray.release();
//                     freshStart = true;
//                     return;
//                } else {
//                     freshStart = false;
//                }
// 
//                Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,corners,false);
//                Mat_<float> nrt33 = Mat_<float>::eye(3,3);
//                newRigidTransform.copyTo(nrt33.rowRange(0,2));
//                rigidTransform *= nrt33;
//                
//                trackedFeatures.clear();
//                for (unsigned int i = 0; i < status.size(); ++i) {
//                     if(status[i]) {
//                          trackedFeatures.push_back(corners[i]);
//                     }
//                }
//           }
// 
//           for (unsigned int i = 0; i < trackedFeatures.size(); ++i) {
//                circle(img,trackedFeatures[i],3,Scalar(0,0,255),CV_FILLED);
//           }
//           
//           gray.copyTo(prevGray);
//      }
// };
