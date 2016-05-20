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

class Tracker {
     vector<Point2f> trackedFeatures;
     Mat             prevGray;
public:
     bool            freshStart;
     Mat_<float>     rigidTransform;

     Tracker():freshStart(true) {
          rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
     }

     void processImage(Mat& img) {
          Mat gray; cvtColor(img,gray,CV_BGR2GRAY);
          vector<Point2f> corners;
          if(trackedFeatures.size() < 200) {
               goodFeaturesToTrack(gray,corners,300,0.01,10);
               cout << "found " << corners.size() << " features\n";
               for (unsigned int i = 0; i < corners.size(); ++i) {
                    trackedFeatures.push_back(corners[i]);
               }
          }

          if(!prevGray.empty()) {
               
               int min_row = gray.rows;               
               int diff = gray.rows - prevGray.rows;
               if (diff < 0) {                    
                    min_row = gray.rows;
                    cv::copyMakeBorder(gray, gray, cvRound(-diff / 2.0), cvRound(-diff / 2.0), 0, 0, cv::BORDER_REPLICATE);
               } else if (diff > 0) {
                    min_row = prevGray.rows;
                    cv::copyMakeBorder(prevGray, prevGray, cvRound(diff / 2.0), cvRound(diff / 2.0), 0, 0, cv::BORDER_REPLICATE);
               }

               int min_col = gray.cols;
               diff = gray.cols - prevGray.cols;
               if (diff < 0) {                    
                    min_col = gray.cols;
                    cv::copyMakeBorder(gray, gray, 0, 0, cvRound(-diff / 2.0), cvRound(-diff / 2.0), cv::BORDER_REPLICATE);
               } else if (diff > 0) {
                    min_col = prevGray.cols;
                    cv::copyMakeBorder(prevGray, prevGray, 0, 0, cvRound(diff / 2.0), cvRound(diff / 2.0), cv::BORDER_REPLICATE);
               }

               cv::Mat valid_size(min_row, min_col, CV_8UC1);
               
               // Filter new and old features based on minimize size               
               {
                    std::vector<cv::Point2f>::iterator it = trackedFeatures.begin();
                    while (it != trackedFeatures.end()) {                         
                         if (!wb::point_inside<cv::Point2f>(*it,valid_size)) {
                              it = trackedFeatures.erase(it);
                         } else {
                              it++;
                         }
                    }
               }
               {
                    std::vector<cv::Point2f>::iterator it = corners.begin();
                    while (it != corners.end()) {                         
                         if (!wb::point_inside<cv::Point2f>(*it,valid_size)) {
                              it = corners.erase(it);
                         } else {
                              it++;
                         }
                    }
               }               

               cv::imshow("Prev", prevGray);
               cv::imshow("Gray", gray);
               
               std::vector<uchar> status; std::vector<float> errors;
               cv::calcOpticalFlowPyrLK(prevGray,gray,trackedFeatures,corners,status,errors,Size(20,20));

               if(countNonZero(status) < status.size() * 0.8) {
                    cout << "cataclysmic error \n";
                    rigidTransform = Mat::eye(3,3,CV_32FC1);
                    trackedFeatures.clear();
                    prevGray.release();
                    freshStart = true;
                    return;
               } else {
                    freshStart = false;
               }

               Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,corners,false);
               Mat_<float> nrt33 = Mat_<float>::eye(3,3);
               newRigidTransform.copyTo(nrt33.rowRange(0,2));
               rigidTransform *= nrt33;
               
               trackedFeatures.clear();
               for (unsigned int i = 0; i < status.size(); ++i) {
                    if(status[i]) {
                         trackedFeatures.push_back(corners[i]);
                    }
               }
          }

          for (unsigned int i = 0; i < trackedFeatures.size(); ++i) {
               circle(img,trackedFeatures[i],3,Scalar(0,0,255),CV_FILLED);
          }
          
          gray.copyTo(prevGray);
     }
};

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

     Tracker tracker;
     cv::Mat img;
     cv::Mat orig, orig_warped;
     
     for (std::vector<fs::path>::iterator it = files.begin();
          it != files.end(); it++) {
          
          img = cv::imread(it->string());
          cv::imshow("img", img);

          cv::Mat orig;
          img.copyTo(orig);
          
          tracker.processImage(orig);
          
          cv::Mat invTrans = tracker.rigidTransform.inv(DECOMP_SVD);
          cv::warpAffine(img,orig_warped,invTrans.rowRange(0,2),Size());
          
          cv::imshow("Stab", orig_warped);
          
          cv::waitKey(0);
     }          
}
