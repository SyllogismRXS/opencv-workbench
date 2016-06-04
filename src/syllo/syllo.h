#ifndef SYLLO_H_
#define SYLLO_H_

#include <iostream>
#include <sstream>

#include <cv.h>
#include <highgui.h>

#include <Eigen/Dense>

//#include "Sonar.h"
//#include "filters.h"
#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/track/ParticleFilter.h>
#include <opencv_workbench/track/MotionModels.h>

#include <opencv_workbench/syllo/Blob.h>
#include <opencv_workbench/syllo/Cluster.h>

#include <boost/filesystem.hpp>
namespace fs = ::boost::filesystem;

namespace syllo
{
     int str2int(std::string str);
     double str2double(std::string str);
          
     std::string int2str(int x);
     std::string double2str(double x);

     std::vector<cv::Point3d> point2d_to_point3d_vectors(std::vector<cv::Point2d> &p2);     
     cv::Point3d point2d_to_point3d(cv::Point2d &p2);     

     // return the filenames of all files that have the specified extension
     // in the specified directory and all subdirectories
     void get_files_with_ext(const fs::path& root, const std::string& ext, 
                             std::vector<fs::path>& ret, bool recursive);

     bool copy_file_with_value(std::string &ranges_dir, 
                               std::string &output_file,
                               std::string &param_sweep, 
                               double search_value);
     
     void fill_line(std::string ch);
     
     cv::Point getTypeCentroid(EntityType type, std::map<int,syllo::Cluster> &clusters);

     int flipClusterEntity(cv::Point point, std::map<int,syllo::Cluster> &clusters);
     int tagClusterEntity(cv::Point point, EntityType target, EntityType others, 
                          std::map<int,syllo::Cluster> &clusters);

     int ClusterTemplateMatch(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Cluster> &clusters, cv::Mat &templateImg);

     int formClusters(std::map<int,syllo::Blob> blobs, std::map<int,syllo::Cluster> &clusters);
     int ClusterMatch(std::map<int,syllo::Cluster> &prevClusters, std::map<int,syllo::Cluster> &clusters, std::map<int,syllo::Cluster> &newClusters, std::map<int,syllo::Cluster> &allClusters, int curTime, std::map<int,bool> &IDs);
     int drawClusters(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Cluster> &clusters, int radius);

     int StepClusterPF(std::map<int,syllo::Cluster> &clusters);
     int drawClusterParticles(std::map<int,syllo::Cluster> &clusters, cv::Mat &input, cv::Mat &output);

     int getFastestCluster(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster);
     int getConstVelCluster(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster);
     int getFarthestTravel(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster);

     int templateDetect(const cv::Mat &inputImg, const cv::Mat &templImg, cv::Mat &outputImg);

     int DetectBlobs(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Blob> &blobs);

     int BlobTemplateMatch(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Blob> &blobs, cv::Mat &templateImage);

     int RunningGaussian(double alpha, cv::Mat &img1, cv::Mat &img2, cv::Mat &avgs, cv::Mat &varImg);

     cv::Point calcCentroid(cv::Mat &img, uchar value);
     int CentroidsOfBlobs(cv::Mat &img, std::map<int,syllo::Blob> &blobs);

     int drawCircleList(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Blob> &blobs, int radius, cv::Scalar color); 

     int drawBlobTargets(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Blob> &blobs, int radius);

     cv::Point GetLargestBlob(std::map<int,syllo::Blob> &blobs);

     int BlobMatch(std::map<int,syllo::Blob> &prevBlobs, std::map<int,syllo::Blob> &blobs, std::map<int,syllo::Blob> &newBlobs, int curTime);


     int drawParticles(ParticleFilter &pf, cv::Mat &src, cv::Mat &dst);

     // Tests...
     //int var = 5;
     void helloWorld();
}

#endif 
