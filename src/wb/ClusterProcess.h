#ifndef CLUSTERPROCESS_H_
#define CLUSTERPROCESS_H_
/// ---------------------------------------------------------------------------
/// @file ClusterProcess.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-09-08 14:16:42 syllogismrxs>
///
/// @version 1.0
/// Created: 04 Sep 2015
///
/// ---------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ---------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The ClusterProcess class ...
/// 
/// ---------------------------------------------------------------------------
#include <list>

#include <opencv_workbench/wb/Cluster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

class ClusterProcess {
public:
     ClusterProcess();

     void set_threshold(int threshold) { threshold_ = threshold; }
     void set_gate(float gate) { gate_ = gate; }
     void set_min_cluster_size(int min_cluster_size) 
     { min_cluster_size_ = min_cluster_size; }
     
     void process_frame(cv::Mat &src);
     void overlay_clusters(cv::Mat &src, cv::Mat &dst);
     void overlay_tracks(cv::Mat &src, cv::Mat &dst);
     
     int next_available_id();
     
     void cluster_maintenance();         
     
protected:
private:
     std::vector<wb::Point> points_;
     std::list<wb::Cluster*> clusters_;
     
     std::vector<wb::Point> prev_points_;
     std::list<wb::Cluster*> prev_clusters_;
     
     int threshold_;
     float gate_;
     int min_cluster_size_;

     int next_id_;
};

#endif
