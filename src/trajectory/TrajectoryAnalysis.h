#ifndef TRAJECTORYANALYSIS_H_
#define TRAJECTORYANALYSIS_H_
/// ---------------------------------------------------------------------------
/// @file TrajectoryAnalysis.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-01-08 13:17:13 syllogismrxs>
///
/// @version 1.0
/// Created: 08 Dec 2015
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
/// The TrajectoryAnalysis class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <list>

#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/wb/Blob.h>
#include <opencv_workbench/wb/Polar.h>
#include <opencv_workbench/track/KalmanFilter.h>


class TrajectoryAnalysis {
public:

     struct TrajRMSE {
          int ID1;
          int ID2;
          double RMSE;
     };

     
     class Avgs {
     public:
     Avgs() : computed(false), count(0) {}
          
          bool computed;
          int ID1;
          int ID2;
          double diff_sum;          
          int count;
     };
     
     TrajectoryAnalysis();

     double trajectory_diff(std::list<cv::Point2d> &t1,
                            std::list<cv::Point2d> &t2);

     void trajectory_polar_diff(std::list<wb::Blob> &traj,
                                std::list<cv::Point2d> &diffs);

     void trajectory_similarity(std::map<int, std::list<wb::Blob> > 
                                &tracks_history,int frame_number, 
                                cv::Mat &img, double RMSE_threshold);

     void set_simulated(bool simulated) { simulated_ = simulated; }

     void trajectory_similarity_track_diff(std::map<int, wb::Blob> &tracks_frame,
                                           int frame_number, 
                                           cv::Mat &img, double threshold);

     
     
protected:

     std::map<std::string, Avgs> traj_avgs_;
     
     bool simulated_;
private:
};

#endif
