#ifndef OBJECTTRACKER_H_
#define OBJECTTRACKER_H_
/// ---------------------------------------------------------------------------
/// @file ObjectTracker.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-08-07 16:49:34 syllogismrxs>
///
/// @version 1.0
/// Created: 21 Mar 2016
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
/// The ObjectTracker class ...
/// 
/// ---------------------------------------------------------------------------
#include <opencv_workbench/wb/PositionTracker.h>
#include <opencv_workbench/wb/Blob.h>
#include <opencv_workbench/wb/MatrixTracker.h>
#include <opencv_workbench/wb/Parameters.h>


class ObjectTracker {
public:
     ObjectTracker();
     void process_frame(cv::Mat &src, cv::Mat &dst,
                        std::vector<wb::Blob> &blobs, Parameters *params);
     
     int next_available_id();

     void overlay(cv::Mat &src, cv::Mat &dst, OverlayFlags_t flags);
     void overlay(std::vector<wb::Blob> &tracks, cv::Mat &src, cv::Mat &dst, 
                  OverlayFlags_t flags);
     void overlay(std::vector<wb::Blob*> &tracks, cv::Mat &src, cv::Mat &dst, 
                  OverlayFlags_t flags);

     std::vector<wb::Blob> &tracks() { return tracks_; }

     std::vector<wb::Entity> & estimated_divers() { return estimated_divers_; }

     void diver_classification(cv::Mat &src, cv::Mat &dst,
                               std::vector<wb::Blob> &meas, 
                               Parameters *params);
     
     bool is_diver(std::vector<wb::Entity> &objects, int id);     
     
protected:          
     std::vector<wb::Blob> tracks_;
     std::vector<wb::Blob> prev_tracks_;
     
     MatrixTracker covar_tracker_;

     std::vector<wb::Entity> estimated_divers_;
     std::vector<wb::Entity> prev_estimated_divers_;
     
     //PositionTracker left_tracker_;
     //PositionTracker right_tracker_;

     int left_side_sign_;
     
     int next_id_;
     
private:
};

#endif
