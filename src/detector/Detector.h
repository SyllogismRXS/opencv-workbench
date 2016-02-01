#ifndef DETECTOR_H_
#define DETECTOR_H_
/// ---------------------------------------------------------------------------
/// @file Detector.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-02-01 00:30:58 syllogismrxs>
///
/// @version 1.0
/// Created: 04 Feb 2015
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
/// The Detector class ...
///
/// ---------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <opencv_workbench/plugin_manager/PluginManager.h>

// OpenCV headers
#include <cv.h>
#include <highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/wb/Entity.h>
#include <opencv_workbench/wb/Parameters.h>
#include <opencv_workbench/wb/BlobProcess.h>
#include <opencv_workbench/utils/Stream.h>

//struct Params {
//     int history_length;
//     double history_distance;
//};
//
//void operator >> (const YAML::Node& node, Params& params)
//{
//     if(const YAML::Node *p = node.FindValue("history_length")) {
//          *p >> params.history_length;
//     }
//     if(const YAML::Node *p = node.FindValue("history_distance")) {
//          *p >> params.history_distance;
//     }
//}

class Detector {
public:

     typedef enum Status{
          Success = 0,
          Failure = -1
     }Status_t;

     typedef enum StageType{
          color = 0,
          thresh,
          cluster,
          data_association,
          detection
     }StageType_t;


     Detector() : last_stage_(detection)
     {
     }
     
     virtual void print()=0;
     virtual ~Detector()
     {
          std::cout << "Detector destructor" << std::endl;
     }

     // accept cv::Mat and return tracks?
     virtual int set_frame(int frame_number, const cv::Mat &original)=0;
     virtual void set_stream(syllo::Stream *stream) { stream_ = stream; }

     std::vector<wb::Entity> & tracks() { return tracks_; }
     void hide_windows(bool hide) { hide_windows_ = hide; }
          
     void set_params(Parameters *params) { params_ = params; }
     
     //virtual Status_t set_own_pose(const AVIA::Pose &pose)=0;
     //virtual Status_t set_command(const Command &command) {return Success;}
     //virtual Status_t set_contacts(const sensor_track_list &sensorTracks) {return Success;}
     //virtual void set_own_id(unsigned int own_id) {own_id_ = own_id;}
     //
     //virtual bool autonomy_ready() {return false;}
     //
     //virtual vec2f desired_trajectory()=0;
     cv::Mat thresh_img() { return thresh_img_; }

     void frame_ents(std::vector<wb::Entity> &frame_ents) 
     {
          frame_ents = frame_ents_;
     } 

     virtual void color_conversion(const cv::Mat &src, cv::Mat &dst) {}
     virtual void thresholding(cv::Mat &src, cv::Mat &dst) {}

     void set_last_stage(StageType_t last_stage) { last_stage_ = last_stage; }
     StageType_t last_stage() { return last_stage_; }
     
protected:
     //unsigned int own_id_;
     std::vector<wb::Entity> tracks_;
     bool hide_windows_;

     cv::Mat thresh_img_;     
     std::vector<wb::Entity> frame_ents_;

     syllo::Stream *stream_;
     
     Parameters *params_;

     StageType_t last_stage_;

     cv::Mat mask_;
     
private:
};

// typedef to make it easier to set up our factory
typedef Detector *Detector_maker_t();

// our global factory
//extern std::map<std::string, Detector_maker_t *, std::less<std::string> > factory;
extern PluginManager<Detector, Detector_maker_t> plugin_manager_;

#endif
