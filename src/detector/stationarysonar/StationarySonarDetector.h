#ifndef STATIONARYSONARDETECTOR_H_
#define STATIONARYSONARDETECTOR_H_
/// ---------------------------------------------------------------------------
/// @file StationarySonardetector.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-01-21 19:13:37 syllogismrxs>
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
/// The BlankDetector class ...
///
/// ---------------------------------------------------------------------------
#include <map>
#include <list>

#include <opencv_workbench/detector/Detector.h>
#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/wb/ClusterProcess.h>
#include <opencv_workbench/wb/BlobProcess.h>
#include <opencv_workbench/wb/NDT.h>
#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/wb/Entity.h>
#include <opencv_workbench/wb/OpticalFlow.h>
#include <opencv_workbench/wb/RunningGaussian.h>
#include <opencv_workbench/trajectory/TrajectoryAnalysis.h>

//#include "/home/syllogismrxs/repos/3rd-party/bgslibrary/package_bgs/jmo/MultiLayerBGS.h"

class StationarySonarDetector : public Detector{
public:
     StationarySonarDetector();
     ~StationarySonarDetector();
     void print();

     int set_frame(int frame_number, const cv::Mat &original);  
     void set_stream(syllo::Stream *stream);
     
protected:     
     int thresh_value_;
     int grad_thresh_value_;
     BlobProcess blob_process_;
     ClusterProcess cluster_process_;
     NDT ndt_;

     cv::Mat erosionConfig_;
     cv::Mat dilationConfig_;

     std::map<int, std::list<wb::Blob> > tracks_history_;
     
     TrajectoryAnalysis traj_;
     OpticalFlow flow_;

     RunningGaussian rg_;

//MultiLayerBGS* bgs;
     
private:
};

extern "C" {
    Detector *maker();
}

#endif
