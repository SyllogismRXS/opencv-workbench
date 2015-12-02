#ifndef BLOBPROCESS_H_
#define BLOBPROCESS_H_
/// ---------------------------------------------------------------------------
/// @file BlobProcess.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-12-02 15:51:54 syllogismrxs>
///
/// @version 1.0
/// Created: 10 Sep 2015
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
/// The BlobProcess class ...
/// 
/// ---------------------------------------------------------------------------
#include <map>
#include <vector>

#include <cv.h>
#include <highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Blob.h"

typedef enum OverlayFlags {
     BLOBS       = 1 << 0,
     RECTS       = 1 << 1,
     TRACKS      = 1 << 2,
     IDS         = 1 << 3,
     ERR_ELLIPSE = 1 << 4
}OverlayFlags_t;
     
inline OverlayFlags_t operator|(OverlayFlags_t a, OverlayFlags_t b)
{
     return static_cast<OverlayFlags_t>(static_cast<int>(a) | 
                                        static_cast<int>(b));
}

class BlobProcess {
public:          
     
     BlobProcess();
     
     int process_frame(cv::Mat &input, cv::Mat &original, int thresh);
     void find_blobs(cv::Mat &input, std::vector<wb::Blob> &blobs, 
                            int min_blob_size);
     
     void overlay_blobs(cv::Mat &src, cv::Mat &dst, 
                        std::vector<wb::Blob> & blobs);
     void overlay_blobs(cv::Mat &src, cv::Mat &dst);
     void overlay_tracks(cv::Mat &src, cv::Mat &dst);  

     void overlay(cv::Mat &src, cv::Mat &dst, OverlayFlags_t flags);  
     
     std::vector<wb::Blob> & blobs() { return blobs_; }

     bool consolidate_tracks(cv::Mat &in, cv::Mat &out);
     
protected:
     
     static uchar valueAt(cv::Mat &img, int row, int col);
     static uchar findMin(uchar NE, uchar N, uchar NW, uchar W);
     static void labelNeighbors(cv::Mat &img, std::vector<uchar> &labelTable, 
                              uchar label, int i, int j);
     int next_available_id();

     void blob_maintenance();

     //std::map<int,wb::Blob> blobs_;
     //std::map<int,wb::Blob> prev_blobs_;
     std::vector<wb::Blob> blobs_;
     std::vector<wb::Blob> prev_blobs_;

private:
     int count_;
     int min_blob_size_;
     int next_id_;
};

#endif
