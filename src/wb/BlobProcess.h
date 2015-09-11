#ifndef BLOBPROCESS_H_
#define BLOBPROCESS_H_
/// ---------------------------------------------------------------------------
/// @file BlobProcess.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-09-10 19:28:42 syllogismrxs>
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

#include <cv.h>
#include <highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Blob.h"

class BlobProcess {
public:
     BlobProcess();
     
     int process_frame(cv::Mat &input, cv::Mat &output);
     
protected:
     
     uchar valueAt(cv::Mat &img, int row, int col);
     uchar findMin(uchar NE, uchar N, uchar NW, uchar W);
     void labelNeighbors(cv::Mat &img, std::vector<uchar> &labelTable, 
                              uchar label, int i, int j);

     std::map<int,wb::Blob> blobs_;

private:
     int count_;
};

#endif
