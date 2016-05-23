#ifndef FINDETECTOR_H_
#define FINDETECTOR_H_
/// ---------------------------------------------------------------------------
/// @file FinDetector.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-05-23 16:27:51 syllogismrxs>
///
/// @version 1.0
/// Created: 18 May 2016
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
/// The FinDetector class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <fstream>

#include <vector>
#include <opencv_workbench/wb/Blob.h>

class FinDetector {
public:
     FinDetector();
     void process_frame(cv::Mat &gray, cv::Mat &src, cv::Mat &dst, 
                        std::vector<wb::Blob> &tracks, 
                        std::vector<wb::Blob> &blobs,
                        int frame_number,
                        std::vector<wb::Blob> &short_lived);
protected:
     
     std::ofstream output_file_;
     cv::Mat prev_roi_;     
private:
};

#endif
