#ifndef DIVERCLASSIFIER_H_
#define DIVERCLASSIFIER_H_
/// ---------------------------------------------------------------------------
/// @file DiverClassifier.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-05-26 14:20:09 syllogismrxs>
///
/// @version 1.0
/// Created: 26 May 2016
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
/// The DiverClassifier class ...
/// 
/// ---------------------------------------------------------------------------
#include <vector>
#include <opencv_workbench/wb/Blob.h>

class DiverClassifier {
public:
     DiverClassifier();     

     void process_frame(cv::Mat &gray,
                        std::vector<wb::Blob> &tracks, 
                        std::vector<wb::Blob> &blobs,
                        int frame_number);                        

     std::vector<wb::Entity> &estimated_divers()
     {
          return estimated_divers_;
     }

protected:
     
     std::vector<wb::Entity> estimated_divers_;

private:
};

#endif
