#ifndef SYLLOQT_H_
#define SYLLOQT_H_
/// ---------------------------------------------------------------------------
/// @file SylloQt.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-04-27 17:38:12 syllogismrxs>
///
/// @version 1.0
/// Created: 27 Apr 2015
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
/// The SylloQt class ...
/// 
/// ---------------------------------------------------------------------------

#include <QtGui>
#include <QResource>
#include <QSettings>

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class SylloQt {
public:
     static QImage Mat2QImage(cv::Mat const& src)
     {
          cv::Mat temp; // make the same cv::Mat
          cv::cvtColor(src, src,CV_BGR2RGB); // cvtColor Makes a copy, that what i need
          QImage dest((uchar*) src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
          dest.detach(); //enforce deep copy
          return dest;
     }

     static cv::Mat QImage2Mat(QImage const& src)
     {
          cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
          cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
          cv::cvtColor(tmp, result,CV_BGR2RGB);
          return result;
     }
protected:
private:
};

#endif
