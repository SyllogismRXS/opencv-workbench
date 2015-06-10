#ifndef AnnotationParser_H_
#define AnnotationParser_H_
/// ---------------------------------------------------------------------------
/// @file AnnotationParser.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-06-10 15:40:04 syllogismrxs>
///
/// @version 1.0
/// Created: 29 Apr 2015
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
/// The AnnotationParser class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <map>
#include <opencv_workbench/utils/Frame.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

class AnnotationParser {
public:

     typedef enum {
          hand = 0,
          track = 1,
     }AnnotateType_t;
     
     AnnotationParser();
     int ParseFile(std::string file);
     int CheckForFile(std::string video_file, AnnotateType_t ann_type);
     void reset();
     void write_annotation();
     void print();
     bool export_roi();
     void clear();

     void set_width(int width) { width_ = width; }
     void set_height(int height) { height_ = height; }
     void set_depth(int depth) { depth_ = depth; }
     void set_number_of_frames(int num) { number_of_frames_ = num; }
     void set_type(std::string type) { type_ = type; }
     void set_video_filename(std::string video_filename) { video_filename_ = video_filename; }
     void set_folder(std::string folder) { folder_ = folder; }     
     void write_header();

     std::map<int,Frame> frames;

protected:
     std::string xml_filename_;     
     AnnotateType_t ann_type_;
     
     std::string folder_;
     std::string type_;
     std::string video_filename_;
     int width_;
     int height_;
     int depth_;     
     int number_of_frames_;

     fs::path dir_;
     fs::path basename_;
     
private:
};

#endif
