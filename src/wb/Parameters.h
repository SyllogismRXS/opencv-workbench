#ifndef PARAMETERS_H_
#define PARAMETERS_H_
/// ---------------------------------------------------------------------------
/// @file Parameters.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-06-15 16:59:47 syllogismrxs>
///
/// @version 1.0
/// Created: 26 Jan 2016
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
/// The Parameters class ...
/// 
/// ---------------------------------------------------------------------------

class Parameters {
public:

     typedef enum ThresholdType {
          static_type = 0,
          ratio_type = 1,
          gradient_type = 2
     }ThresholdType_t;
     
     Parameters();
     void set_yaml_file(std::string yaml_file);

     int history_length;
     double history_distance;
     double ratio_threshold;
     double static_threshold;
     double gradient_threshold;

     double min_velocity_threshold;
     double max_velocity_threshold;
     
     ThresholdType_t threshold_type;

     double covar_threshold;
     double max_leg_diff;
     double min_velocity_threshold_2;
     double min_relative_norm;

     int class_age_confirmed;
     
protected:

     std::string yaml_file_;
     
private:
};

#endif
