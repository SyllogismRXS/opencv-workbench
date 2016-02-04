#ifndef ROC_H_
#define ROC_H_
/// ---------------------------------------------------------------------------
/// @file ROC.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-02-04 18:13:30 syllogismrxs>
///
/// @version 1.0
/// Created: 04 Feb 2016
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
/// The ROC class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <vector>
#include <map>

class ROC {
public:
     static void OperatingPoint(std::vector< std::map<std::string,double> > &metrics_vector,
                                std::vector< std::map<std::string,double> >::iterator &it_oppt)
     {
          double dist_champ = 1e9;
          std::vector< std::map<std::string,double> >::iterator it_champ;               
          
          double b = 1; // Y-intercept
          double m = 1; // Slope
          double x0 = -100;
          double xf = 100;
          cv::Point2f p1(x0, m*x0 + b);
          cv::Point2f p2(xf, m*xf + b);          
     
          double dist_champ_2 = 1e9;
          std::vector< std::map<std::string,double> >::iterator it_champ_2;
     
          for(std::vector< std::map<std::string,double> >::iterator it = metrics_vector.begin();
              it != metrics_vector.end(); it++) {          
                             
               cv::Point2f p((*it)["PRE_FPR"],(*it)["PRE_TPR"]);
               
               cv::Point2f goal(0,1);
               double dist = sqrt( pow(goal.x-p.x,2) + pow(goal.y-p.y,2) );
          
               if (dist < dist_champ) {
                    dist_champ = dist;
                    it_champ = it;
               }
                    
               cv::Point2f p3(p);
               double u = ((p3.x-p1.x)*(p2.x-p1.x) + (p3.y-p1.y)*(p2.y-p1.y)) / pow(cv::norm(p2-p1),2);
          
               cv::Point2f close_point(p1.x + u*(p2.x-p1.x) , p1.y + u*(p2.y-p1.y));
               double dist_2 = cv::norm(p3-close_point);
          
               if (dist_2 < dist_champ_2) {
                    dist_champ_2 = dist_2;
                    it_champ_2 = it;
               }                         
          }     

          cout << "Champ Threshold (Method 1): " << (*it_champ)["thresh_value"] << endl;     
          cout << "Champ Threshold (Method 2): " << (*it_champ_2)["thresh_value"] << endl; 

          std::vector< std::map<std::string,double> >::iterator it_champ_3;
          b = 1; // Y-intercept
          m = 1; // Slope
          x0 = -100;
          xf = 100;          
          dist_champ_2 = 1e9;     
          bool iterating = true;
          while (iterating && b > -0.2) {   
               cv::Point2f p1(x0, m*x0 + b);
               cv::Point2f p2(xf, m*xf + b);          
               for(std::vector< std::map<std::string,double> >::iterator it = metrics_vector.begin();
                   it != metrics_vector.end(); it++) {     

                    cv::Point2f p3((*it)["PRE_FPR"],(*it)["PRE_TPR"]);
                    double u = ((p3.x-p1.x)*(p2.x-p1.x) + (p3.y-p1.y)*(p2.y-p1.y)) / pow(cv::norm(p2-p1),2);
          
                    cv::Point2f close_point(p1.x + u*(p2.x-p1.x) , p1.y + u*(p2.y-p1.y));
                    double dist = cv::norm(p3-close_point);

                    if (std::abs(dist) < 0.001) {
                         it_champ_3 = it;                    
                         iterating = false;
                         break;
                    }
               }
               b -= 0.00001;
          }
          cout << "Champ Threshold (Method 3): " << (*it_champ_3)["thresh_value"] << endl; 
          
          it_oppt = it_champ_3;

          if ((*it_champ_3)["thresh_value"] != (*it_champ_2)["thresh_value"]) {
               cout << "=======================================================" << endl;
               cout << "Warning: thresh values don't match in fold-aggregate" << endl;
               cout << "=======================================================" << endl;
               it_oppt = it_champ_2;
          }
     }
protected:
private:     
};

#endif