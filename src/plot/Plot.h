#ifndef PLOT_H_
#define PLOT_H_

// OpenCV headers
#include <cv.h>
#include <highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <opencv_workbench/plot/gnuplot-iostream.h>
#include <opencv_workbench/math/types.h>

namespace syllo
{

     class Plot {
     public:
          static void gnuplot_test();

          static void genTimeVector(double t0, double tStep, double tEnd, 
                                    std::vector<double> &tt);
          static void genTimeVector(int t0, int tStep, int tEnd, 
                                    std::vector<int> &tt);

          static std::vector<double> genTimeVector(double t0, double tStep, 
                                                   double tEnd);

          void wait();
                    
          void plot(std::vector< std::vector<state_5d_type> > &vectors, 
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles);

          int plot(std::vector<int> &time , std::vector<cv::Point> &data, 
                   const std::string &title);

          int plot(std::vector<double> &time, 
                   std::vector< std::vector<double> > &vectors, 
                   const std::string &title, 
                   std::vector<std::string> &labels);
          
          void plot(std::vector< std::vector<cv::Point2f> > &vectors, 
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles);  
          
          void plot(std::vector< std::vector<cv::Point2f> > &vectors, 
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles,
                    std::string options);

          void plot_heading_position(
               std::vector< std::vector<state_5d_type> > &vectors, 
               const std::string &title, 
               std::vector<std::string> &labels,
               std::vector<std::string> &styles);
          
          void gnuplot_test_2();

          

     private:
          Gnuplot gp_;
     };
}

#endif
