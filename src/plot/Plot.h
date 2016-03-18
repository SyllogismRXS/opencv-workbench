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

#include <boost/shared_ptr.hpp>

namespace syllo
{
     class Plot {
     public:
          Plot();
          
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
                    std::vector<std::string> &styles,
                    bool enable_3d);

          int plot(std::vector<int> &time , std::vector<cv::Point> &data, 
                   const std::string &title);

          int plot(std::vector<double> &time, 
                   std::vector< std::vector<double> > &vectors, 
                   const std::string &title, 
                   std::vector<std::string> &labels,
                   bool enable_3d);
          
          void plot(std::vector< std::vector<cv::Point3d> > &vectors, 
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles,
                    bool enable_3d);  

          void plot(std::vector< std::vector<cv::Point3d> > &vectors, 
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles,
                    std::string options,
                    bool enable_3d);

          void plot(std::map< std::string, std::vector<cv::Point3d> > &vectors,
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles,
                    std::string options,
                    std::vector<std::string> &objects,
                    bool enable_3d);
          
          void plot(std::vector< std::vector<cv::Point3d> > &vectors, 
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles,
                    std::vector<std::string> &objects,
                    bool enable_3d);
          
          void plot(std::vector< std::vector<cv::Point3d> > &vectors, 
                    const std::string &title, 
                    std::vector<std::string> &labels,
                    std::vector<std::string> &styles,
                    std::string options,
                    std::vector<std::string> &objects,
                    bool enable_3d);

          void plot_heading_position(
               std::vector< std::vector<state_5d_type> > &vectors, 
               const std::string &title, 
               std::vector<std::string> &labels,
               std::vector<std::string> &styles);
          
          void gnuplot_test_2();

          

     private:
          boost::shared_ptr<Gnuplot> gp_;
          //Gnuplot gp_;
     };
}

#endif
