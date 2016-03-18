#include <iostream>
#include <fstream>
#include <string>

#include<boost/tokenizer.hpp>
#include<boost/algorithm/string/predicate.hpp>

#include <map>

//#include <sam/common/plot/gnuplot_i.hpp>
#include <opencv_workbench/plot/gnuplot-iostream.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/math/types.h>

#include <boost/make_shared.hpp>

using std::cout;
using std::endl;

namespace syllo
{
     Plot::Plot()
     {
          gp_ = boost::make_shared<Gnuplot>("gnuplot -persist");
     }

     void Plot::plot(std::vector< std::vector<state_5d_type> > &vectors, 
                     const std::string &title, 
                     std::vector<std::string> &labels,
                     std::vector<std::string> &styles)
     {
          
     }

     void Plot::plot(std::vector< std::vector<cv::Point2d> > &vectors, 
                     const std::string &title, 
                     std::vector<std::string> &labels,
                     std::vector<std::string> &styles)
     {
          std::vector<std::string> objects;
          plot(vectors, title, labels, styles, std::string(""), objects);
     }
     
     void Plot::plot(std::vector< std::vector<cv::Point2d> > &vectors, 
                     const std::string &title, 
                     std::vector<std::string> &labels,
                     std::vector<std::string> &styles,
                     std::vector<std::string> &objects)
     {
          plot(vectors, title, labels, styles, std::string(""), objects);
     }

     void Plot::plot(std::vector< std::vector<cv::Point2d> > &vectors, 
                     const std::string &title, 
                     std::vector<std::string> &labels,
                     std::vector<std::string> &styles,
                     std::string options)
     {
          std::vector<std::string> objects;
          plot(vectors, title, labels, styles, options, objects);
     }
     
     void Plot::plot(std::vector< std::vector<cv::Point2d> > &vectors, 
                     const std::string &title, 
                     std::vector<std::string> &labels,
                     std::vector<std::string> &styles,
                     std::string options,
                     std::vector<std::string> &objects)
     {
          //gp_ << "plot sin(x)" << endl;                   
          //return;

          // used to plot "tracks"
          /// gp_ << "reset\n";
          /// //gp_ << "set terminal wxt\n";
          /// gp_ << "set title '" << title << "'\n";
          /// gp_ << "set size ratio -1\n";
          /// //gp_ << "set view equal xy\n";
          /// gp_ << "set grid xtics ytics\n";
          /// gp_ << "set size 1,1\n";
          /// gp_ << "set yrange [*:] reverse\n";          
          /// gp_ << "plot";

          (*gp_) << "reset\n";
          (*gp_) << "set title '" << title << "'\n";          
          (*gp_) << "set grid xtics ytics\n";   

          if (options != "") {
               (*gp_) << options;
          }

          //(*gp_) << "set size ratio -1\n";
          //(*gp_) << "set view equal xy\n";
          //(*gp_) << "set size 1,1\n";
          //(*gp_) << "set yrange [*:] reverse\n";          

          // Draw objects
          for(std::vector<std::string>::iterator it = objects.begin(); 
              it != objects.end(); it++) {
               (*gp_) << *it << endl;               
          }
          
          (*gp_) << "plot";

          int count = 0;
          std::vector< std::vector<cv::Point2d> >::iterator it;
          for (it = vectors.begin(); it != vectors.end(); it++) {
               std::vector<double> xLocs;
               std::vector<double> yLocs;
        	    
               std::vector<cv::Point2d>::iterator it2;
               for (it2 = it->begin() ; it2 != it->end() ; it2++) {
                    xLocs.push_back(it2->x);
                    yLocs.push_back(it2->y);
               } 

               (*gp_) << (*gp_).file1d(std::make_pair(xLocs,yLocs)) 
                   << "with " << styles[count]  << " title '" << labels[count] << "'";

               if (++it != vectors.end()) {
                    (*gp_) << ",";
               }
               --it;
               ++count;                    
          }
          (*gp_) << endl;
     }

     void Plot::plot(std::map< std::string, std::vector<cv::Point2d> > &vectors,
                     const std::string &title, 
                     std::vector<std::string> &labels,
                     std::vector<std::string> &styles,
                     std::string options,
                     std::vector<std::string> &objects)
     {
          //(*gp_) << "plot sin(x)" << endl;                   
          //return;

          // used to plot "tracks"
          /// (*gp_) << "reset\n";
          /// //(*gp_) << "set terminal wxt\n";
          /// (*gp_) << "set title '" << title << "'\n";
          /// (*gp_) << "set size ratio -1\n";
          /// //(*gp_) << "set view equal xy\n";
          /// (*gp_) << "set grid xtics ytics\n";
          /// (*gp_) << "set size 1,1\n";
          /// (*gp_) << "set yrange [*:] reverse\n";          
          /// (*gp_) << "plot";

          (*gp_) << "reset\n";
          (*gp_) << "set title '" << title << "'\n";          
          (*gp_) << "set grid xtics ytics\n";   

          if (options != "") {
               (*gp_) << options;
          }

          //(*gp_) << "set size ratio -1\n";
          //(*gp_) << "set view equal xy\n";
          //(*gp_) << "set size 1,1\n";
          //(*gp_) << "set yrange [*:] reverse\n";          

          // Draw objects
          for(std::vector<std::string>::iterator it = objects.begin(); 
              it != objects.end(); it++) {
               (*gp_) << *it << endl;               
          }
          
          (*gp_) << "plot";

          int count = 0;
          std::map< std::string, std::vector<cv::Point2d> >::iterator it;
          for (it = vectors.begin(); it != vectors.end(); it++) {
               std::vector<double> xLocs;
               std::vector<double> yLocs;
        	    
               std::vector<cv::Point2d>::iterator it2;
               for (it2 = it->second.begin() ; it2 != it->second.end() ; it2++) {
                    xLocs.push_back(it2->x);
                    yLocs.push_back(it2->y);
               } 

               (*gp_) << (*gp_).file1d(std::make_pair(xLocs,yLocs)) 
                   << "with linespoints title '" << it->first << "'";

               if (++it != vectors.end()) {
                    (*gp_) << ",";
               }
               --it;
               ++count;                    
          }
          (*gp_) << endl;
     }

     void Plot::plot_heading_position(
          std::vector< std::vector<state_5d_type> > &vectors, 
          const std::string &title, 
          std::vector<std::string> &labels,
          std::vector<std::string> &styles)
     {
          
          (*gp_) << "set title '" << title << "'\n";
          
          //// Plot arrows first;
          std::vector< std::vector<state_5d_type> >::iterator it;
          for (it = vectors.begin(); it != vectors.end(); it++) {
               std::vector<double> xLocs;
               std::vector<double> yLocs;
               std::vector<double> headings;
               
               std::vector<state_5d_type>::iterator it2;
               for (it2 = it->begin() ; it2 != it->end() ; it2++) {
                    state_5d_type state = *it2;
                    //xLocs.push_back(state[0]);
                    //yLocs.push_back(state[1]);
                    //headings.push_back(state[2]);
                    double hyp = 3;
                    double x = hyp*cos(state[2]);
                    double y = hyp*sin(state[2]);
                    (*gp_) << "set arrow from " << state[0] << "," << state[1] << " to " << state[0]+x << "," << state[1]+y << endl;
               }
          }
          
          (*gp_) << "plot";
          
          int count = 0;
          for (it = vectors.begin(); it != vectors.end(); it++) {
               std::vector<double> xLocs;
               std::vector<double> yLocs;
                                   
               std::vector<state_5d_type>::iterator it2;
               for (it2 = it->begin() ; it2 != it->end() ; it2++) {
                    state_5d_type state = *it2;
                    xLocs.push_back(state[0]);
                    yLocs.push_back(state[1]);                    
               } 
          
               (*gp_) << (*gp_).file1d(std::make_pair(xLocs,yLocs)) 
                   << "with " << styles[count]  << " title '" 
                   << labels[count] << "'";
               
               if (++it != vectors.end()) {
                    (*gp_) << ",";
               } 
               --it;
               ++count;                    
          }
          (*gp_) << endl;
     }
     
     void Plot::gnuplot_test_2()          
     {
          std::vector<double> tt;
          std::vector<std::pair<double,double> > vec;
          
          genTimeVector(0, 0.01, 2, tt);
          
          std::vector<double>::iterator it;
          for(it = tt.begin(); it != tt.end(); it++) {
               vec.push_back(std::make_pair(*it,sin(2*3.14159265359*(*it))));
          }
          
          (*gp_) << "plot" << (*gp_).file1d(vec) << "with lines title 'My Title'" << endl;
     }
     void Plot::gnuplot_test()
     {
          //Gnuplot gp;
          Gnuplot gp("gnuplot -persist");
          // Create a script which can be manually fed into gnuplot later:
          //    Gnuplot gp(">script.gp");
          // Create script and also feed to gnuplot:
          //    Gnuplot gp("tee plot.gp | gnuplot -persist");
          // Or choose any of those options at runtime by setting the GNUPLOT_IOSTREAM_CMD
          // environment variable.

          // Gnuplot vectors (i.e. arrows) require four columns: (x,y,dx,dy)
          std::vector<boost::tuple<double, double, double, double> > pts_A;

          // You can also use a separate container for each column, like so:
          std::vector<double> pts_B_x;
          std::vector<double> pts_B_y;
          std::vector<double> pts_B_dx;
          std::vector<double> pts_B_dy;

          // You could also use:
          //   std::vector<std::vector<double> >
          //   boost::tuple of four std::vector's
          //   std::vector of std::tuple (if you have C++11)
          //   arma::mat (with the Armadillo library)
          //   blitz::Array<blitz::TinyVector<double, 4>, 1> (with the Blitz++ library)
          // ... or anything of that sort

          for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
               double theta = alpha*2.0*3.14159;
               pts_A.push_back(boost::make_tuple(
                                    cos(theta),
                                    sin(theta),
                                    -cos(theta)*0.1,
                                    -sin(theta)*0.1
                                    ));

               pts_B_x .push_back( cos(theta)*0.8);
               pts_B_y .push_back( sin(theta)*0.8);
               pts_B_dx.push_back( sin(theta)*0.1);
               pts_B_dy.push_back(-cos(theta)*0.1);
          }

          // Don't forget to put "\n" at the end of each line!
          gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
          // '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
          gp << "plot '-' with vectors title 'pts_A', '-' with vectors title 'pts_B'\n";
          gp.send1d(pts_A);
          gp.send1d(boost::make_tuple(pts_B_x, pts_B_y, pts_B_dx, pts_B_dy));          
     }
     
     void Plot::genTimeVector(double t0, double tStep, double tEnd, std::vector<double> &tt)
     {
	  for (double i = t0 ; i < tEnd ; i += tStep) {
	       tt.push_back(i);
	  }
     }

     std::vector<double> Plot::genTimeVector(double t0, double tStep, double tEnd)
     {
	  std::vector<double> tt;
	  for (double i = t0 ; i < tEnd ; i += tStep) {
	       tt.push_back(i);
	  }
	  return tt;
     }

     void Plot::genTimeVector(int t0, int tStep, int tEnd, std::vector<int> &tt)
     {
	  for (int i = t0 ; i <= tEnd ; i += tStep) {
	       tt.push_back(i);
	  }
     }

     //int plotCentroids(std::map<int,syllo::Cluster> &clusters, int clusterID)
     //{
     //     std::vector<int> tt;
     //     std::vector<cv::Point> prevCentroids;
     //
     //     prevCentroids = clusters[clusterID].prevCentroids();
     //
     //     genTimeVector(0,1,prevCentroids.size()-1, tt);
     //
     //     //cout << "Size of tt:" << tt.size() << endl;
     //     //cout << "Size of data:" << prevCentroids.size() << endl;
     //
     //     int status;
     //     status = plot(tt, prevCentroids, "Centroids");
     //     if (status == -1) {
     //          return 1;
     //     } else {
     //          return status;
     //     }
     //}
     
     //int plot(std::vector<int> &time , std::vector<cv::Point> &data, const std::string &title)
     //{
     //     try{
     //          Gnuplot g1("lines");
     //          
     //          std::vector<int> xLocs, yLocs;
     //
     //          std::vector<cv::Point>::iterator it;
     //          for (it = data.begin() ; it != data.end() ; it++) {
     //   	    xLocs.push_back(it->x);
     //   	    yLocs.push_back(it->y);
     //          } 
     //
     //          g1.set_style("points").plot_xy(xLocs,yLocs,"Centroid Locations");
     //   
     //          //g1.unset_grid();
     //          g1.plot_xyz(xLocs,yLocs,time,title);
     //
     //          g1.showonscreen(); // window output
     //
     //          return 0;
     //
     //     }
     //     catch (GnuplotException ge)
     //     {
     //          cout << ge.what() << endl;
     //          return -1;
     //     }
     //
     //}
     //
     ////int wait_for_key ()
     ////{
     ////     int status;
     ////     cout << endl << "Enter a value:" << endl;
     ////     std::cin.clear();
     ////     std::cin.ignore(std::cin.rdbuf()->in_avail());
     ////     //std::cin.get()
     ////     std::cin >> status;
     ////     return status;
     ////}
     //
     //void wait()
     //{
     //     std::cin.ignore(std::cin.rdbuf()->in_avail());
     //     std::cin.get();
     //}
     //
     //
     //int plot(std::vector<double> &time, std::vector< std::vector<double> > &vectors, const std::string &title, std::vector<std::string> &labels)
     //{
     //     try{
     //          Gnuplot g1;
     //          g1.reset_plot();
     //          g1.set_grid();
     //     
     //          g1.set_title(title);               
     //
     //          int count = 0;
     //          std::vector< std::vector<double> >::iterator it;
     //          for (it = vectors.begin(); it != vectors.end(); it++) {
     //   	    g1.set_style("lines").plot_xy(time,*it,labels[count++]);
     //          }
     //
     //          g1.showonscreen(); // window output
     //          
     //          return 0;
     //
     //     }
     //     catch (GnuplotException ge)
     //     {
     //          cout << ge.what() << endl;
     //          return -1;
     //     }
     //}
     //
     //int plot(std::vector< std::vector<cv::Point> > &vectors, 
     //         const std::string &title, 
     //         std::vector<std::string> &labels)
     //{
     //     try{
     //          Gnuplot g1;
     //          g1.reset_plot();
     //          g1.set_grid();
     //     
     //          g1.set_title(title);
     //          
     //          int count = 0;
     //          std::vector< std::vector<cv::Point> >::iterator it;
     //          for (it = vectors.begin(); it != vectors.end(); it++) {
     //   	    std::vector<double> xLocs;
     //   	    std::vector<double> yLocs;
     //   	    
     //   	    std::vector<cv::Point>::iterator it2;
     //   	    cout << "----------Counts: " << it->size() << endl;
     //   	    for (it2 = it->begin() ; it2 != it->end() ; it2++) {
     //   		 xLocs.push_back(it2->x);
     //   		 yLocs.push_back(it2->y);
     //   	    }
     //
     //   	    g1.set_style("lines").plot_xy(xLocs,yLocs,labels[count++]);
     //   	    //g1.set_style("dots").plot_xy(xLocs,yLocs,labels[count++]);
     //          }
     //
     //          g1.showonscreen(); // window output
     //
     //          return 0;
     //
     //     }
     //     catch (GnuplotException ge)
     //     {
     //          cout << ge.what() << endl;
     //          return -1;
     //     }
     //}
     //
     ////int plotTrajClusters(std::vector< std::vector<cv::Point> > &trajectories)
     ////{
     ////     try{
     ////          Gnuplot g1;
     ////          g1.reset_plot();
     ////          g1.set_grid();
     ////          g1.set_title("Trajectories");
     ////          g1.set_xlabel("X");
     ////          g1.set_ylabel("Y");
     ////          g1.set_zlabel("Frame");
      ////          
     ////          
     ////          int count = 0;
     ////          std::vector< std::vector<cv::Point> >::iterator it;
     ////          for (it = trajectories.begin(); it != trajectories.end(); it++) {
     ////   	    std::vector<double> xLocs;
     ////   	    std::vector<double> yLocs;
     ////   	    
     ////   	    std::vector<cv::Point>::iterator it2;
     ////               for (it2 = it->begin() ; it2 != it->end() ; it2++) {
     ////   		 xLocs.push_back(it2->x);
     ////   		 yLocs.push_back(it2->y);
     ////               }
     ////
     ////   	    //g1.set_style("lines").plot_xy(xLocs,yLocs,labels[count++]);
     ////   	    //g1.set_style("dots").plot_xy(xLocs,yLocs,labels[count++]);
     ////               std::vector<double> tt;
     ////               genTimeVector(0,1,xLocs.size(),tt);
     ////               g1.set_style("lines").plot_xyz(xLocs,yLocs,tt);
     ////          }
     ////
     ////          g1.showonscreen(); // window output
     ////          wait();
     ////          return 0;
     ////     }
     ////     catch (GnuplotException ge)
     ////     {
     ////          cout << ge.what() << endl;
     ////          return -1;
     ////     }
     ////}
     //
     //int plotTrajClusters2(std::vector< std::vector<cv::Point> > &trajectories)
     //{
     ////  gnuplot_ctrl * h;
     ////  h = gnuplot_init();
     ////
     ////  
     ////
     ////  try{
     ////       Gnuplot g1;
     ////       g1.reset_plot();
     ////       g1.set_grid();
     ////       g1.set_title("Trajectories");
     ////       
     ////       int count = 0;
     ////       std::vector< std::vector<cv::Point> >::iterator it;
     ////       for (it = trajectories.begin(); it != trajectories.end(); it++) {
     ////	    std::vector<int> xLocs;
     ////	    std::vector<int> yLocs;
     ////	    
     ////	    std::vector<cv::Point>::iterator it2;
     ////            for (it2 = it->begin() ; it2 != it->end() ; it2++) {
     ////		 xLocs.push_back(it2->x);
     ////		 yLocs.push_back(it2->y);
     ////	    }
     ////
     ////	    //g1.set_style("lines").plot_xy(xLocs,yLocs,labels[count++]);
     ////	    //g1.set_style("dots").plot_xy(xLocs,yLocs,labels[count++]);
     ////            std::vector<int> tt;
     ////            genTimeVector(0,1,xLocs.size()-1,tt);
     ////            g1.set_style("lines").plot_xyz(xLocs,yLocs,tt);
     ////       }
     ////
     ////       g1.showonscreen(); // window output
     ////       wait();
     ////       return 0;
     ////  }
     ////  catch (GnuplotException ge)
     ////  {
     ////       cout << ge.what() << endl;
     ////       return -1;
     ////  }
     //     return -1;
     //}

}
