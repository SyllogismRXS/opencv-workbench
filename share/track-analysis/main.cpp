#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

//// OpenCV headers
//#include <cv.h>
//#include <highgui.h>
//
//#include <opencv2/contrib/contrib.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/syllo.h>
//
#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/plot/Plot.h>
#include <opencv_workbench/wb/PositionTracker.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     //syllo::Plot plot;
     //plot.gnuplot_test();
     //plot.gnuplot_test_2();
     cout << "Plot Tracks" << endl;     
     
     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input-file>" << endl;
          return -1;
     }
     
     // Setup Annotation Parser
     AnnotationParser parser;
     int status = parser.ParseFile(argv[1]);
     if (status != 0) {
          cout << "Error parsing tracks file." << endl;
          return -1;
     }
     //parser.print();
     
     //// Get list of all track IDs
     std::vector<std::string> names = parser.track_names();     
     
     //std::vector<std::string> to_plot;
     //int min_track_length = 0;
     //// Print out the list of object names
     //cout << "----------------------------------" << endl;
     //cout << "Which track do you want to plot?" << endl;
     //std::vector<std::string>::iterator it_names = names.begin();
     //for (; it_names != names.end(); it_names++) {
     //     cout << *it_names << endl;
     //}
     //cout << "-----------------" << endl;
     //cout << "a : all tracks" << endl;
     //cout << " Type 'e' to end." << endl;     
          
     //std::string name_str;
     //do {
     //     cout << "$ " ;          
     //     std::cin >> name_str;
     //     cout << "You entered: " << name_str << endl;
     //     if (name_str == "a") {
     //          cout << "Adding all tracks. Using min track length." << endl;
     //          min_track_length = 10;
     //          
     //          std::vector<std::string>::iterator it_names = names.begin();
     //          for (; it_names != names.end(); it_names++) {
     //               to_plot.push_back(*it_names);
     //          }
     //          break;
     //     } else if (name_str != "e") {
     //          to_plot.push_back(name_str);
     //     } 
     //} while (name_str != "e");
     
     //to_plot.push_back("unknown:18");
     
     //parser.plot_tracks(to_plot, min_track_length);        
     
     std::vector<wb::Entity> tracks = parser.get_tracks("unknown:18");
     //std::vector<wb::Entity> tracks = parser.get_tracks("unknown:47");
     PositionTracker tracker;

     // Copy the track centroids into a vector:
     std::vector<cv::Point3d> centroids;
     std::vector<cv::Point3d> estimates;

     std::vector<cv::Point3d> vels;
          
     int frame = 0;
     for (std::vector<wb::Entity>::iterator it = tracks.begin(); it != tracks.end(); it++) {
          cv::Point3d p;
          p.x = it->pixel_centroid().x;
          p.y = it->pixel_centroid().y;
          //p.x = it->centroid().x;
          //p.y = it->centroid().y;
          p.z = frame;
          centroids.push_back(p);
          
          tracker.predict();
          tracker.set_measurement(it->pixel_centroid());
          //tracker.set_measurement(it->centroid());
          
          cv::Point3d est;
          est.x = tracker.position().x;
          est.y = tracker.position().y;
          est.z = frame;

          estimates.push_back(est);          

          cv::Point3d vel;
          vel.x = tracker.velocity().x;
          vel.y = tracker.velocity().y;
          vel.z = frame;
          
          vels.push_back(vel);

          frame++;
     }
     
     /////////////////////////////////////////////////////////     
     // Plot the track positions
     std::vector< std::vector<cv::Point3d> > vectors;
     const std::string title = "Tracks";
     std::vector<std::string> labels;
     std::vector<std::string> styles;
     
     vectors.push_back(centroids);
     labels.push_back("Centroids");
     styles.push_back("points");       

     vectors.push_back(estimates);
     labels.push_back("Estimate");
     styles.push_back("points");       
     
     std::string options;
     options += "set xrange [0:300]\n";     
     options += "set yrange [0:300] reverse\n";     
     options += "set xlabel \"X\"\n";
     options += "set ylabel \"Y\"\n";
          
     syllo::Plot plot;     
     plot.plot(vectors, title, labels, styles, options, true);

     /////////////////////
     // Plot the track velocities
     /////////////////////////////////////////////////////////     
     // Plot the track positions
     vectors.clear();
     labels.clear();
     styles.clear();

     const std::string title2 = "Velocities";
          
     vectors.push_back(vels);
     labels.push_back("Velocities");
     styles.push_back("points");       

     std::string options2;
     options2 += "set xrange [*:*]\n";     
     options2 += "set yrange [*:*] reverse\n";     
     options2 += "set xlabel \"X\"\n";
     options2 += "set ylabel \"Y\"\n";
          
     syllo::Plot plot2;     
     plot2.plot(vectors, title2, labels, styles, options2, true);
     
     return 0;
}
