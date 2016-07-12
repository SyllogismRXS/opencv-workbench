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
     
     // Print out the list of object names
     cout << "----------------------------------" << endl;
     cout << "Which track do you want to plot?" << endl;
     std::vector<std::string>::iterator it_names = names.begin();
     for (; it_names != names.end(); it_names++) {
          cout << *it_names << endl;
     }
     cout << "-----------------" << endl;
     cout << "a : all tracks" << endl;
     cout << " Type 'e' to end." << endl;     
     
     int min_track_length = 0;
     
     std::vector<std::string> to_plot;
     std::string name_str;
     do {
          cout << "$ " ;          
          std::cin >> name_str;
          cout << "You entered: " << name_str << endl;
          if (name_str == "a") {
               cout << "Adding all tracks. Using min track length." << endl;
               min_track_length = 5;//10;
               
               std::vector<std::string>::iterator it_names = names.begin();
               for (; it_names != names.end(); it_names++) {
                    to_plot.push_back(*it_names);
               }
               break;
          } else if (name_str != "e") {
               to_plot.push_back(name_str);
          } 
     } while (name_str != "e");
     //to_plot.push_back("unknown:1");
     //to_plot.push_back("unknown:4");
     
     parser.plot_tracks(to_plot, min_track_length);        
     
     return 0;
}
