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
     cout << "Plot Tracks" << endl;     

     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input-file>" << endl;
          return -1;
     }
     
     // Setup Annotation Parser
     AnnotationParser parser;
     int status = parser.CheckForFile(argv[1], AnnotationParser::track);
     if (status != 0) {
          status = parser.CheckForFile(argv[1], AnnotationParser::hand);
          if (status != 0) {
               cout << "Error parsing tracks file." << endl;
               return -1;
          }
     }     
     
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
     cout << "a : all tracks over size" << endl;
     cout << " Type 'e' to end." << endl;     
     
     std::vector<std::string> to_plot;
     std::string name_str;
     //bool plot_all = false;
     do {
          cout << "$ " ;          
          std::cin >> name_str;
          if (name_str != "e") {
               to_plot.push_back(name_str);
          } else if (name_str == "a") {
               //plot_all = true;
               break;
          }
     } while (name_str != "e");
     
     //int id = syllo::str2int(name_str);
     //cout << "You selected ID (name): " << id << endl;          
     //to_plot.push_back(name_str);

     parser.plot_tracks(to_plot);     
     
     return 0;
}
