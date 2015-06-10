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
          cout << "Error parsing tracks file." << endl;
          return -1;
     }     
     
     // Get list of all track IDs
     std::map<std::string,std::string> IDs;
     std::map<int,Frame>::iterator it_frame = parser.frames.begin();
     for (; it_frame != parser.frames.end(); it_frame++) {
          Frame frame = it_frame->second;
          
          // Loop through all objects in each frame
          std::map<std::string, Object>::iterator it_obj = frame.objects.begin();
          for (; it_obj != frame.objects.end(); it_obj++) {
               IDs[it_obj->first] = it_obj->first;
          }          
     }

     // Print out the list of object names
     cout << "----------------------------------" << endl;
     cout << "Which track do you want to plot?" << endl;
     std::map<std::string,std::string>::iterator it_IDs = IDs.begin();
     for (; it_IDs != IDs.end(); it_IDs++) {
          cout << it_IDs->first << endl;
     }
     cout << "-----------------" << endl;
     cout << "$ " ;

     std::string id_str;
     std::cin >> id_str;

     int id = syllo::str2int(id_str);
     cout << "You selected ID: " << id << endl;

     std::vector<cv::Point> points; 
     
     // Loop through all frames, plotting tracks that match the user's input
     // Get list of all track IDs
     it_frame = parser.frames.begin();
     for (; it_frame != parser.frames.end(); it_frame++) {
          Frame frame = it_frame->second;
          
          // Loop through all objects in each frame
          std::map<std::string, Object>::iterator it_obj = frame.objects.begin();
          for (; it_obj != frame.objects.end(); it_obj++) {
               // Does this object name match?
               if (it_obj->first == id_str) {
                    points.push_back(it_obj->second.bbox.centroid());
               }                
          }          
     }

     // Plot the tracks;
     std::vector< std::vector<cv::Point> > vectors;
     const std::string title = "Tracks";
     std::vector<std::string> labels;
     std::vector<std::string> styles;

     vectors.push_back(points);
     labels.push_back(id_str);
     styles.push_back("points");     
     
     syllo::Plot plot;
     //plot.gnuplot_test();
     plot.plot(vectors, title, labels, styles);  


     
     //std::string temp;
     //std::cin >> temp;
     
     return 0;
}
