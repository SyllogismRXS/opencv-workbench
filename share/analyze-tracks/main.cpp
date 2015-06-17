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
     cout << "Analyze Tracks" << endl;     

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
     
     // Which track exhibited the most displacement?
     
     
     // Which track is the oldest?
     
     return 0;
}
