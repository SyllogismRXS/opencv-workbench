#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/syllo.h>

#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/plot/Plot.h>

#include <opencv_workbench/detector/Detector.h>
#include <opencv_workbench/plugin_manager/PluginManager.h>

#include <opencv_workbench/syllo/syllo.h>

using std::cout;
using std::endl;

PluginManager<Detector, Detector_maker_t> plugin_manager_;

int main(int argc, char *argv[])
{    
     cout << "Aggregate." << endl;
     
     std::string xml_dir = "";
     
     int c;
     while ((c = getopt (argc, argv, "d:")) != -1) {
          switch (c) {
          //case 'a':
          //     aflag = 1;
          //     break;
          //case 'b':
          //     bflag = 1;
          //     break;
          case 'd':
               xml_dir = std::string(optarg);
               break;
          case '?':
               if (optopt == 'd') {
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
               } else if (isprint (optopt)) {
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
               } else {
                    fprintf (stderr,
                             "Unknown option character `\\x%x'.\n",
                             optopt);
               }
               return 1;
          default:
               abort ();
          }
     }

     // Check existence of xml directory
     if ( !boost::filesystem::exists( fs::path(xml_dir) ) ) {
          cout << "Error: XML Directory doesn't exist." << endl;
          return -1;
     }
     
     // Open each xml file and count metrics
     std::vector<fs::path> file_paths;
     syllo::get_files_with_ext(fs::path(xml_dir), ".xml", 
                               file_paths);     
     
     int TP = 0, TN = 0, FP = 0, FN = 0;
     std::vector<fs::path>::iterator it = file_paths.begin();
     for (; it != file_paths.end(); it++) {
          cout << "File: " << it->string() << endl;
          
          AnnotationParser parser;
          int retcode = parser.ParseFile(it->string());
          if (retcode != 0) {
               cout << "Failed to parse: " << it->string();
               continue;
          }
          std::map<std::string,int> metrics = parser.get_metrics();
          TP += metrics["TP"];
          TN += metrics["TN"];
          FP += metrics["FP"];
          FN += metrics["FN"];
     }
     
     double TPR = 0, FPR = 0;
     
     TPR = (double)TP / (double)(TP + FN) ; 
     FPR = (double)FP / (double)(FP + TN) ;      
     
     // Display results
     syllo::fill_line("+");
     cout << "True Positives: " << TP << endl;
     cout << "True Negatives: " << TN << endl;
     cout << "False Positives: " << FP << endl;
     cout << "False Negatives: " << FN << endl;     
     cout << "True Positive Rate: " << TPR << endl;
     cout << "False Positive Rate: " << FPR << endl;
     syllo::fill_line("+");     
     
     cout << "Finished aggregating results" << endl;
          
     return 0;
}
