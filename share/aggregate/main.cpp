#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <algorithm>
#include <vector>
#include <map>

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

bool myfunction (std::map<std::string,double> i, std::map<std::string,double> j) 
{
     if (i["FPR"] == j["FPR"]) {
          return (i["TPR"] < j["TPR"]);     
     } else {
          return (i["FPR"] < j["FPR"]);     
     }
}

int main(int argc, char *argv[])
{              
     std::string xml_dir = "";
     
     int c;
     int create_roc_file = 0;
     int output_file_set = 0;
     std::string output_file = "";
     std::string output_dir = "./";
     while ((c = getopt (argc, argv, "f:o:d:")) != -1) {
          switch (c) {
          //case 'a':
          //     aflag = 1;
          //     break;
          //case 'b':
          //     bflag = 1;
          //     break;
          case 'o':
               output_dir = std::string(optarg);
               create_roc_file = 1;
               break;
          case 'f':
               output_file = std::string(optarg);
               output_file_set = 1;
               break;
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
                               file_paths,true);   
     
          
     std::vector< std::map<std::string,double> > metrics_vector;
     
     int TP = 0, TN = 0, FP = 0, FN = 0;        
     for (std::vector<fs::path>::iterator it = file_paths.begin(); 
          it != file_paths.end(); it++) {
          cout << "File: " << it->string() << endl;          
          
          AnnotationParser parser;
          int retcode = parser.ParseFile(it->string());
          if (retcode != 0) {
               cout << "Failed to parse: " << it->string();
               continue;
          }
          std::map<std::string,double> metrics = parser.get_metrics();          
          
          metrics_vector.push_back(metrics);          
     }     
     
     std::sort(metrics_vector.begin(), metrics_vector.end(), myfunction);     

     std::string roc_fn;
     if (output_file_set == 1) {
          roc_fn = output_dir + "/" + output_file;
     } else {
          // time string generation
          time_t rawtime;
          struct tm * timeinfo;
          char buffer [80];     
          time (&rawtime);
          timeinfo = localtime (&rawtime);     
          strftime (buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
          std::string time_str = std::string(buffer);
     
          roc_fn = output_dir + "/" + time_str + "_ROC.csv";
     }
     
     std::ofstream roc_stream;
     if (create_roc_file) {
          cout << "ROC Output file: " << roc_fn << endl;
          roc_stream.open (roc_fn.c_str(), std::ofstream::out);
          roc_stream << "# FPR,TPR,thresh,FP,FN,TP,TN," << endl;
     }

     for(std::vector< std::map<std::string,double> >::iterator it = metrics_vector.begin();
         it != metrics_vector.end(); it++) {          
          
          // 0 for threshold place holder for now: TODO
          roc_stream << (*it)["FPR"] <<","<< (*it)["TPR"] << ",0,"
                     << (*it)["FP"] <<","<< (*it)["FN"] <<","
                     << (*it)["TP"] <<","<< (*it)["TN"] <<","
                     << endl;
          
          TP += (*it)["TP"];
          TN += (*it)["TN"];
          FP += (*it)["FP"];
          FN += (*it)["FN"];
     }
     
     roc_stream.close();
     
     double TPR = 0, FPR = 0;
     
     TPR = (double)TP / (double)(TP + FN) ; 
     FPR = (double)FP / (double)(FP + TN) ;      
     
     // Display results
     syllo::fill_line("+");
     cout << "Totals... " << endl;
     cout << "True Positives: " << TP << endl;
     cout << "True Negatives: " << TN << endl;
     cout << "False Positives: " << FP << endl;
     cout << "False Negatives: " << FN << endl;     
     cout << "True Positive Rate: " << TPR << endl;
     cout << "False Positive Rate: " << FPR << endl;
     syllo::fill_line("+");               
          
     return 0;
}
