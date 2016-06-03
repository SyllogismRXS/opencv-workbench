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
#include <opencv_workbench/wb/ROC.h>
#include <yaml-cpp/yaml.h>

using std::cout;
using std::endl;

namespace fs = ::boost::filesystem;

int main(int argc, char *argv[])
{              
     std::string xml_dir = "";
     
     int c;          
     std::string output_dir = "";
     std::string param_sweep = "";
     std::string last_stage_str = "THRESHOLD"; // or "CLASSIFIER"
     while ((c = getopt (argc, argv, "g:o:d:")) != -1) {
          switch (c) {                         
          case 'o':
               output_dir = std::string(optarg);               
               break;          
          case 'd':
               xml_dir = std::string(optarg);
               break;          
          case 'g':
               last_stage_str = std::string(optarg);
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
                               file_paths, false);
     
     // Loop through all files. Summing TP, TN, FP, FN
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
               
          if (last_stage_str == "THRESHOLD") {          
               TP += metrics["PRE_TP"];
               TN += metrics["PRE_TN"];
               FP += metrics["PRE_FP"];
               FN += metrics["PRE_FN"];                    
          } else if (last_stage_str == "CLASSIFIER") {     
               TP += metrics["TP"];
               TN += metrics["TN"];
               FP += metrics["FP"];
               FN += metrics["FN"];                    
          } else {
               cout << "INVALID LAST STAGE DEFINED!" << endl;
          }          
     }

     double accuracy = (double)(TP + TN) / ((double)(TP + TN + FN + FP));
     cout << "Accuracy: " << accuracy << endl;

     double TPR = (double)TP / ((double)TP + (double)FN);
     double FPR = (double)FP / ((double)FP + (double)TN);
     double TNR = (double)TN / ((double)FP + (double)TN);
     double PPV = (double)TP / ((double)TP + (double)FP);
     double NPV = (double)TN / ((double)TN + (double)FN);
     double FDR = (double)FP / ((double)FP + (double)TP);
     double FNR = (double)FN / ((double)FN + (double)TP);
     double F1 = 2.0*(double)TP / (double)(2*TP+FP+FN);
     double MCC = ((double)TP*(double)TN - (double)FP*(double)FN) / sqrt(((double)TP+(double)FP)*((double)TP+(double)FN)*((double)TN+(double)FP)*((double)TN+(double)FN));
                                                  
     if (output_dir == "") {
          output_dir = xml_dir;
     }
     
     std::string out_filename = output_dir + "/test-results.txt";
     std::ofstream output;
     output.open(out_filename.c_str(), std::ofstream::out);
     output << "Accuracy: " << syllo::double2str(accuracy) << endl;
     output << "TPR: " << syllo::double2str(TPR) << endl;
     output << "FPR: " << syllo::double2str(FPR) << endl;

     output << "TNR: " << syllo::double2str(TNR) << endl;
     output << "PPV: " << syllo::double2str(PPV) << endl;
     output << "NPV: " << syllo::double2str(NPV) << endl;
     output << "FDR: " << syllo::double2str(FDR) << endl;
     output << "FNR: " << syllo::double2str(FNR) << endl;
     output << "F1: " << syllo::double2str(F1) << endl;
     output << "MCC: " << syllo::double2str(MCC) << endl;
     
     output << "TP: " << syllo::int2str(TP) << endl;
     output << "TN: " << syllo::int2str(TN) << endl;
     output << "FP: " << syllo::int2str(FP) << endl;
     output << "FN: " << syllo::int2str(FN) << endl;
     output.close();          
          
     return 0;
}
