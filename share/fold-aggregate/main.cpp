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

PluginManager<Detector, Detector_maker_t> plugin_manager_;

bool myfunction (std::map<std::string,double> i, std::map<std::string,double> j) 
{
     if (i["FPR"] == j["FPR"]) {
          return (i["TPR"] < j["TPR"]);     
     } else {
          return (i["FPR"] < j["FPR"]);     
     }
}

bool myfunction_pre (std::map<std::string,double> i, std::map<std::string,double> j) 
{
     if (i["PRE_FPR"] == j["PRE_FPR"]) {
          return (i["PRE_TPR"] < j["PRE_TPR"]);     
     } else {
          return (i["PRE_FPR"] < j["PRE_FPR"]);     
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
     std::string param_sweep = "";
     while ((c = getopt (argc, argv, "s:f:o:d:")) != -1) {
          switch (c) {               
          case 's':
               param_sweep = std::string(optarg);
               break;
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

     if (param_sweep == "") {
          cout << "Error: Specify a parameter to sweep with -s" << endl;
          return -1;
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
     
          
     //std::vector< std::map<std::string,double> > metrics_vector;
     
     int threshold_type = -1;
     
     // Loop through all files.  use a map based on the current threshold
     // parameter to index the TP,TN,FP,FN count
     std::map<std::string, std::map<std::string,double> > metrics_counts;

     for (std::vector<fs::path>::iterator it = file_paths.begin(); 
          it != file_paths.end(); it++) {          
          
          AnnotationParser parser;
          int retcode = parser.ParseFile(it->string());
          if (retcode != 0) {
               cout << "Failed to parse: " << it->string();
               continue;
          }
          
          std::map<std::string,double> params = parser.get_params();

          threshold_type = (int)(params["threshold_type"]);

          if (params.count(param_sweep) > 0) {
               double param_value = params[param_sweep];
               std::string param_value_str = syllo::double2str(param_value);
               std::map<std::string,double> metrics = parser.get_metrics();
               
               metrics_counts[param_value_str]["PRE_TP"] += metrics["PRE_TP"];
               metrics_counts[param_value_str]["PRE_TN"] += metrics["PRE_TN"];
               metrics_counts[param_value_str]["PRE_FP"] += metrics["PRE_FP"];
               metrics_counts[param_value_str]["PRE_FN"] += metrics["PRE_FN"];
          } else {
               cout << "Error: Using invalid param_sweep: " << param_sweep << endl;
               return -1;
          }          
     }

     // For each of the threshold values, compute the TPR, FPR, Store in a
     // vector of maps
     std::vector<std::map<std::string, double> > metrics_vector;
     for (std::map<std::string, std::map<std::string,double> >::iterator it = metrics_counts.begin();
          it != metrics_counts.end(); it++) {
          
          std::map<std::string,double> metrics = it->second;          
          double PRE_TPR = metrics["PRE_TP"] / (metrics["PRE_TP"] + metrics["PRE_FN"]);
          double PRE_FPR = metrics["PRE_FP"] / (metrics["PRE_FP"] + metrics["PRE_TN"]);
          metrics["PRE_TPR"] = PRE_TPR;
          metrics["PRE_FPR"] = PRE_FPR;
          metrics["thresh_value"] = syllo::str2double(it->first);
          metrics_vector.push_back(metrics);
     }     
     
     std::sort(metrics_vector.begin(), metrics_vector.end(), myfunction_pre);
     
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
          //roc_stream << "# FP,FN,TP,TN,TPR,FPR" << endl;
          roc_stream << "# FPR,TPR,thresh,FP,FN,TP,TN,thresh_type" << endl;
     }
     
     for(std::vector< std::map<std::string,double> >::iterator it = metrics_vector.begin();
         it != metrics_vector.end(); it++) {                              
          roc_stream << (*it)["PRE_FPR"] <<","<< (*it)["PRE_TPR"] <<","
                     << (*it)["thresh_value"] <<"," << (*it)["PRE_FP"] <<","
                     << (*it)["PRE_FN"] <<","
                     << (*it)["PRE_TP"] <<","<< (*it)["PRE_TN"] << ","
                     << syllo::int2str(threshold_type) << endl;
     }     
     roc_stream.close();

     // Get the "optimal" operating point
     std::vector< std::map<std::string,double> >::iterator it_oppt;
     ROC::OperatingPoint(metrics_vector, it_oppt);
     
     // Write out the threshold to a yaml param file for the last validation
     // fold
     YAML::Emitter out;
     out << YAML::BeginMap;
     out << YAML::Key << param_sweep;
     out << YAML::Value << syllo::double2str((*it_oppt)["thresh_value"]);
     out << YAML::Key << "threshold_type";
     out << YAML::Value << syllo::int2str(threshold_type);     
     out << YAML::EndMap;

     std::string out_filename = output_dir + "/validate.yaml";
     std::ofstream output;
     output.open(out_filename.c_str(), std::ofstream::out);
     output << out.c_str();
     output.close();
     
     //double TPR = 0, FPR = 0;
     //TPR = (double)TP / (double)(TP + FN) ; 
     //FPR = (double)FP / (double)(FP + TN) ;      
     //// Display results
     //syllo::fill_line("+");
     //cout << "Totals... " << endl;
     //cout << "True Positives: " << TP << endl;
     //cout << "True Negatives: " << TN << endl;
     //cout << "False Positives: " << FP << endl;
     //cout << "False Negatives: " << FN << endl;     
     //cout << "True Positive Rate: " << TPR << endl;
     //cout << "False Positive Rate: " << FPR << endl;
     //syllo::fill_line("+");               
          
     return 0;
}
