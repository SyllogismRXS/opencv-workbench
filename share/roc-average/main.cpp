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

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

using std::cout;
using std::endl;

PluginManager<Detector, Detector_maker_t> plugin_manager_;

struct ROCData{
     double TPR;
     double FPR;
     double threshold;
     double TPR_error_low;
     double TPR_error_high;
     double FPR_error_low;
     double FPR_error_high;
};

bool myfunction (struct ROCData i, struct ROCData j) 
{
     if (i.FPR == j.FPR) {
          return (i.TPR < j.TPR);
     } else {
          return (i.FPR < j.FPR);
     }
}

int main(int argc, char *argv[])
{              
     std::string work_dir = "";
     
     int c;
     std::string param_sweep = "";
     std::string output_file = "";
     std::string output_dir = "./";
     while ((c = getopt (argc, argv, "s:f:o:d:")) != -1) {
          switch (c) {
          case 's':
               param_sweep = std::string(optarg);
               break;
          case 'o':
               output_dir = std::string(optarg);               
               break;
          case 'f':
               output_file = std::string(optarg);               
               break;
          case 'd':
               work_dir = std::string(optarg);
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
     if ( !boost::filesystem::exists( fs::path(work_dir) ) ) {
          cout << "Error: XML Directory doesn't exist." << endl;
          return -1;
     }
     
     std::string threshold_type = "NOTSET";
     
     // Key = threshold (as string)
     std::map<std::string, std::vector<struct ROCData> > all_data;
     boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
     for( boost::filesystem::directory_iterator i( work_dir ); i != end_itr; ++i )
     {
          // Skip if not a file
          //if( !boost::filesystem::is_regular_file( i->status() ) ) continue;
     
          boost::smatch what;
     
          std::string str = i->path().stem().string();
                    
          // Skip if no match
          if (str.compare(0,4,"fold") != 0 ) continue;          
     
          std::string roc_file = i->path().string() + "/tracks/roc.csv";

          // Check existence of roc file
          if ( !boost::filesystem::exists( fs::path(roc_file) ) ) {
               cout << "Error: ROC File Directory doesn't exist: " << roc_file << endl;
               continue;
          }

          // Open the file and read the data
          std::ifstream file(roc_file.c_str());
          
          std::string TPR, FPR, threshold,FP,FN,TP,TN;
          while (std::getline(file, FPR, ',')) {

               if (FPR.compare(0,1,"#") == 0) {
                    // Throw away comment lines
                    std::getline(file, FPR) ;
                    continue;
               }                             

               std::getline(file, TPR, ',') ;               
               std::getline(file, threshold, ',') ;               

               std::getline(file, FP, ',');
               std::getline(file, FN, ',');
               std::getline(file, TP, ',');
               std::getline(file, TN, ',');
               std::getline(file, threshold_type);

               struct ROCData d;
               d.TPR = syllo::str2double(TPR);
               d.FPR = syllo::str2double(FPR);
               d.threshold = syllo::str2double(threshold);
               
               all_data[syllo::double2str(d.threshold)].push_back(d);
          }         
     }

     // Compute mean and std for each threshold point
     std::vector< struct ROCData > final;
     for(std::map<std::string, std::vector<struct ROCData> >::iterator it1 = all_data.begin();
         it1 != all_data.end(); it1++) {
          double TPR_sum = 0;
          double FPR_sum = 0;
          for (std::vector<struct ROCData>::iterator it2 = it1->second.begin();
               it2 != it1->second.end(); it2++) {
               if (it2->threshold != syllo::str2double(it1->first)) {
                    cout << "ERROR: Mismatched thresholds" << endl;
                    return -1;
               }

               TPR_sum += it2->TPR;
               FPR_sum += it2->FPR;               
          }
          double TPR_avg = TPR_sum / ((double)it1->second.size());
          double FPR_avg = FPR_sum / ((double)it1->second.size());
          
          double TPR_sum_var = 0;
          double FPR_sum_var = 0;
          for (std::vector<struct ROCData>::iterator it2 = it1->second.begin();
               it2 != it1->second.end(); it2++) {
               if (it2->threshold != syllo::str2double(it1->first)) {
                    cout << "ERROR: Mismatched thresholds" << endl;
                    return -1;
               }

               TPR_sum_var += pow(it2->TPR - TPR_avg,2);
               FPR_sum_var += pow(it2->FPR - FPR_avg,2);               
          }
          int count = it1->second.size()-1;
          double TPR_std = sqrt(TPR_sum_var / (double)count);
          double FPR_std = sqrt(FPR_sum_var / (double)count);
          
          struct ROCData d;
          d.TPR = TPR_avg;
          d.FPR = FPR_avg;
          d.threshold = syllo::str2double(it1->first);
          
          double z = 1.96; // 95% confidence interval          
          //d.TPR_error_low = TPR_avg - z*sqrt(TPR_avg*(1-TPR_avg)/count);
          //d.TPR_error_high = TPR_avg + z*sqrt(TPR_avg*(1-TPR_avg)/count);
          //d.FPR_error_low = FPR_avg - z*sqrt(FPR_avg*(1-FPR_avg)/count);
          //d.FPR_error_high = FPR_avg + z*sqrt(FPR_avg*(1-FPR_avg)/count);
          d.TPR_error_low = TPR_avg - z * TPR_std / sqrt(count);
          d.TPR_error_high = TPR_avg + z * TPR_std / sqrt(count);
          d.FPR_error_low = FPR_avg - z * FPR_std / sqrt(count);
          d.FPR_error_high = FPR_avg + z * FPR_std / sqrt(count);
          
          final.push_back(d);
     }

     // sort final vector by TPR/FPR
     std::sort(final.begin(), final.end(), myfunction);
     
     // Print out the results
     std::string roc_fn = output_dir + "/avg-roc.csv";
     std::ofstream roc_stream;
     cout << "ROC Output file: " << roc_fn << endl;
     roc_stream.open (roc_fn.c_str(), std::ofstream::out);
     roc_stream << "# FPR,TPR,FPR_error_low,FPR_error_high,TPR_error_low,TPR_error_high,thresh" << endl;
          
     // Save the average ROC plot to a text file for gnuplot. Also, create a
     // metrics_vector to be used by ROC.h to find the operating point.
     std::vector< std::map<std::string,double> > metrics_vector;     
     for(std::vector< struct ROCData >::iterator it = final.begin(); 
         it != final.end(); it++) {
          roc_stream << it->FPR << "," << it->TPR << ","
                     << it->FPR_error_low << "," << it->FPR_error_high << "," 
                     << it->TPR_error_low << "," << it->TPR_error_high << "," 
                     << it->threshold << endl;

          std::map<std::string,double> metrics;
          metrics["PRE_FPR"] = it->FPR;
          metrics["PRE_TPR"] = it->TPR;
          metrics["thresh_value"] = it->threshold;
          metrics_vector.push_back(metrics);
     }
     roc_stream.close();

     std::vector< std::map<std::string,double> >::iterator it_oppt;
     ROC::OperatingPoint(metrics_vector, it_oppt);

     // Write out the threshold to a yaml param file for the final test set
     YAML::Emitter out;
     out << YAML::BeginMap;
     out << YAML::Key << param_sweep;
     out << YAML::Value << syllo::double2str((*it_oppt)["thresh_value"]);
     out << YAML::Key << "threshold_type";
     out << YAML::Value << threshold_type;
     out << YAML::EndMap;

     std::string out_filename = output_dir + "/test.yaml";
     std::ofstream output;
     output.open(out_filename.c_str(), std::ofstream::out);
     output << out.c_str();
     output.close();
     
     return 0;
}
