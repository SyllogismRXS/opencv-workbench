#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

#include <boost/filesystem.hpp>

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

#include <yaml-cpp/yaml.h>

using std::cout;
using std::endl;

PluginManager<Detector, Detector_maker_t> plugin_manager_;

// our data types
struct Range3_Yaml {
     double start, step, stop;
};

void operator >> (const YAML::Node& node, Range3_Yaml& r)
{
     node[0] >> r.start;
     node[1] >> r.step;
     node[2] >> r.stop;
}


int main(int argc, char *argv[])
{     
     int c;
     std::string xml_output_dir = "";
     std::string yaml_file = "";
     int xml_output_dir_flag = 0;
     bool print_yaml = false;

     while ((c = getopt (argc, argv, "po:y:")) != -1) {
          switch (c) {               
          case 'o':
               xml_output_dir_flag = 1;
               xml_output_dir = std::string(optarg);
               break;          
          case 'y':
               yaml_file = std::string(optarg);
               break;          
          case 'p':
               print_yaml = true;
               break;
          case '?':
               if (optopt == 'c') {
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);               
               } else if (optopt == 'y') {
                    fprintf (stderr, "Option -%c requires a yaml file name as an argument.\n", optopt);
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

     if (yaml_file == "") {
          cout << "Missing Input Yaml File" << endl;
          return -1;
     }

     if (xml_output_dir_flag == 0) {
          cout << "Output directory required" << endl;
          return -1;
     }

     if ( !fs::exists( fs::path(xml_output_dir) ) ) {
          cout << "Output directory doesn't exist" << endl;
          return -1;
     }     

     std::ifstream fin(yaml_file.c_str());
     YAML::Parser parser(fin);
     YAML::Node doc;
     parser.GetNextDocument(doc);

     // Parse the YAML File.
     // Store lists of the ranges
     std::map<std::string, std::list<double> > params;     
     for(YAML::Iterator it = doc.begin(); it!=doc.end(); ++it) {
          std::string key;
          double value;
          std::string str_value;
          it.first() >> key;

          std::list<double> nums;
          if (it.second().Type() == YAML::NodeType::Scalar) {
               // Simple static parameter               
               it.second() >> value;
               nums.push_back(value);          
          } else {
               // A range of parameters
               Range3_Yaml range;
               it.second() >> range;
               for (double i = range.start; i <= range.stop; i += range.step) {
                    nums.push_back(i);
               }
          }
          params[key] = nums;
     }

     // Determine the total number of permutations required
     int permutations = 1;
     for (std::map<std::string, std::list<double> >::iterator it1 = params.begin();
          it1 != params.end(); it1++) {
          permutations *= it1->second.size();
     }

     if (print_yaml) {
          cout << "Permutations: " << permutations << endl;
     }

     // Determine the number of permutations required for each param, before
     // the next value for that parameter has to be acquired
     std::map<std::string, int> original_param_perms;

     // Keep a map of the original iterators that point to the lists
     //std::map<std::string, std::list<double>::iterator > iterators;
     std::map<std::string, std::list<double>::iterator > original_iterators;

     // Loop through all of the parameters.
     for (std::map<std::string, std::list<double> >::iterator it1 = params.begin();
          it1 != params.end(); it1++) {

          int perms = 1;

          // For the remaining params in the map, determine the number of times
          // the current parameter has to be repeated to accomodate all
          // permutations of the "later" parameters. We can just multiply the
          // sizes of the "later" lists.
          for (std::map<std::string, std::list<double> >::iterator it2 = it1;
               it2 != params.end(); it2++) {
               if (it1 == it2) {
                    // Don't count when it's the same parameter.
                    continue;
               }
               perms *= it2->second.size();
          }

          original_param_perms[it1->first] = perms;
          original_iterators[it1->first] = it1->second.begin();
     }

     if (print_yaml) {
          for (std::map<std::string, int>::iterator it = original_param_perms.begin();
               it != original_param_perms.end(); it++) {
               cout << it->first << " : " << it->second << endl;
          }
          cout << endl << endl;
     }

     // Need temporary values of the original_param_perms and iterators for
     // when we need to reset values.
     std::map<std::string, int> curr_param_perms = original_param_perms;
     std::map<std::string, std::list<double>::iterator> curr_iterators = original_iterators;

     // Loop for the max number of permutations.
     for (int i = 0; i < permutations; i++) {                    
          YAML::Emitter out;
          out << YAML::BeginMap;

          for (std::map<std::string, std::list<double>::iterator >::iterator it = curr_iterators.begin();
               it != curr_iterators.end(); it++) {

               out << YAML::Key << it->first << YAML::Value << *(it->second);

               //Decrement count
               curr_param_perms[it->first] = curr_param_perms[it->first]-1;

               if (curr_param_perms[it->first] == 0) {

                    // Reset curr_param_perms count;
                    curr_param_perms[it->first] = original_param_perms[it->first];

                    // Go to the next iterator for this parameter
                    (it->second)++;

                    // If this iterator is at the end of this list for this
                    // parameter, reset it!
                    if (it->second == params[it->first].end()) {
                         // Reset the iterator
                         it->second = original_iterators[it->first];
                    }
               }
          }

          out << YAML::EndMap;

          std::string filename = xml_output_dir + "/" + syllo::int2str(i) + "-expanded.yaml";

          if (print_yaml) {
               cout << "-------------------------------------" << endl;
               cout << "  Output file:  " << filename << endl;
               cout << "--------------------------------------" << endl;
               cout << out.c_str() << endl;               
          }                    
          
          std::ofstream output;
          output.open(filename.c_str(), std::ofstream::out);
          output << out.c_str();
          output.close();          
     }     
     return 0;
}
