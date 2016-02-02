#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "Parameters.h"

using std::cout;
using std::endl;

Parameters::Parameters()
{
     history_length = 10;
     history_distance = 15;
     ratio_threshold = 0.003;
     //static_threshold = 150;
     //static_threshold = 50;
     static_threshold = 150;

     threshold_type = static_type;
}

void Parameters::set_yaml_file(std::string yaml_file)
{
     yaml_file_ = yaml_file;
     
     if (yaml_file_ != "") {
          std::ifstream fin(yaml_file_.c_str());
          YAML::Parser parser(fin);     
          YAML::Node doc;
          parser.GetNextDocument(doc);
          
          if(const YAML::Node *p = doc.FindValue("ratio_threshold")) {
               *p >> ratio_threshold;
          }

          if(const YAML::Node *p = doc.FindValue("static_threshold")) {
               *p >> static_threshold;
          }

          if(const YAML::Node *p = doc.FindValue("history_length")) {
               *p >> history_length;
          }
          
          if(const YAML::Node *p = doc.FindValue("history_distance")) {
               *p >> history_distance;
          }

          if(const YAML::Node *p = doc.FindValue("threshold_type")) {
               std::string str;
               *p >> str;
               if (str == "static") {
                    threshold_type = static_type;
               } else if (str == "ratio") {
                    threshold_type = ratio_type;
               } else if (str == "gradient") {
                    threshold_type = gradient_type;
               } else {
                    cout << "ERROR: Invalid threshold_type: " << str << endl;
               }
          }
     }
}
