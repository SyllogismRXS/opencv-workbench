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
     //ratio_threshold = 0.003;
     ratio_threshold = 0.00162001;
     //static_threshold = 150;
     //static_threshold = 50;
     static_threshold = 150;
     gradient_threshold = 150;
     //threshold_type = static_type;
     threshold_type = ratio_type;
     //threshold_type = gradient_type;
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

          if(const YAML::Node *p = doc.FindValue("gradient_threshold")) {
               *p >> gradient_threshold;
          }

          if(const YAML::Node *p = doc.FindValue("history_length")) {
               *p >> history_length;
          }
          
          if(const YAML::Node *p = doc.FindValue("history_distance")) {
               *p >> history_distance;
          }

          if(const YAML::Node *p = doc.FindValue("threshold_type")) {
               int type;
               *p >> type;

               switch ((ThresholdType_t)type) {
               case static_type:
                    threshold_type = static_type;
                    break;
               case ratio_type:
                    threshold_type = ratio_type;
                    break;
               case gradient_type:
                    threshold_type = gradient_type;
                    break;
               default:                    
                    cout << "ERROR: Invalid threshold_type: " << type << endl;
                    break;
               }
          }
     }
}
