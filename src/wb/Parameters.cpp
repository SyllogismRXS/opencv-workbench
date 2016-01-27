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
}

void Parameters::set_yaml_file(std::string yaml_file)
{
     yaml_file_ = yaml_file;
     
     if (yaml_file_ != "") {
          std::ifstream fin(yaml_file_.c_str());
          YAML::Parser parser(fin);     
          YAML::Node doc;
          parser.GetNextDocument(doc);
          
          if(const YAML::Node *p = doc.FindValue("history_length")) {
               *p >> history_length;
          }
          
          if(const YAML::Node *p = doc.FindValue("history_distance")) {
               *p >> history_distance;
          }
     }
}
