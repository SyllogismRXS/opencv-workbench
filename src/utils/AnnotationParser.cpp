#include <iostream>
#include <fstream>
#include <string>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

#include <opencv_workbench/syllo/syllo.h>

#include <opencv_workbench/rapidxml/rapidxml.hpp>
#include <opencv_workbench/rapidxml/rapidxml_utils.hpp>

#include "AnnotationParser.h"

using std::cout;
using std::endl;

using namespace rapidxml;
namespace fs = boost::filesystem;

AnnotationParser::AnnotationParser()
{
     
}

void AnnotationParser::CheckForFile(std::string video_file)
{
     int lastindex = video_file.find_last_of("."); 
     xml_filename_ = video_file.substr(0, lastindex) + ".xml";
     
     //fs::path dir = fs::path(video_file).parent_path();

     if ( !boost::filesystem::exists( xml_filename_ ) ) {
          cout << "Annotation file doesn't exist. Creating..." << endl;
          std::ofstream outfile;
          outfile.open(xml_filename_.c_str());          
          outfile.close();
     } else {
          cout << "Found annotation file." << endl;
     }
}

int AnnotationParser::ParseFile(std::string file)
{
     xml_filename_ = file;

     rapidxml::file<> xmlFile(file.c_str());
     rapidxml::xml_document<> doc;
     doc.parse<0>(xmlFile.data());

     // First node should be named <annotation>
     if (strcmp(doc.first_node()->name(), "annotation") != 0) {
          cout << "Invalid xml file: annotation" << endl;
          return -1;
     }

     // Find <frames>
     xml_node<> * frames_node = doc.first_node()->first_node("frames");
     if (frames_node == 0) {
          return -1;          
     }
     
     // Loop through all frames
     xml_node<> *frame_node = frames_node->first_node("frame");
     do {
          if (frame_node == 0) {
               cout << "Missing frame node" << endl;
               return -1;
          }
          
          xml_node<> *frame_number_node = frame_node->first_node("frame_number");
          if (frame_number_node == 0) {               
               cout << "Missing frame number" << endl;
               return -1;
          }
                    
          int frame_number = syllo::str2int(frame_number_node->value());
                    
          Frame frame;
          frame.set_frame_number(frame_number);
          
          // Loop through objects in this frame.
          xml_node<> *object_node = frame_node->first_node("object");
          do {
               if (object_node == 0) {
                    cout << "Missing an object node" << endl;
               }               
               
               std::string object_name = object_node->first_node("name")->value();
               
               Object object;
               object.set_name(object_name);
               
               xml_node<> *box = object_node->first_node("bndbox");
               if (box == 0) {
                    cout << "Missing bounding box" << endl;
                    return -1;
               }
               
               int xmin, ymin, xmax, ymax;               
               xmin = syllo::str2int(box->first_node("xmin")->value());
               ymin = syllo::str2int(box->first_node("ymin")->value());
               xmax = syllo::str2int(box->first_node("xmax")->value());
               ymax = syllo::str2int(box->first_node("ymax")->value());

               object.bbox = BoundingBox(xmin,xmax,ymin,ymax);
               
               frame.objects[object_name] = object;
               
          }while ((object_node = object_node->next_sibling()) != 0 );
          
          frames_[frame_number] = frame;
          
     }while ((frame_node = frame_node->next_sibling()) != 0 );
     
     return 0;
}

void AnnotationParser::print()
{
     std::map<int,Frame>::iterator frame_it = frames_.begin();
     for(; frame_it != frames_.end(); frame_it++) {
          cout << "-----------------------------" << endl;
          cout << "Frame Number: " << frame_it->second.frame_number() << endl; 
          cout << "---------" << endl;
          cout << "Objects: " << endl;
          
          std::map<std::string, Object>::iterator object_it;
          object_it = frame_it->second.objects.begin();
          for (; object_it != frame_it->second.objects.end(); object_it++) {
               cout << "\t Name: " << object_it->second.name() << endl;
               cout << "\t Bbox: (" << object_it->second.bbox.xmin() << ","
                    << object_it->second.bbox.ymin() << ","
                    << object_it->second.bbox.xmax() << ","
                    << object_it->second.bbox.ymax() << ")" << endl << endl;
          }
     }
}
