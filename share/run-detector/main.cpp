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

using std::cout;
using std::endl;

PluginManager<Detector, Detector_maker_t> plugin_manager_;

int main(int argc, char *argv[])
{
     cout << "Running Detector" << endl;
     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input-file>" << endl;
          return -1;
     }

     syllo::Stream stream;
     syllo::Status status = stream.open(argv[1]);

     if (status != syllo::Success) {
          cout << "Failed to open: " << argv[1] << endl;
          return -1;
     }

     // Setup Annotation Parser_Tracks
     AnnotationParser parser_tracks;
     parser_tracks.CheckForFile(argv[1], AnnotationParser::track);
     parser_tracks.clear();
     parser_tracks.set_width(stream.width());
     parser_tracks.set_height(stream.height());
     parser_tracks.set_depth(3);
     parser_tracks.set_type("video");
     parser_tracks.set_number_of_frames(stream.get_frame_count());

     // Load the Bridge shared library (based on yml file)
     int retcode = plugin_manager_.search_for_plugins("OPENCV_WORKBENCH_PLUGIN_PATH");
     if (retcode != 0) {
          cout << "Failed to find plugins." << endl;
          return -1;
     }

     std::string plugin_name = "displace_detector";
     retcode = plugin_manager_.open_library(plugin_name);
     if (retcode != 0) {
          cout << "Unable to open library: " << plugin_name << endl;
          
          return -1;
     } else {
          cout << "Using Bridge Library: " << plugin_name << endl;
     }
     
     Detector * detector_;     
     detector_ = plugin_manager_.object();
     detector_->print();          
     
     cv::Mat original;
     int frame_number = 0;
     while (stream.read(original)) {
          
          // Pass the frame to the detector plugin
          detector_->set_frame(frame_number, original);

          // Get track list from detector
          std::vector<syllo::Track> tracks = detector_->tracks();
          
          // Put all track data in parser for saving and
          // Draw estimated diver locations on original image
          Frame frame;
          frame.set_frame_number(frame_number);
          std::vector<syllo::Track>::iterator it = tracks.begin();
          for (; it != tracks.end(); it++) {
               cv::Point3d point3d = it->position();

               Object object;                                             
               object.set_id(it->id());
               object.set_name(it->name());
                              
               object.bbox = BoundingBox(point3d.x, 
                                         point3d.x,
                                         point3d.y, 
                                         point3d.y);
               object.set_age(it->age());
               
               // Save object to current frame
               frame.objects[object.name()] = object;                          

               if (it->type() == syllo::Diver) {
                    // If this is a diver type, mark it on the original image                    
                    int radius = 3;
                    cv::circle(original, cv::Point(point3d.x,point3d.y), 
                               radius, cv::Scalar(0,0,0), 2, 8, 0);
               }
          }     
          // Save frame to parser
          parser_tracks.frames[frame_number] = frame;

          cv::imshow("Detection", original);
          
          if (cv::waitKey(10) >= 0) {
               cout << "Ending early." << endl;
               break;
          }
          frame_number++;
     }          

     //// Setup Annotation Parser
     //AnnotationParser parser;
     //int status = parser.CheckForFile(argv[1], AnnotationParser::track);
     //if (status != 0) {
     //     cout << "Error parsing tracks file." << endl;
     //     return -1;
     //}     
     
     // Which track exhibited the most displacement?
     
     
     // Which track is the oldest?
  
//plugin_manager_.close_libraries();


     cout << "Saving tracks to xml file" << endl;
     parser_tracks.write_annotation();

     cout << "Done Processing." << endl;
     return 0;
}
