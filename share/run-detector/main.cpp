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
#include <opencv_workbench/wb/Parameters.h>

#include <yaml-cpp/yaml.h>

using std::cout;
using std::endl;

PluginManager<Detector, Detector_maker_t> plugin_manager_;

int main(int argc, char *argv[])
{    
     //int aflag = 0;
     //int bflag = 0;
     //char *cvalue = NULL;
     //int index;
     int hide_window_flag = 0;
     int step_flag = 0;
     int c;
     //std::string video_filename = "/home/syllogismrxs/Documents/Thesis/data/sonar-avi/2014-01-24-ME-sonar-only-diver/2014_01_24_15_24_06.avi";
     std::string video_filename = "/home/syllogismrxs/Documents/Thesis/data/NEEMO/neemo-sonar/2015_07_29-Walking-Habitat/2015_07_29_13_28_17-hab-diver-fish.son";
     std::string plugin_name = "relative_detector";
     std::string xml_output_dir = "";
     std::string yaml_file = "";     
     std::string k_folds_file = "";
     int xml_output_dir_flag = 0;
     int threshold_sweep_flag = 0;
  
     while ((c = getopt (argc, argv, "k:tf:hp:o:sy:")) != -1) {
          switch (c) {               
          case 'k':
               k_folds_file = std::string(optarg);
               break;
          case 'o':
               xml_output_dir_flag = 1;
               xml_output_dir = std::string(optarg);
               break;
          case 't':
               threshold_sweep_flag = 1;
               break;
          case 'h':
               hide_window_flag = 1;
               break;
          case 'p':
               plugin_name = std::string(optarg);
               break;
          case 'f':
               video_filename = std::string(optarg);
               break;
          case 'y':
               yaml_file = std::string(optarg);
               break;
          case 's':
               step_flag = 1;
               break;
          case '?':
               if (optopt == 'c') {
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
               } else if (optopt == 'f') {
                    fprintf (stderr, "Option -%c requires a video filename as an argument.\n", optopt);
               } else if (optopt == 'o') {
                    fprintf (stderr, "Option -%c requires an output directory as an argument.\n", optopt);
               } else if (optopt == 'p') {
                    fprintf (stderr, "Option -%c requires a plugin name as an argument.\n", optopt);
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
          
     syllo::fill_line("=");
     cout << "Running Detector" << endl;
     syllo::fill_line("=");
     
     syllo::Stream stream;
     syllo::Status status = stream.open(video_filename);
     //stream.set_sonar_data_mode(syllo::image);
     //stream.set_sonar_data_mode(syllo::range);
     
     if (status != syllo::Success) {
          cout << "Failed to open: " << video_filename << endl;
          return -1;
     }

     // Setup Hand Annotated Parser (Truth)
     bool hand_ann_found = true;
     AnnotationParser parser_truth;
     int retcode = parser_truth.CheckForFile(video_filename, AnnotationParser::hand);
     if (retcode != 0) {
          cout << "Error parsing hand annotated file." << endl;
          hand_ann_found = false;          
     }

     Parameters params;
     params.set_yaml_file(yaml_file);
     
     // Setup Annotation Parser_Tracks
     AnnotationParser parser_tracks;
     parser_tracks.CheckForFile(video_filename, AnnotationParser::track);
     parser_tracks.clear();
     parser_tracks.set_width(stream.width());
     parser_tracks.set_height(stream.height());     
     parser_tracks.set_depth(3);
     parser_tracks.set_type("video");
     parser_tracks.set_number_of_frames(stream.get_frame_count());
     parser_tracks.set_plugin_name(plugin_name);
     parser_tracks.set_params(params);
     
     if (xml_output_dir_flag == 1) {
          parser_tracks.set_xml_output_dir(xml_output_dir);
     }

     // If we are doing a threshold sweep, the output file should match the
     // the input yaml file name.
     if (threshold_sweep_flag == 1 && yaml_file != "") {          
          parser_tracks.set_xml_output_filename(yaml_file);
     }

     // Load the Bridge shared library (based on yml file)
     retcode = plugin_manager_.search_for_plugins("OPENCV_WORKBENCH_PLUGIN_PATH");
     if (retcode != 0) {
          cout << "Failed to find plugins." << endl;
          return -1;
     }

     retcode = plugin_manager_.open_library(plugin_name);
     if (retcode != 0) {
          cout << "Unable to open library: " << plugin_name << endl;
          
          return -1;
     } else {
          cout << "Using Bridge Library: " << plugin_name << endl;
     }     

     // Parse the k-folds file, if it exists
     std::map<unsigned int,Frame> frame_types;
     if (k_folds_file != "") {
          std::ifstream fin(k_folds_file.c_str());
          YAML::Parser parser(fin);
          YAML::Node doc;
          parser.GetNextDocument(doc);

          //TODO HERE
     }
               
     Detector * detector_;     
     detector_ = plugin_manager_.object();
     detector_->set_params(params);
     detector_->set_stream(&stream);
     detector_->print();       
     
     if (hide_window_flag) {
          detector_->hide_windows(true);
     } else {
          detector_->hide_windows(false);
     }
     
     cv::Mat original;
     int frame_number = 0;
     while (stream.read(original)) {
          
          // Is this frame part of the final test set?
          // Is this frame part of the validation set?
          
          // Pass the frame to the detector plugin
          detector_->set_frame(frame_number, original);

          // Get track list from detector
          std::vector<wb::Entity> tracks = detector_->tracks();
          
          if (hand_ann_found) {
               // Score preprocessing, if available
               cv::Mat thresh_img = detector_->thresh_img();
               parser_tracks.score_preprocessing(frame_number, parser_truth, 
                                                 thresh_img);

               std::vector<wb::Entity> frame_ents;
               detector_->frame_ents(frame_ents);
          }
          
          // Put all track data in parser for saving and
          // Draw estimated diver locations on original image
          Frame frame;
          frame.set_frame_number(frame_number);
          std::vector<wb::Entity>::iterator it = tracks.begin();
          for (; it != tracks.end(); it++) {
               cv::Point point = it->centroid();
               //cv::Point point = it->estimated_centroid();
               
               frame.objects[it->name()] = *it;

               if (it->type() == wb::Entity::Diver) {
                    // If this is a diver type, mark it on the original image
                    int radius = 3;
                    cv::circle(original, cv::Point(point.x,point.y), 
                               radius, cv::Scalar(0,0,0), 2, 8, 0);
               }
          }     
          // Save frame to parser
          parser_tracks.frames[frame_number] = frame;

          if (!hide_window_flag) { 
               cv::imshow("Detection", original);
               
               if (step_flag) {
                    int key = cv::waitKey(0);    
                    if (key == 'q' || key == 1048689) { // 'q' key
                         cout << "Ending early." << endl;
                         break;
                    } else if (key == 'p' || key == 1048688) {
                         step_flag = 0;
                    }     
               } else {
                    int key = cv::waitKey(1);               
                    if (key == 'q' || key == 1048689) {
                         cout << "Ending early." << endl;
                         break;
                    } else if (key == 'p' || key == 1048688) {
                         step_flag = 1;
                    } 
               }

          }                    
          frame_number++;
     }
     
     // Which track exhibited the most displacement?
     // Which track is the oldest?               
     
     // We can only compare the detector to truth data if we have a hand
     // annotated file.
     if (hand_ann_found) {
          cout << "Scoring detector..." << endl;     
          std::vector<std::string> names;
          //names.push_back("diver:1");
          names.push_back("diver");
          //parser_tracks.score_detector(parser_truth, names);          
          parser_tracks.score_detector_2(parser_truth, names);          
          parser_tracks.score_preprocessing_final(parser_truth);
     } else {
          cout << "WARNING: Can't score detector because hand annotated "
               << "file is missing" << endl;
     }

     cout << "Saving tracks to xml file" << endl;
     parser_tracks.write_annotation();
     
     plugin_manager_.close_libraries();

     cout << "Done Processing." << endl;
     return 0;
}
