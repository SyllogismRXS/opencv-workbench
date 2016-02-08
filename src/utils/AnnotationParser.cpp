#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#include <stdlib.h>
#include <time.h> 
#include <stdio.h>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
//#include <boost/random.hpp>
//#include <boost/random/normal_distribution.hpp>
//#include <boost/random/uniform_int.hpp>

#include <opencv_workbench/syllo/syllo.h>

#include <opencv_workbench/rapidxml/rapidxml.hpp>
#include <opencv_workbench/rapidxml/rapidxml_utils.hpp>
#include <opencv_workbench/rapidxml/rapidxml_print.hpp>

#include <opencv_workbench/plot/Plot.h>

#include "AnnotationParser.h"

using std::cout;
using std::endl;

using namespace rapidxml;

AnnotationParser::AnnotationParser()
{
     srand (time(NULL));
     this->reset();
}

void AnnotationParser::reset()
{
     video_filename_ = "";
     xml_filename_ = "";
     dir_ = "";
     basename_ = "";
     metrics_present_ = false;         
     neg_to_pos_ratio_ = 1;
     
     reset_metrics();
     frames.clear();
}

void AnnotationParser::clear()
{
     reset_metrics();
     frames.clear();
}

bool AnnotationParser::find_file( const fs::path & dir_path,      // in this directory,
                                  const std::string & file_name, // search for this name,
                                  fs::path & path_found )            // placing path here if found
{
     if ( !fs::exists( dir_path ) ) return false;
     fs::directory_iterator end_itr; // default construction yields past-the-end
     for ( fs::directory_iterator itr( dir_path );
           itr != end_itr;
           ++itr )
     {
          if ( fs::is_directory(itr->status()) )
          {
               if ( find_file( itr->path(), file_name, path_found ) ) return true;
          }
          else if ( itr->path().filename() == file_name ) // see below
          {
               path_found = itr->path();
               return true;
          }
     }
     return false;
}

// Look for .tracks.xml or .truth.xml file in the same directory as the .avi or
// .son file first. If it's not there, look in the environment variable path
// recursively.
int AnnotationParser::CheckForFile(std::string video_file, 
                                    AnnotationParser::AnnotateType_t ann_type)
{
     video_filename_ = video_file;
     video_file_stem_ = fs::path(video_filename_).stem().string();

     ann_type_ = ann_type;
     
     // Remove the .son or .avi from the end of the filename
     int lastindex = video_file.find_last_of("."); 
     xml_filename_ = video_file.substr(0, lastindex);          
     
     if (ann_type_ == hand) {
          xml_filename_ += ".truth.xml";
     } else if (ann_type_ == track) {
          xml_filename_ += ".tracks.xml";
     } else {
          cout << "ERROR: Invalid annotation type" << endl;
     }         
     
     dir_ = fs::path(video_file).parent_path();
     basename_ = fs::path(video_file).filename();
         
     // Search for the .truth.xml or .tracks.xml filename
     if ( !fs::exists( xml_filename_ ) ) {

          // Search for the hand annotated file (truth) using the 
          // OPENCV_WORKBENCH_ANNOTATED_FILES environment variable

          if (ann_type_ == hand) {
               // Build the filename to search for.
               std::string file = fs::path(video_file).stem().string();
               file += ".truth.xml";
                              
               // See if the env var exists
               char* pPath = std::getenv("OPENCV_WORKBENCH_ANNOTATED_FILES");
               if (pPath != NULL) {                              
                    fs::path result_path;
                    fs::path search_path = std::string(pPath);

                    //cout << "Searching for file: " << file << ", under: " << search_path.string() << endl;
                    
                    bool path_found = this->find_file(search_path, file, result_path);

                    if (path_found) {
                         xml_filename_ = result_path.string();
                         cout << "Found file at: " << xml_filename_ << endl;
                         return this->ParseFile(xml_filename_);
                    } else {
                         //cout << "File not found, recursively." << endl;
                    }
               } else {
                    cout << "Env Var Not Set: OPENCV_WORKBENCH_ANNOTATED_FILES" << endl;
               }
          }

          if (ann_type_ == hand) {
               cout << "Hand ";
          } else if (ann_type_ == track) {
               cout << "Track ";
          }
          cout << "annotation file doesn't exist yet." << endl;
          return 1;
     } else {
          if (ann_type_ == hand) {
               //cout << "Hand ";
          } else if (ann_type_ == track) {
               //cout << "Track ";
          }
          //cout << "annotation file found." << endl;
          return this->ParseFile(xml_filename_);
     }
}

void AnnotationParser::set_xml_output_dir(std::string dir)
{
     // Check existence of output directory
     if ( !boost::filesystem::exists( dir ) ) {
          // Create it if it doesn't exist
          //if(!fs::create_directory(fs::path(dir))) {
          if(!fs::create_directories(fs::path(dir))) {
               cout << "ERROR: Unable to create output directory: "
                    << dir << endl;
               return;
          }
     }
     
     fs::path filename = fs::path(xml_filename_).filename();
     xml_filename_ = dir + "/" + filename.c_str();
}

void AnnotationParser::prepend_xml_output_filename(std::string yaml_file)
{          
     std::string yaml_stem = fs::path(yaml_file).stem().string();
     fs::path dir = fs::path(xml_filename_).parent_path();
     xml_filename_ = dir.string() + "/" + yaml_stem + "_" + video_file_stem_ + ".tracks.xml";
}

void AnnotationParser::write_annotation()
{
     this->write_header();
}

void AnnotationParser::write_header()
{     
     xml_document<> doc;
     xml_node<> *root_node = doc.allocate_node(node_element, "annotation");
     doc.append_node(root_node);

     xml_node<> *folder_node = doc.allocate_node(node_element, "folder", dir_.c_str());
     root_node->append_node(folder_node);
     
     xml_node<> *filename_node = doc.allocate_node(node_element, "filename", basename_.c_str());
     root_node->append_node(filename_node);

     xml_node<> *type_node = doc.allocate_node(node_element, "type", "video");
     root_node->append_node(type_node);

     if (ann_type_ == track) {
          xml_node<> *plugin_name_node = doc.allocate_node(node_element, "plugin_name", plugin_name_.c_str());
          root_node->append_node(plugin_name_node);

          if (metrics_present_) {
               xml_node<> *metrics_node = doc.allocate_node(node_element, "metrics");
               root_node->append_node(metrics_node);
               
               // "Pre"processing metrics               
               xml_node<> *pre_node = doc.allocate_node(node_element, "pre");
               metrics_node->append_node(pre_node);

               char * PRE_TP = doc.allocate_string(syllo::int2str(PRE_TP_).c_str());
               xml_node<> *PRE_TP_node = doc.allocate_node(node_element, "PRE_TP", PRE_TP);
               pre_node->append_node(PRE_TP_node);

               char * PRE_TN = doc.allocate_string(syllo::int2str(PRE_TN_).c_str());
               xml_node<> *PRE_TN_node = doc.allocate_node(node_element, "PRE_TN", PRE_TN);
               pre_node->append_node(PRE_TN_node);

               char * PRE_FP = doc.allocate_string(syllo::int2str(PRE_FP_).c_str());
               xml_node<> *PRE_FP_node = doc.allocate_node(node_element, "PRE_FP", PRE_FP);
               pre_node->append_node(PRE_FP_node);

               char * PRE_FN = doc.allocate_string(syllo::int2str(PRE_FN_).c_str());
               xml_node<> *PRE_FN_node = doc.allocate_node(node_element, "PRE_FN", PRE_FN);
               pre_node->append_node(PRE_FN_node);               

               char * PRE_TPR = doc.allocate_string(syllo::double2str(PRE_TPR_).c_str());
               xml_node<> *PRE_TPR_node = doc.allocate_node(node_element, "PRE_TPR", PRE_TPR);
               pre_node->append_node(PRE_TPR_node);               

               char * PRE_FPR = doc.allocate_string(syllo::double2str(PRE_FPR_).c_str());
               xml_node<> *PRE_FPR_node = doc.allocate_node(node_element, "PRE_FPR", PRE_FPR);
               pre_node->append_node(PRE_FPR_node);               

               char * NEG_SAMPLES = doc.allocate_string(syllo::int2str(negative_sample_count_).c_str());
               xml_node<> *NEG_SAMPLES_node = doc.allocate_node(node_element, "NEG_SAMPLES", NEG_SAMPLES);
               pre_node->append_node(NEG_SAMPLES_node);               

               char * PRE_Accuracy = doc.allocate_string(syllo::double2str(PRE_Accuracy_).c_str());
               xml_node<> *PRE_Accuracy_node = doc.allocate_node(node_element, "PRE_Accuracy", PRE_Accuracy);
               pre_node->append_node(PRE_Accuracy_node);               
               
               char * Pd = doc.allocate_string(syllo::double2str(Pd_).c_str());
               xml_node<> *Pd_node = doc.allocate_node(node_element, "Pd", Pd);
               pre_node->append_node(Pd_node);

               char * Pfa = doc.allocate_string(syllo::double2str(Pfa_).c_str());
               xml_node<> *Pfa_node = doc.allocate_node(node_element, "Pfa", Pfa);
               pre_node->append_node(Pfa_node);               
               
               // For the higher level metrics...
               char * TP = doc.allocate_string(syllo::int2str(TP_).c_str());
               xml_node<> *TP_node = doc.allocate_node(node_element, "TP", TP);
               metrics_node->append_node(TP_node);

               char * TN = doc.allocate_string(syllo::int2str(TN_).c_str());
               xml_node<> *TN_node = doc.allocate_node(node_element, "TN", TN);
               metrics_node->append_node(TN_node);

               char * FP = doc.allocate_string(syllo::int2str(FP_).c_str());
               xml_node<> *FP_node = doc.allocate_node(node_element, "FP", FP);
               metrics_node->append_node(FP_node);

               char * FN = doc.allocate_string(syllo::int2str(FN_).c_str());
               xml_node<> *FN_node = doc.allocate_node(node_element, "FN", FN);
               metrics_node->append_node(FN_node);

               char * TPR = doc.allocate_string(syllo::double2str(TPR_).c_str());
               xml_node<> *TPR_node = doc.allocate_node(node_element, "TPR", TPR);
               metrics_node->append_node(TPR_node);

               char * FPR = doc.allocate_string(syllo::double2str(FPR_).c_str());
               xml_node<> *FPR_node = doc.allocate_node(node_element, "FPR", FPR);
               metrics_node->append_node(FPR_node);               
          }

          // Write parameters for current run
          xml_node<> *parameters_node = doc.allocate_node(node_element, "parameters");
          root_node->append_node(parameters_node);

          char * ratio_threshold = doc.allocate_string(syllo::double2str(params_.ratio_threshold).c_str());
          xml_node<> *ratio_threshold_node = doc.allocate_node(node_element, "ratio_threshold", ratio_threshold);
          parameters_node->append_node(ratio_threshold_node);          

          char * gradient_threshold = doc.allocate_string(syllo::double2str(params_.gradient_threshold).c_str());
          xml_node<> *gradient_threshold_node = doc.allocate_node(node_element, "gradient_threshold", gradient_threshold);
          parameters_node->append_node(gradient_threshold_node);          
          
          char * static_threshold = doc.allocate_string(syllo::double2str(params_.static_threshold).c_str());
          xml_node<> *static_threshold_node = doc.allocate_node(node_element, "static_threshold", static_threshold);
          parameters_node->append_node(static_threshold_node);          
          
          char * history_length = doc.allocate_string(syllo::int2str(params_.history_length).c_str());
          xml_node<> *history_length_node = doc.allocate_node(node_element, "history_length", history_length);
          parameters_node->append_node(history_length_node);

          char * history_distance = doc.allocate_string(syllo::int2str(params_.history_distance).c_str());
          xml_node<> *history_distance_node = doc.allocate_node(node_element, "history_distance", history_distance);
          parameters_node->append_node(history_distance_node);          

          char * threshold_type = doc.allocate_string(syllo::int2str((int)(params_.threshold_type)).c_str());
          xml_node<> *threshold_type_node = doc.allocate_node(node_element, "threshold_type", threshold_type);
          parameters_node->append_node(threshold_type_node);          

     }

     xml_node<> *size_node = doc.allocate_node(node_element, "size");
     root_node->append_node(size_node);

     ///////////////////////////
     /// Width, height, depth
     ///////////////////////////
     std::string width = syllo::int2str(width_);
     xml_node<> *width_node = doc.allocate_node(node_element, "width", width.c_str());
     size_node->append_node(width_node);

     std::string height = syllo::int2str(height_);
     xml_node<> *height_node = doc.allocate_node(node_element, "height", height.c_str());
     size_node->append_node(height_node);

     std::string depth = syllo::int2str(depth_);
     xml_node<> *depth_node = doc.allocate_node(node_element, "depth", depth.c_str());
     size_node->append_node(depth_node);

     // Number of frames
     std::string number_of_frames = syllo::int2str(number_of_frames_);
     xml_node<> *num_frames_node = doc.allocate_node(node_element, "number_of_frames", number_of_frames.c_str());
     root_node->append_node(num_frames_node);

     // Number of Annotated frames
     std::string number_of_annotated_frames = syllo::int2str(frames.size());    
     xml_node<> *num_annotated_frames_node = doc.allocate_node(node_element, "number_of_annotated_frames", number_of_annotated_frames.c_str());
     root_node->append_node(num_annotated_frames_node);
     
     // Frames
     xml_node<> *frames_node = doc.allocate_node(node_element, "frames");
     root_node->append_node(frames_node);

     // Loop through each frame
     std::map<int,Frame>::iterator frame_it = frames.begin();
     for(; frame_it != frames.end(); frame_it++) {         
          
          // "frame"
          xml_node<> *frame_node = doc.allocate_node(node_element, "frame");
          frames_node->append_node(frame_node);

          // "frame_number"
          char * frame_number = doc.allocate_string(syllo::int2str(frame_it->second.frame_number()).c_str());
          xml_node<> *frame_number_node = doc.allocate_node(node_element, "frame_number", frame_number);
          frame_node->append_node(frame_number_node);         
          
          // Loop through each object          
          std::map<std::string, wb::Entity>::iterator object_it;
          object_it = frame_it->second.objects.begin();
          for (; object_it != frame_it->second.objects.end(); object_it++) {

               // "object" node
               xml_node<> *object_node = doc.allocate_node(node_element, "object");
               frame_node->append_node(object_node);
               
               // "Name" node               
               char * name = doc.allocate_string(object_it->second.name().c_str());
               xml_node<> *name_node = doc.allocate_node(node_element, "name", name);
               object_node->append_node(name_node);

               // "ID" node               
               char * id = doc.allocate_string(syllo::int2str(object_it->second.id()).c_str());
               xml_node<> *id_node = doc.allocate_node(node_element, "ID", id);
               object_node->append_node(id_node);

               //// "Type" node               
               //char * type = doc.allocate_string(object_it->second.type_str().c_str());
               //xml_node<> *type_node = doc.allocate_node(node_element, "type", type);
               //object_node->append_node(type_node);

               // "Age" node               
               char * age = doc.allocate_string(syllo::int2str(object_it->second.age()).c_str());
               xml_node<> *age_node = doc.allocate_node(node_element, "age", age);
               object_node->append_node(age_node);

               // "pose" node               
               char * pose = doc.allocate_string("Unspecified");
               xml_node<> *pose_node = doc.allocate_node(node_element, "pose", pose);
               object_node->append_node(pose_node);

               // "truncated" node               
               char * truncated = doc.allocate_string("0");
               xml_node<> *truncated_node = doc.allocate_node(node_element, "truncated", truncated);
               object_node->append_node(truncated_node);

               // "difficult" node               
               char * difficult = doc.allocate_string("0");
               xml_node<> *difficult_node = doc.allocate_node(node_element, "difficult", difficult);
               object_node->append_node(difficult_node);

               // "centroid" node
               xml_node<> *centroid_node = doc.allocate_node(node_element, "centroid");
               object_node->append_node(centroid_node);

               // "x" position node
               char * x = doc.allocate_string(syllo::double2str(object_it->second.undistorted_centroid().x).c_str());
               xml_node<> *x_node = doc.allocate_node(node_element, "x", x);
               centroid_node->append_node(x_node);

               // "y" position node
               char * y = doc.allocate_string(syllo::double2str(object_it->second.undistorted_centroid().y).c_str());
               xml_node<> *y_node = doc.allocate_node(node_element, "y", y);
               centroid_node->append_node(y_node);
               
               // "bndbox" node               
               xml_node<> *bndbox_node = doc.allocate_node(node_element, "bndbox");
               object_node->append_node(bndbox_node);
               
               // "xmin" node               
               char * xmin = doc.allocate_string(syllo::int2str(object_it->second.bbox().xmin()).c_str());
               xml_node<> *xmin_node = doc.allocate_node(node_element, "xmin", xmin);
               bndbox_node->append_node(xmin_node);

               // "ymin" node               
               char * ymin = doc.allocate_string(syllo::int2str(object_it->second.bbox().ymin()).c_str());
               xml_node<> *ymin_node = doc.allocate_node(node_element, "ymin", ymin);
               bndbox_node->append_node(ymin_node);

               // "xmax" node               
               char * xmax = doc.allocate_string(syllo::int2str(object_it->second.bbox().xmax()).c_str());
               xml_node<> *xmax_node = doc.allocate_node(node_element, "xmax", xmax);
               bndbox_node->append_node(xmax_node);

               // "ymax" node               
               char * ymax = doc.allocate_string(syllo::int2str(object_it->second.bbox().ymax()).c_str());
               xml_node<> *ymax_node = doc.allocate_node(node_element, "ymax", ymax);
               bndbox_node->append_node(ymax_node);                              
          }
     }
     
     // Prints the document to screen
     //std::cout << doc;
     
     cout << "Writing XML file to: " << xml_filename_ << endl;

     std::ofstream outfile;
     outfile.open(xml_filename_.c_str());          
     outfile << doc;
     outfile.close();
}

int AnnotationParser::ParseFile(std::string file)
{
     xml_filename_ = file;

     bool is_truth = xml_filename_.find(".truth.xml") != std::string::npos;
     bool is_track = xml_filename_.find(".tracks.xml") != std::string::npos;
          
     if (is_truth) {
          ann_type_ = hand;
     } else if (is_track) {
          ann_type_ = track;
     } else {
          cout << "ERROR: Invalid annotation type" << endl;
          return -1;
     }         

     rapidxml::file<> xmlFile(file.c_str());
     rapidxml::xml_document<> doc;
     doc.parse<0>(xmlFile.data());

     // First node should be named <annotation>
     if (strcmp(doc.first_node()->name(), "annotation") != 0) {
          cout << "Invalid xml file: annotation" << endl;
          return -1;
     }

     // Find size
     xml_node<> * size_node = doc.first_node()->first_node("size");
     if (size_node != 0) {
          xml_node<> * width_node = size_node->first_node("width");
          xml_node<> * height_node = size_node->first_node("height");
          xml_node<> * depth_node = size_node->first_node("depth");
          
          width_ = syllo::str2int(width_node->value());
          height_ = syllo::str2int(height_node->value());
          depth_ = syllo::str2int(depth_node->value());
          
     } else {
          cout << "Error: Can't find size node" << endl;
          return -1;
     }

     // Find <number_of_frames>
     xml_node<> * number_of_frames_node = doc.first_node()->first_node("number_of_frames");
     if (number_of_frames_node != 0) {
          number_of_frames_ = syllo::str2int(number_of_frames_node->value());
     }
     
     // Find <metrics>
     xml_node<> * metrics_node = doc.first_node()->first_node("metrics");
     if (metrics_node != 0) {

          xml_node<> * pre_node = metrics_node->first_node("pre");
          if (pre_node != 0) {
               // PRE_TP
               xml_node<> * PRE_TP_node = pre_node->first_node("PRE_TP");
               if (PRE_TP_node != 0) {
                    PRE_TP_ = syllo::str2int(PRE_TP_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing PRE_TP node" << endl;
               }

               // PRE_TN
               xml_node<> * PRE_TN_node = pre_node->first_node("PRE_TN");
               if (PRE_TN_node != 0) {
                    PRE_TN_ = syllo::str2int(PRE_TN_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing PRE_TN node" << endl;
               }

               // PRE_FP
               xml_node<> * PRE_FP_node = pre_node->first_node("PRE_FP");
               if (PRE_FP_node != 0) {
                    PRE_FP_ = syllo::str2int(PRE_FP_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing PRE_FP node" << endl;
               }

               // PRE_FN
               xml_node<> * PRE_FN_node = pre_node->first_node("PRE_FN");
               if (PRE_FN_node != 0) {
                    PRE_FN_ = syllo::str2int(PRE_FN_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing PRE_FN node" << endl;
               }

               // PRE_TPR
               xml_node<> * PRE_TPR_node = pre_node->first_node("PRE_TPR");
               if (PRE_TPR_node != 0) {
                    PRE_TPR_ = syllo::str2double(PRE_TPR_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing PRE_TPR node" << endl;
               }

               // PRE_FPR
               xml_node<> * PRE_FPR_node = pre_node->first_node("PRE_FPR");
               if (PRE_FPR_node != 0) {
                    PRE_FPR_ = syllo::str2double(PRE_FPR_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing PRE_FPR node" << endl;
               }

               // NEG_SAMPLES
               xml_node<> * NEG_SAMPLES_node = pre_node->first_node("NEG_SAMPLES");
               if (NEG_SAMPLES_node != 0) {
                    negative_sample_count_ = syllo::str2int(NEG_SAMPLES_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing NEG_SAMPLES node" << endl;
               }
                              
               // Pd
               xml_node<> * Pd_node = pre_node->first_node("Pd");
               if (Pd_node != 0) {
                    Pd_ = syllo::str2int(Pd_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing Pd node" << endl;
               }

               // Pfa
               xml_node<> * Pfa_node = pre_node->first_node("Pfa");
               if (Pfa_node != 0) {
                    Pfa_ = syllo::str2int(Pfa_node->value());               
               } else { 
                    cout << xml_filename_ << ": Missing Pfa node" << endl;
               }          
          }                   
          
          // TP
          xml_node<> * TP_node = metrics_node->first_node("TP");
          if (TP_node != 0) {
               TP_ = syllo::str2int(TP_node->value());               
          } else { 
               cout << xml_filename_ << ": Missing TP node" << endl;
          }

          // TN
          xml_node<> * TN_node = metrics_node->first_node("TN");
          if (TN_node != 0) {
               TN_ = syllo::str2int(TN_node->value());               
          } else { 
               cout << xml_filename_ << ": Missing TN node" << endl;
          }

          // FP
          xml_node<> * FP_node = metrics_node->first_node("FP");
          if (FP_node != 0) {
               FP_ = syllo::str2int(FP_node->value());               
          } else { 
               cout << xml_filename_ << ": Missing FP node" << endl;
          }

          // FN
          xml_node<> * FN_node = metrics_node->first_node("FN");
          if (FN_node != 0) {
               FN_ = syllo::str2int(FN_node->value());               
          } else { 
               cout << xml_filename_ << ": Missing FN node" << endl;
          }

          // TPR
          xml_node<> * TPR_node = metrics_node->first_node("TPR");
          if (TPR_node != 0) {
               TPR_ = syllo::str2double(TPR_node->value());               
          } else { 
               cout << xml_filename_ << ": Missing TPR node" << endl;
          }

          // FPR
          xml_node<> * FPR_node = metrics_node->first_node("FPR");
          if (FPR_node != 0) {
               FPR_ = syllo::str2double(FPR_node->value());               
          } else { 
               cout << xml_filename_ << ": Missing FPR node" << endl;
          }
     }

     // Does the parameters block exist?
     // Find <parameters>
     xml_node<> * parameters_node = doc.first_node()->first_node("parameters");
     if (parameters_node != 0) {
          
          xml_node<> * ratio_threshold_node = parameters_node->first_node("ratio_threshold");
          if (ratio_threshold_node != 0) {
               params_.ratio_threshold = syllo::str2double(ratio_threshold_node->value());
          } else { 
               cout << xml_filename_ << ": Missing ratio_threshold node" << endl;
          }

          xml_node<> * gradient_threshold_node = parameters_node->first_node("gradient_threshold");
          if (gradient_threshold_node != 0) {
               params_.gradient_threshold = syllo::str2double(gradient_threshold_node->value());
          } else { 
               cout << xml_filename_ << ": Missing gradient_threshold node" << endl;
          }

          xml_node<> * static_threshold_node = parameters_node->first_node("static_threshold");
          if (static_threshold_node != 0) {
               params_.static_threshold = syllo::str2double(static_threshold_node->value());
          } else { 
               cout << xml_filename_ << ": Missing static_threshold node" << endl;
          }

          xml_node<> * history_length_node = parameters_node->first_node("history_length");
          if (history_length_node != 0) {
               params_.history_length = syllo::str2double(history_length_node->value());
          } else { 
               cout << xml_filename_ << ": Missing history_length node" << endl;
          }

          xml_node<> * history_distance_node = parameters_node->first_node("history_distance");
          if (history_distance_node != 0) {
               params_.history_distance = syllo::str2double(history_distance_node->value());
          } else { 
               cout << xml_filename_ << ": Missing history_distance node" << endl;
          }

          xml_node<> * threshold_type_node = parameters_node->first_node("threshold_type");
          if (threshold_type_node != 0) {
               params_.threshold_type = (Parameters::ThresholdType_t)syllo::str2int(threshold_type_node->value());
          } else { 
               cout << xml_filename_ << ": Missing threshold_type node" << endl;
          }
     }
     
     // Find <frames>
     xml_node<> * frames_node = doc.first_node()->first_node("frames");     
     if (frames_node == 0) {
          return -1;          
     }          
     
     // Loop through all frames
     int loop_frame_number = 0;
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
           
          if (frame_number != loop_frame_number) {
               // Add an empty frame, increment loop_frame_number, continue
               Frame frame;
               frame.set_frame_number(loop_frame_number);
               frames[loop_frame_number] = frame;               
               loop_frame_number++;
               continue;
          }
         
          Frame frame;
          frame.set_frame_number(frame_number);
          
          // Loop through objects in this frame.
          xml_node<> *object_node = frame_node->first_node("object");
          do {
               if (object_node == 0) {
                    //cout << "Missing an object node" << endl;
                    break;
               }               
               
               std::string object_name = object_node->first_node("name")->value();                              
               
               wb::Entity object;
               object.set_name(object_name);
               
               //xml_node<> *type = object_node->first_node("type");
               //if (type == 0) {
               //     cout << "Missing type information" << endl;
               //} else {
               //     object.set_type(wb::Entity::str_2_type(type->value()));
               //}
               
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

               object.set_bbox(BoundingBox(xmin,xmax,ymin,ymax));
               
               // Get centroid info
               xml_node<> *centroid = object_node->first_node("centroid");
               if (centroid == 0) {
                    cout << "Missing centroid" << endl;                    
                    cv::Point p((xmin + xmax)/2, (ymin + ymax) / 2);
                    object.set_centroid(p);
               } else {
                    double x, y;
                    x = syllo::str2double(centroid->first_node("x")->value());
                    y = syllo::str2double(centroid->first_node("y")->value());                    
                    object.set_undistorted_centroid(cv::Point2d(x,y));
               }
               
               positive_sample_count_++;
               frame.objects[object_name] = object;
               
          }while ((object_node = object_node->next_sibling()) != 0 );
          
          frames[frame_number] = frame;
          
          loop_frame_number++;
          frame_node = frame_node->next_sibling();          
     }while (frame_node != 0);     
     
     return 0;
}

void AnnotationParser::print()
{
     std::map<int,Frame>::iterator frame_it = frames.begin();
     for(; frame_it != frames.end(); frame_it++) {
          cout << "-----------------------------" << endl;
          cout << "Frame Number: " << frame_it->second.frame_number() << endl; 
          cout << "---------" << endl;
          cout << "Objects: " << endl;
          
          std::map<std::string, wb::Entity>::iterator object_it;
          object_it = frame_it->second.objects.begin();
          for (; object_it != frame_it->second.objects.end(); object_it++) {
               cout << "\t Name: " << object_it->second.name() << endl;
               cout << "\t Bbox: (" << object_it->second.bbox().xmin() << ","
                    << object_it->second.bbox().ymin() << ","
                    << object_it->second.bbox().xmax() << ","
                    << object_it->second.bbox().ymax() << ")" << endl << endl;
          }
     }
}

bool AnnotationParser::export_roi()
{
     if (video_filename_ == "") {
          cout << "No annotation file" << endl;
          return false;
     }
     
     // First determine size of movie we will export
     // Currently, find average height and width of all video frames
     int width_sum = 0, height_sum = 0;
     int count = 0;
     
     // Assumes only single ROI per frame
     std::map<int,Frame>::iterator frame_it = frames.begin();
     for(; frame_it != frames.end(); frame_it++) {
          std::map<std::string, wb::Entity>::iterator object_it;
          object_it = frame_it->second.objects.begin();
          for (; object_it != frame_it->second.objects.end(); object_it++) {
               width_sum += object_it->second.bbox().width();
               height_sum += object_it->second.bbox().height();
               count++;
          }
     }
     
     double width_avg = (double)width_sum / (double)count;
     double height_avg = (double)height_sum / (double)count;
     
     // New Video File to Write
     int lastindex = video_filename_.find_last_of("."); 
     std::string roi_fn = video_filename_.substr(0, lastindex) + 
          std::string("_roi.avi");
     //cout << "Output: " << roi_fn << endl;
     
     cv::Size size(width_avg, height_avg); // height width?
     cv::VideoWriter out(roi_fn, CV_FOURCC('M','J','P','G'), 23, size, true);
     if (!out.isOpened()) {
          cout << "Failed to Export ROI" << endl;
          return false;
     }

     // Open original video file
     cv::VideoCapture in;
     in.open(video_filename_);
     if (!in.isOpened()) {
          cout << "Failed to open input video: Export ROI" << endl;
          return false;
     }
     
     // Find the centroid of each annotated frame, export the average
     // ROI size
     for(frame_it = frames.begin(); frame_it != frames.end(); frame_it++) {
          std::map<std::string, wb::Entity>::iterator object_it;
          object_it = frame_it->second.objects.begin();
          for (; object_it != frame_it->second.objects.end(); object_it++) {
               cv::Mat img;
               in.set(CV_CAP_PROP_POS_FRAMES, frame_it->second.frame_number());
               in.read(img);
               cv::Rect rect = object_it->second.bbox().ForceBox(width_avg, height_avg);
               cv::Mat roi(img, rect);
               out.write(roi);
          }
     }         
     //cout << "Export ROI Complete" << endl;
     return true;
}

std::vector<std::string> AnnotationParser::track_names()
{
     // Get list of all track IDs
     std::vector<std::string> names;
     std::map<int,Frame>::iterator it_frame = frames.begin();
     for (; it_frame != frames.end(); it_frame++) {
          Frame frame = it_frame->second;
          
          // Loop through all objects in each frame
          std::map<std::string, wb::Entity>::iterator it_obj = frame.objects.begin();
          for (; it_obj != frame.objects.end(); it_obj++) {
               // Add the track name to the names vector only if it doesn't
               // already exist in the names vector
               if (std::find(names.begin(), names.end(), 
                             it_obj->first) == names.end()) {
                    names.push_back(it_obj->first);
               }
               //IDs[it_obj->first] = it_obj->first;
          }          
     }
     return names;
}

void AnnotationParser::plot_tracks(std::vector<std::string> &names, 
                                   int min_track_length)
{
     std::map<std::string, std::vector<cv::Point2d> > points;     
     
     // Loop through all frames, plotting tracks that match the user's input
     std::map<int,Frame>::iterator it_frame = frames.begin();
     for (; it_frame != frames.end(); it_frame++) {
          Frame frame = it_frame->second;
          
          // Loop through all objects in each frame
          std::map<std::string, wb::Entity>::iterator it_obj = frame.objects.begin();
          for (; it_obj != frame.objects.end(); it_obj++) {                                                                 
               // Does this object name match any of the IDs we care about?
               if (std::find(names.begin(), names.end(), 
                             it_obj->first) != names.end()) {                    
                    // Push the point onto the appropriate points vector
                    //points[it_obj->first].push_back(it_obj->second.bbox().centroid());
                    //points[it_obj->first].push_back(it_obj->second.centroid());
                    points[it_obj->first].push_back(it_obj->second.undistorted_centroid());
               }     
          }          
     }

     // Plot the tracks;
     std::vector< std::vector<cv::Point2d> > vectors;
     const std::string title = "Tracks";
     std::vector<std::string> labels;
     std::vector<std::string> styles;

     std::map<std::string, std::vector<cv::Point2d> >::iterator it_points;
     for (it_points = points.begin(); it_points != points.end(); it_points++) {          
          // Only plot tracks that have more points than the min track length
          if (it_points->second.size() < (unsigned int)min_track_length) { 
               continue;
          }
          
          vectors.push_back(it_points->second);
          labels.push_back(it_points->first);
          styles.push_back("linespoints");
     }          
     
     syllo::Plot plot;
     std::string options;
     options = "set size ratio -1\n";
     options += "set view equal xy\n"; 
     options += "set size 1,1\n";
     options += "set yrange [*:] reverse\n";
     //options += "set yrange [" + syllo::int2str(height_)  + ":" + syllo::int2str(0) + "]\n";
     //options += "set xrange [" + syllo::int2str(0)  + ":" + syllo::int2str(width_) + "]\n";
     options += "set key outside\n";
     plot.plot(vectors, title, labels, styles, options);
}

std::vector<wb::Entity> AnnotationParser::get_tracks(std::string name)
{
     std::vector<wb::Entity> tracks;
     
     // Loop through all frames
     std::map<int,Frame>::iterator it_frame = frames.begin();
     for (; it_frame != frames.end(); it_frame++) {
          Frame frame = it_frame->second;
          
          // Loop through all objects in each frame
          std::map<std::string, wb::Entity>::iterator it_obj = frame.objects.begin();
          for (; it_obj != frame.objects.end(); it_obj++) {                                                                 
               // Does this object name match the ID we care about?
               if (name == it_obj->first) {                    
                    // Push the point onto the appropriate points vector
                    tracks.push_back(it_obj->second);
               }     
          }          
     }

     return tracks;
}

void AnnotationParser::write_gnuplot_data()
{
     //TODO, future work, maybe.
}

std::map<std::string,double> AnnotationParser::get_params()
{          
     std::map<std::string,double> params;
     params["ratio_threshold"] = params_.ratio_threshold;     
     params["static_threshold"] = params_.static_threshold;
     params["gradient_threshold"] = params_.gradient_threshold;
     params["history_length"] = params_.history_length;
     params["history_distance"] = params_.history_distance;
     params["threshold_type"] = params_.threshold_type;
     return params;
}

void AnnotationParser::reset_metrics()
{
     positive_sample_count_ = 0;
     negative_sample_count_ = 0;
     PRE_Accuracy_ = 0;
     
     TP_ = 0;
     TN_ = 0;
     FP_ = 0;
     FN_ = 0;
     TPR_ = -1;
     FPR_ = -1;
     
     PRE_TP_ = 0;
     PRE_TN_ = 0;
     PRE_FP_ = 0;
     PRE_FN_ = 0;
     PRE_TPR_ = -1;
     PRE_FPR_ = -1;
     Pd_ = -1;
     Pfa_ = -1;

     inside_count_total_ = 0;
     outside_count_total_ = 0;
     count_total_ = 0;
}

std::map<std::string,double> AnnotationParser::get_metrics()
{          
     std::map<std::string,double> metrics;
     metrics["TP"] = TP_;
     metrics["TN"] = TN_;
     metrics["FP"] = FP_;
     metrics["FN"] = FN_;
     metrics["TPR"] = TPR_;
     metrics["FPR"] = FPR_;
     
     metrics["PRE_TP"] = PRE_TP_;
     metrics["PRE_TN"] = PRE_TN_;
     metrics["PRE_FP"] = PRE_FP_;
     metrics["PRE_FN"] = PRE_FN_;
     metrics["PRE_TPR"] = PRE_TPR_;
     metrics["PRE_FPR"] = PRE_FPR_;
     metrics["PRE_Accuracy"] = PRE_Accuracy_;
     
     return metrics;
}

void AnnotationParser::score_detector(AnnotationParser &truth, 
                                   std::vector<std::string> &names)
{
     TP_ = 0;
     TN_ = 0;
     FP_ = 0;
     FN_ = 0;
     TPR_ = FPR_ = -1;

     // Starting with frame number 0, increment through the truth and detector
     // frames until both truth and detector frames are all processed.
     std::map<int,Frame>::const_iterator it_tru_frame = truth.frames.begin();
     std::map<int,Frame>::iterator it_detect_frame = frames.begin();
     for (int i = 0; it_tru_frame != truth.frames.end() || 
               it_detect_frame != frames.end(); i++) {
          
          // Does the truth vector have an entry for this frame?
          bool tru_frame_exists = false;
          Frame tru_frame;          
          if (it_tru_frame != truth.frames.end() && it_tru_frame->first == i) {
               // Grab the truth frame
               tru_frame = it_tru_frame->second;
               tru_frame_exists = true;
               // Increment the frame pointer
               it_tru_frame++;
          }

          // Does the detector have an entry for this frame?
          bool detect_frame_exists = false;
          Frame detect_frame;
          if (it_detect_frame != frames.end() && it_detect_frame->first == i) {
               // Grab the detector's frame
               detect_frame = it_detect_frame->second;
               detect_frame_exists = true;
               // Increment the frame pointer
               it_detect_frame++;
          }

          if (!tru_frame_exists && !detect_frame_exists) {
               // Neither a true frame or detected frame exists.
               // Thus, the detector didn't incorrectly detect an object.
               TN_++;
          } else if (!tru_frame_exists && detect_frame_exists) {
               // If a truth frame doesn't exist, but a detected frame exists
               // determine if the detected frame labelled an object we care
               // about
               std::vector<std::string>::iterator it_name = names.begin();
               for (; it_name != names.end(); it_name++) {
                    // Does this object exist in the detector frame?
                    //Object detected_obj;
                    wb::Entity detected_obj;
                    bool detect_obj_exists = false;
                    detect_obj_exists = detect_frame.contains_object(it_name->c_str(), 
                                                                     detected_obj);
                    if (detect_obj_exists) {
                         // The detector labelled an object, but it doesn't 
                         // exist in a truth frame.
                         FP_++;                         
                    } else {
                         // The object doesn't exist in the sequence of truth
                         // frames. Also, it doesn't exist in the detector
                         // frame.
                         TN_++;
                    }
               }               
          } else if (tru_frame_exists && !detect_frame_exists) {
               // If a truth frame exists, but a detector didn't detect
               // anything. We need to check to see if the truth frame has an 
               // object that we care about that the detector missed.
               std::vector<std::string>::iterator it_name = names.begin();
               for (; it_name != names.end(); it_name++) {
                    // Does this object exist in the truth frame?
                    //Object tru_obj;
                    wb::Entity tru_obj;
                    bool tru_obj_exists = false;
                    tru_obj_exists = tru_frame.contains_object(it_name->c_str(), 
                                                                     tru_obj);
                    if (tru_obj_exists) {
                         // The object exists in the truth frame, but doesn't
                         // exist in the detector frame. False Negative.
                         FN_++;
                    } else {
                         // The object doesn't exist in the truth frame and it
                         // doesn't exist in the detector frame. True Negative.
                         TN_++;
                    }
               }               
          } else if (tru_frame_exists && detect_frame_exists) {
               // Both a truth frame and a detector frame exists. Loop through 
               // all object names we care about and count stats.

               // Loop through the "names" vector, which contains names of
               // objects we care about scoring the detector on
               std::vector<std::string>::iterator it_name = names.begin();
               for (; it_name != names.end(); it_name++) {
                    // Search for this named object in ground truth frame
                    //Object tru_obj;
                    wb::Entity tru_obj;
                    bool tru_obj_exists;
                    tru_obj_exists = tru_frame.contains_object(it_name->c_str(), 
                                                               tru_obj);
                   
                    // Does this object exist in the detector frame?
                    //Object detected_obj;
                    wb::Entity detected_obj;
                    bool obj_exists;
                    obj_exists = detect_frame.contains_object(it_name->c_str(), 
                                                       detected_obj);
                    
                    if (!tru_obj_exists && !obj_exists) {
                         // If the object name doesn't exist in the truth frame
                         // or the detector frame, it is a True Negative
                         TN_++;                         
                    } else if (!tru_obj_exists && obj_exists) {
                         // If the object doesn't exist in the truth frame, 
                         // but exists in the detector frame, it is a false
                         // positive
                         FP_++;
                    } else if (tru_obj_exists && !obj_exists) {
                         // If the object exists in the truth frame, but it 
                         // doesn't exist in the detector frame, it is a false
                         // negative
                         FN_++;
                    } else if (tru_obj_exists && obj_exists) {
                         // If the object exists in both the truth frame and
                         // the detector frame, we have to determine if the
                         // detector's placement of the object's centroid is
                         // within the bounding box of the truth frame.
                         bool contains;
                         contains = tru_obj.bbox().contains(detected_obj.bbox().centroid());
                         if (contains) {
                              // The detected object's centroid is within the
                              // bounding box of the truth frame's object.
                              TP_++;
                         } else {
                              // The detected object's centroid is outside of
                              // the bounding box of the truth frame's object.
                              FP_++;
                         }
                    }
               }
          }
     }
     
     TPR_ = (double)TP_ / (double)(TP_ + FN_) ; 
     FPR_ = (double)FP_ / (double)(FP_ + TN_) ; 
    
     syllo::fill_line("+");
     cout << "True Positives: " << TP_ << endl;
     cout << "True Negatives: " << TN_ << endl;
     cout << "False Positives: " << FP_ << endl;
     cout << "False Negatives: " << FN_ << endl;     
     metrics_present_ = true;
     syllo::fill_line("+");
}

void AnnotationParser::score_detector_2(AnnotationParser &truth, 
                                   std::vector<std::string> &names)
{
     TP_ = 0;
     TN_ = 0;
     FP_ = 0;
     FN_ = 0;
     TPR_ = FPR_ = -1;

     // Starting with frame number 0, increment through the truth and detector
     // frames until both truth and detector frames are all processed.
     std::map<int,Frame>::const_iterator it_tru_frame = truth.frames.begin();
     std::map<int,Frame>::iterator it_detect_frame = frames.begin();
     for (int i = 0; it_tru_frame != truth.frames.end() || 
               it_detect_frame != frames.end(); i++) {
          
          // Does the truth vector have an entry for this frame?
          bool tru_frame_exists = false;
          Frame tru_frame;          
          if (it_tru_frame != truth.frames.end() && it_tru_frame->first == i) {
               // Grab the truth frame
               tru_frame = it_tru_frame->second;
               tru_frame_exists = true;
               // Increment the frame pointer
               it_tru_frame++;
          }

          // Does the detector have an entry for this frame?
          bool detect_frame_exists = false;
          Frame detect_frame;
          if (it_detect_frame != frames.end() && it_detect_frame->first == i) {
               // Grab the detector's frame
               detect_frame = it_detect_frame->second;
               detect_frame_exists = true;
               // Increment the frame pointer
               it_detect_frame++;
          }

          if (!tru_frame_exists && !detect_frame_exists) {
               // Neither a true frame or detected frame exists.
               // Thus, the detector didn't incorrectly detect an object.
               TN_++;
          } else if (!tru_frame_exists && detect_frame_exists) {
               // If a truth frame doesn't exist, but a detected frame exists
               // determine if the detected frame labelled an object we care
               // about
               std::vector<std::string>::iterator it_name = names.begin();
               for (; it_name != names.end(); it_name++) {
                    // Does this object exist in the detector frame?
                    //Object detected_obj;
                    wb::Entity detected_obj;
                    bool detect_obj_exists = false;
                    wb::Entity::EntityType_t type = wb::Entity::str_2_type(*it_name);
                    detect_obj_exists = detect_frame.contains_type(type, detected_obj);
                    
                    if (detect_obj_exists) {
                         // The detector labelled an object, but it doesn't 
                         // exist in a truth frame.
                         FP_++;                         
                    } else {
                         // The object doesn't exist in the sequence of truth
                         // frames. Also, it doesn't exist in the detector
                         // frame.
                         TN_++;
                    }
               }               
          } else if (tru_frame_exists && !detect_frame_exists) {
               // If a truth frame exists, but a detector didn't detect
               // anything. We need to check to see if the truth frame has an 
               // object that we care about that the detector missed.
               std::vector<std::string>::iterator it_name = names.begin();
               for (; it_name != names.end(); it_name++) {
                    // Does this object exist in the truth frame?
                    //Object tru_obj;
                    wb::Entity tru_obj;
                    bool tru_obj_exists = false;

                    wb::Entity::EntityType_t type = wb::Entity::str_2_type(*it_name);
                    tru_obj_exists = tru_frame.contains_type(type, tru_obj);
                                        
                    if (tru_obj_exists) {
                         // The object exists in the truth frame, but doesn't
                         // exist in the detector frame. False Negative.
                         FN_++;
                    } else {
                         // The object doesn't exist in the truth frame and it
                         // doesn't exist in the detector frame. True Negative.
                         TN_++;
                    }
               }               
          } else if (tru_frame_exists && detect_frame_exists) {
               // Both a truth frame and a detector frame exists. Loop through 
               // all object names we care about and count stats.

               // Loop through the "names" vector, which contains names of
               // objects we care about scoring the detector on
               std::vector<std::string>::iterator it_name = names.begin();
               for (; it_name != names.end(); it_name++) {
                    // Search for this named object in ground truth frame
                    //Object tru_obj;
                    wb::Entity tru_obj;
                    bool tru_obj_exists;
                    //tru_obj_exists = tru_frame.contains_object(it_name->c_str(), 
                    //                                           tru_obj);
                    wb::Entity::EntityType_t type = wb::Entity::str_2_type(*it_name);
                    tru_obj_exists = tru_frame.contains_type(type, tru_obj);
                    
                    // Does this object exist in the detector frame?
                    //Object detected_obj;
                    wb::Entity detected_obj;
                    bool obj_exists;
                    //obj_exists = detect_frame.contains_object(it_name->c_str(), 
                    //                                   detected_obj);
                    obj_exists = detect_frame.contains_type(type, detected_obj);
                    
                    if (!tru_obj_exists && !obj_exists) {
                         // If the object name doesn't exist in the truth frame
                         // or the detector frame, it is a True Negative
                         TN_++;                         
                    } else if (!tru_obj_exists && obj_exists) {
                         // If the object doesn't exist in the truth frame, 
                         // but exists in the detector frame, it is a false
                         // positive
                         FP_++;
                    } else if (tru_obj_exists && !obj_exists) {
                         // If the object exists in the truth frame, but it 
                         // doesn't exist in the detector frame, it is a false
                         // negative
                         FN_++;
                    } else if (tru_obj_exists && obj_exists) {
                         // If the object exists in both the truth frame and
                         // the detector frame, we have to determine if the
                         // detector's placement of the object's centroid is
                         // within the bounding box of the truth frame.
                         bool contains;
                         contains = tru_obj.bbox().contains(detected_obj.bbox().centroid());
                         if (contains) {
                              // The detected object's centroid is within the
                              // bounding box of the truth frame's object.
                              TP_++;
                         } else {
                              // The detected object's centroid is outside of
                              // the bounding box of the truth frame's object.
                              FP_++;
                         }
                    }
               }
          }
     }
     
     TPR_ = (double)TP_ / (double)(TP_ + FN_) ; 
     FPR_ = (double)FP_ / (double)(FP_ + TN_) ; 
    
     syllo::fill_line("+");
     cout << "True Positives: " << TP_ << endl;
     cout << "True Negatives: " << TN_ << endl;
     cout << "False Positives: " << FP_ << endl;
     cout << "False Negatives: " << FN_ << endl;   
     cout << "TPR: " << TPR_ << endl;
     cout << "FPR: " << FPR_ << endl;
     metrics_present_ = true;
     syllo::fill_line("+");
}

//void AnnotationParser::score_preprocessing(int frame, AnnotationParser &truth, 
//                                           cv::Mat &img)
//{
//     if (img.empty()) {
//          return;
//     }
//          
//     cv::Mat img_clone = img.clone();
//     cv::cvtColor(img_clone, img_clone, CV_GRAY2BGR);
//     
//     int inside_count = 0;
//     int outside_count = 0;
//     if (truth.frames.count(frame) > 0) {
//          // Search for non-zero pixels in img
//          for(int r = 0; r < img.rows; r++) {
//               for(int c = 0; c < img.cols; c++) {
//                    if (img.at<uchar>(r,c) != 0) {
//                         cv::Point p(c,r);
//                         // Is the point inside one of the objects?
//                         std::map<std::string, wb::Entity>::iterator it;
//                         for (it = truth.frames[frame].objects.begin(); 
//                              it != truth.frames[frame].objects.end();
//                              it++) {
//                              
//                              cv::rectangle(img_clone, it->second.bbox().rectangle(),cv::Scalar(0,255,0), 1, 8, 0);
//                              
//                              if (it->second.bbox().contains(p)) {
//                                   // Point inside of one of the "truth" boxes
//                                   inside_count++;
//                              } else {
//                                   outside_count++;
//                              }
//                         }
//                    }
//               }
//          }
//     } 
//     inside_count_total_ += inside_count;
//     outside_count_total_ += outside_count;
//     count_total_ += inside_count + outside_count;     
//     
//     //cout << "Inside count: " << inside_count << endl;
//     //cout << "Outside count: " << outside_count << endl;
//     cv::imshow("POD Rects", img_clone);
//}

void AnnotationParser::score_preprocessing_final(AnnotationParser &truth)
{
     //Pd_ = (double)inside_count_total_ / (double)count_total_;     
     //Pfa_ = (double)outside_count_total_ / (double)count_total_;

     cout << "Pd: " << Pd_ << endl;
     cout << "Pfa: " << Pfa_ << endl;

     PRE_TPR_ = (double)PRE_TP_ / (double)(PRE_TP_ + PRE_FN_); 
     PRE_FPR_ = (double)PRE_FP_ / (double)(PRE_FP_ + PRE_TN_); 

     PRE_Accuracy_ = (double)(PRE_TP_ + PRE_TN_) / ((double)(PRE_TP_ + PRE_TN_ + PRE_FN_+ PRE_FP_));

     syllo::fill_line("+");
     cout << "Pre True Positives: " << PRE_TP_ << endl;
     cout << "Pre True Negatives: " << PRE_TN_ << endl;
     cout << "Pre False Positives: " << PRE_FP_ << endl;
     cout << "Pre False Negatives: " << PRE_FN_ << endl;   
     cout << "PRE TPR: " << PRE_TPR_ << endl;
     cout << "PRE FPR: " << PRE_FPR_ << endl;     
     cout << "PRE Accuracy: " << PRE_Accuracy_ << endl;     
}

// Uses negative samples that have a rectangle the same size as the positive
// object's rectangle.
void AnnotationParser::score_preprocessing_2(int frame, AnnotationParser &truth, 
                                             cv::Mat &img, cv::Mat &mask)
{
     if (img.empty()) {
          return;
     }     
     
     cv::Mat img_clone = img.clone();
     cv::cvtColor(img_clone, img_clone, CV_GRAY2BGR);     
     
     // Use rectangles outside of the positive examples as negatives.
     // A rectangle is a positive if it has any non-zero pixels inside of it.     
                         
     int TP = 0, TN = 0, FP = 0, FN = 0;
          
     if (truth.frames.count(frame) > 0) {
          // There is an annotated frame.                    
          
          // Loop through each positive object and determine if it is either a
          // true positive or false negative.          
          for (std::map<std::string, wb::Entity>::iterator it = truth.frames[frame].objects.begin(); 
               it != truth.frames[frame].objects.end();
               it++) {

               // Draw the object's bounding box
               cv::rectangle(img_clone, it->second.bbox().rectangle(), cv::Scalar(0,255,0), 1, 8, 0);
               
               // Does this object contain any non-zero pixels?
               bool object_contains_pixel = false;
               BoundingBox b = it->second.bbox();
               for(int r = b.ymin(); r < b.ymax(); r++) {
                    for(int c = b.xmin(); c < b.xmax(); c++) {
                         if (img.at<uchar>(r,c) != 0) {
                              object_contains_pixel = true;
                         }
                    }
               }

               if (object_contains_pixel) {
                    // At least one non-zero pixel was found in the object's
                    // bounding box, this is a single true positive
                    TP++;
               } else {
                    // There weren't any non-zero pixels found in the object's
                    // bounding box, this is a single false negative.
                    FN++;
               }                              
          }

          // Using the dimensions of the positive samples' rectangles, randomly
          // select negative samples from the current frame.
          std::list<cv::Rect> neg_rects;          
          for (std::map<std::string, wb::Entity>::iterator it1 = truth.frames[frame].objects.begin(); 
               it1 != truth.frames[frame].objects.end(); it1++) {
               cv::Rect pos_rect = it1->second.bbox().rectangle();
               cv::Rect rect = pos_rect;

               // Determine the number of negative examples we will use for this frame
               int attempts = 0;
               int num_neg_examples = neg_to_pos_ratio_;
               for (int i = 0; i < num_neg_examples; i++) {
                    attempts++;
                    
                    // If there are too many attempts to try to fit a negative
                    // box but we keep getting rejected attempts, just exit, so
                    // we don't hang forever. User should adjust ratio.
                    if (attempts > 500) {
                         cout << "Too many attempts to fit negative box" << endl;
                         cout << "File: " << xml_filename_ << endl;
                         break;
                    }
                    
                    //int x = rng_x();
                    //int y = rng_y();
                    int x = rand() % (img.cols - rect.width);
                    int y = rand() % (img.rows - rect.height);
                    rect.x = x;
                    rect.y = y;

                    // This new rectangle can't be outside of the (2) sonar's
                    // mask, (1) outside of the image's dimensions, (3)
                    // intersecting with positive rectangles, (4) intersecting
                    // with other negative rectangles

                    // (1) Is it outside of the original image?
                    bool valid = BoundingBox::is_within_image(rect, img);
                    if (!valid) {
                         i--;
                         continue;
                    }
                    
                    // (2) Is it outside of the sonar's mask?
                    valid = BoundingBox::is_within_mask(rect, mask);
                    if (!valid) {
                         i--;
                         continue;
                    }                    
                    
                    // (3) Does it collide with a positive sample?
                    for (std::map<std::string, wb::Entity>::iterator it2 = truth.frames[frame].objects.begin(); 
                         it2 != truth.frames[frame].objects.end(); it2++) {
                         valid = !BoundingBox::overlap(rect, it2->second.bbox().rectangle());
                         if (!valid) break; // overlaps with positive sample, break.
                    }
                    if (!valid) {
                         i--;
                         continue;
                    }

                    // (4) Does it collide with another negative rectangle?
                    for (std::list<cv::Rect>::iterator it2 = neg_rects.begin();
                         it2 != neg_rects.end(); it2++) {
                         valid = !BoundingBox::overlap(rect, *it2);
                         if (!valid) break; // overlaps with negative sample, break.
                    }
                    if (!valid) {
                         i--;
                         continue;
                    }
                    
                    neg_rects.push_back(rect);
                    negative_sample_count_++;

                    // Draw a red box over the "negative" sample
                    cv::rectangle(img_clone, rect, cv::Scalar(0,0,255), 1, 8, 0);
                    
                    // At this point in the loop, the randomly located
                    // rectangle has passed all of the checks. Now determine if
                    // there are any non-zero pixels in the rectangle (false
                    // positive) or if all of the pixels are zero (true
                    // negative)
                    bool non_zero_found = false;
                    cv::Mat roi(img,rect);
                    for (int r = 0; r < roi.rows; r++) {
                         for (int c = 0; c < roi.cols; c++) {
                              if (roi.at<uchar>(r,c) != 0) {
                                   non_zero_found = true;
                              }
                         }
                    }
                    if (non_zero_found) {
                         FP++;
                    } else {
                         TN++;
                    }
               } // loop over number of negative examples requested
          } // loop over number of positive examples
     } else {
          cout << "=========> Missing annotated frame: " << xml_filename_  << " frame: " << frame << endl;
     }

     PRE_TP_ += TP;
     PRE_TN_ += TN;
     PRE_FP_ += FP;
     PRE_FN_ += FN;

     metrics_present_ = true;

#if 0
     cout << "==========" << endl;
     cout << "TP: " << TP << endl;
     cout << "TN: " << TN << endl;
     cout << "FP: " << FP << endl;
     cout << "FN: " << FN << endl;
     cout << "PRE_FN: " << PRE_FN_ << endl;     
     cv::imshow("POD Rects", img_clone);
#endif
     
}


// Counts at most one false positive per frame.
void AnnotationParser::score_preprocessing(int frame, AnnotationParser &truth, 
                                           cv::Mat &img)
{
     if (img.empty()) {
          return;
     }
          
     cv::Mat img_clone = img.clone();
     cv::cvtColor(img_clone, img_clone, CV_GRAY2BGR);
     
     int TP = 0, TN = 0, FP = 0, FN = 0;
     
     if (truth.frames.count(frame) > 0) {
          // There is an annotated frame.          
          std::map<std::string, wb::Entity>::iterator it;
          for (it = truth.frames[frame].objects.begin(); 
               it != truth.frames[frame].objects.end();
               it++) {

               // Draw the object's bounding box
               cv::rectangle(img_clone, it->second.bbox().rectangle(),cv::Scalar(0,255,0), 1, 8, 0);
               
               // Does this object contain any non-zero pixels?
               bool object_contains_pixel = false;
               BoundingBox b = it->second.bbox();
               for(int r = b.ymin(); r < b.ymax(); r++) {
                    for(int c = b.xmin(); c < b.xmax(); c++) {
                         if (img.at<uchar>(r,c) != 0) {
                              object_contains_pixel = true;
                              //TP++;
                         }
                         //else {
                         //     FN++;
                         //}
                    }
               }

               if (object_contains_pixel) {
                    // At least one non-zero pixel was found in the object's
                    // bounding box, this is a single true positive
                    TP++;
               } else {
                    // There weren't any non-zero pixels found in the object's
                    // bounding box, this is a single false negative.
                    FN++;
               }                              
          }
          // Are there any non-zero pixels outside of the object's bounding
          // boxes? (Counting false positives and true negatives
          bool contains_FP = false;
          bool contains_TN = false;
          for(int r = 0; r < img.rows; r++) {
               for(int c = 0; c < img.cols; c++) {
                    cv::Point p(c,r);
                    
                    // Does this point exist in an object's bounding box?
                    bool object_contains_pixel = false;
                    std::map<std::string, wb::Entity>::iterator it;
                    for (it = truth.frames[frame].objects.begin(); 
                         it != truth.frames[frame].objects.end();
                         it++) {
                         if (it->second.bbox().contains(p)) {
                              // The point exists inside of an object's
                              // bounding box.
                              object_contains_pixel = true;
                         }
                    }
                    
                    if (!object_contains_pixel) {
                         // The point is NOT contained by an object
                         if (img.at<uchar>(r,c) != 0) {
                              // The pixel has non-zero value, but it isn't
                              // contained by any of the objects. This is a
                              // false positive.
                              //FP++;            
                              contains_FP = true;
                         } else {
                              // The pixel has zero value and it isn't
                              // contained by any of the objects. This is a
                              // true negative.
                              //TN++;
                              contains_TN = true;
                         }             
                    } else {
                         // The pixel is contained by an object. We previously
                         // accounted for true positives above.
                    }
               }
          }

          if (contains_FP) FP++;
          if (contains_TN) TN++;
          
     } else {
          cout << "=========> Missing annotated frame: " << xml_filename_  << " frame: " << frame << endl;          
     }        

     PRE_TP_ += TP;
     PRE_TN_ += TN;
     PRE_FP_ += FP;
     PRE_FN_ += FN;

     metrics_present_ = true;

#if 0
     cout << "==========" << endl;
     cout << "TP: " << TP << endl;
     cout << "TN: " << TN << endl;
     cout << "FP: " << FP << endl;
     cout << "FN: " << FN << endl;
     cout << "PRE_FN: " << PRE_FN_ << endl;     
     cv::imshow("POD Rects", img_clone);
#endif
}

// Every pixel is either a TP, FP, TN, or FN
void AnnotationParser::score_preprocessing_3(int frame, AnnotationParser &truth, 
                                             cv::Mat &img, cv::Mat &mask)
{
     if (img.empty()) {
          return;
     }
          
     cv::Mat img_clone = img.clone();
     cv::cvtColor(img_clone, img_clone, CV_GRAY2BGR);
     
     int TP = 0, TN = 0, FP = 0, FN = 0;
     
     if (truth.frames.count(frame) > 0) {
          // There is an annotated frame.          
          std::map<std::string, wb::Entity>::iterator it;
          for (it = truth.frames[frame].objects.begin(); 
               it != truth.frames[frame].objects.end();
               it++) {

               // Draw the object's bounding box
               cv::rectangle(img_clone, it->second.bbox().rectangle(),cv::Scalar(0,255,0), 1, 8, 0);
               
               // Does this object contain any non-zero pixels?               
               BoundingBox b = it->second.bbox();
               for(int r = b.ymin(); r < b.ymax(); r++) {
                    for(int c = b.xmin(); c < b.xmax(); c++) {
                         if (img.at<uchar>(r,c) != 0) {                              
                              TP++;
                         } else {
                              FN++;
                         }
                    }
               }               
          }
          // Are there any non-zero pixels outside of the object's bounding
          // boxes? (Counting false positives and true negatives          
          for(int r = 0; r < img.rows; r++) {
               for(int c = 0; c < img.cols; c++) {
                    cv::Point p(c,r);
                    
                    // Does this point exist in an object's bounding box?
                    bool object_contains_pixel = false;
                    std::map<std::string, wb::Entity>::iterator it;
                    for (it = truth.frames[frame].objects.begin(); 
                         it != truth.frames[frame].objects.end();
                         it++) {
                         if (it->second.bbox().contains(p)) {
                              // The point exists inside of an object's
                              // bounding box.
                              object_contains_pixel = true;
                         }
                    }
                    
                    if (!object_contains_pixel) {
                         // The point is NOT contained by an object
                         if (img.at<uchar>(r,c) != 0) {
                              // The pixel has non-zero value, but it isn't
                              // contained by any of the objects. This is a
                              // false positive.
                              FP++;                              
                         } else {
                              // The pixel has zero value and it isn't
                              // contained by any of the objects. This is a
                              // true negative.
                              TN++;                              
                         }             
                    } else {
                         // The pixel is contained by an object. We previously
                         // accounted for true positives above.
                    }
               }
          }                    
     } else {
          cout << "=========> Missing annotated frame: " << xml_filename_  << " frame: " << frame << endl;          
     }        

     PRE_TP_ += TP;
     PRE_TN_ += TN;
     PRE_FP_ += FP;
     PRE_FN_ += FN;

     metrics_present_ = true;

#if 0
     cout << "==========" << endl;
     cout << "TP: " << TP << endl;
     cout << "TN: " << TN << endl;
     cout << "FP: " << FP << endl;
     cout << "FN: " << FN << endl;
     cout << "PRE_FN: " << PRE_FN_ << endl;     
     cv::imshow("POD Rects", img_clone);
#endif
}
