#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_int.hpp>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/syllo.h>

#include <opencv_workbench/utils/AnnotationParser.h>
#include <opencv_workbench/utils/Frame.h>
#include <opencv_workbench/plot/Plot.h>

#include <opencv_workbench/detector/Detector.h>
#include <opencv_workbench/plugin_manager/PluginManager.h>

#include <yaml-cpp/yaml.h>

using std::cout;
using std::endl;

PluginManager<Detector, Detector_maker_t> plugin_manager_;

class DataSet
{
public:
     DataSet()
          {
               filename = "";
               start_frame = end_frame = 0;
               valid = false;
          }
     std::string filename;
     bool valid;
     int start_frame;
     int end_frame;
     int positive_sample_count;
     int frame_count;     
     int annotated_frame_count;
};

void operator >> (const YAML::Node& node, DataSet& d)
{
     node["file"] >> d.filename;

     // Does the "frames" limiter exist:
     if(const YAML::Node *pframes = node.FindValue("frames")) {
          (*pframes)[0] >> d.start_frame;
          (*pframes)[1] >> d.end_frame;
     } else {
          // Open it and get the number of frames
          syllo::Stream stream;
          syllo::Status status = stream.open(d.filename);
          if (status != syllo::Success) {
               cout << "Failed to open: " << d.filename << endl;
               d.valid = false;
               return;
          }
          d.start_frame = 0;
          d.end_frame = stream.get_frame_count()-1;
     }

     AnnotationParser parser_truth;
     int retcode = parser_truth.CheckForFile(d.filename, AnnotationParser::hand);
     if (retcode != 0) {
          cout << "===> Warning: File doesn't have truth data: " << d.filename << endl;
          d.valid = false;
          return;
     } else {
          d.positive_sample_count = parser_truth.positive_sample_count();
          d.frame_count = parser_truth.number_of_frames();
          d.annotated_frame_count = parser_truth.frames.size();
     }     
     d.valid = true;
}

int main(int argc, char *argv[])
{
     int c;
     std::string output_dir = "";
     std::string yaml_file = "";
     int output_dir_flag = 0;
     std::string seed_str;
     int seed_flag = 0;
     int k_folds = 3;
     double test_ratio = 0.10;

     while ((c = getopt (argc, argv, "r:k:s:o:y:")) != -1) {
          switch (c) {
          case 'r':
               test_ratio = syllo::str2double(std::string(optarg));
               break;
          case 'k':
               k_folds = syllo::str2int(std::string(optarg));
               break;
          case 's':
               seed_str = std::string(optarg);
               seed_flag = 1;
               break;
          case 'o':
               output_dir_flag = 1;
               output_dir = std::string(optarg);
               break;
          case 'y':
               yaml_file = std::string(optarg);
               break;
          case '?':
               if (optopt == 'c') {
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
               } else if (optopt == 'y') {
                    fprintf (stderr, "Option -%c requires a yaml file name as an argument.\n", optopt);
               } else if (isprint (optopt)) {
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
               } else if (optopt == 's') {
                    fprintf (stderr, "Option -%c requires an integer seed.\n", optopt);
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

     if (output_dir_flag == 0) {
          cout << "Output directory required" << endl;
          return -1;
     }

     if ( !fs::exists( fs::path(output_dir) ) ) {
          if(!fs::create_directories(fs::path(output_dir))) {
               cout << "ERROR: Unable to create output directory: "
                    << output_dir << endl;
               return -1;
          }          
     }

     std::ifstream fin(yaml_file.c_str());
     YAML::Parser parser(fin);
     YAML::Node doc;
     parser.GetNextDocument(doc);

     // Write the full filenames to a simple text file for bash to handle
     std::ofstream filenames_output;
     std::string filenames_output_str = output_dir + "/" + "files.txt";
     filenames_output.open(filenames_output_str.c_str(), std::ofstream::out);
     
     int total_pos_count = 0;
     int total_frame_count = 0;
     int total_ann_frame_count = 0;     
          
     std::vector<DataSet> datasets;
     // Check for "data" map
     if(const YAML::Node *data = doc.FindValue("data")) {
          for(unsigned i=0; i < data->size();i++) {
               DataSet d;
               (*data)[i] >> d;
               
               if (d.valid) {
                    datasets.push_back(d);
                    filenames_output << d.filename << endl;
                    total_pos_count += d.positive_sample_count;
                    total_frame_count += d.frame_count;
                    total_ann_frame_count += d.annotated_frame_count;
               }               
          }
     } else {
          cout << "Invalid format. Missing data" << endl;
          return -1;
     }
     filenames_output.close();

     // Write out the numbers on the input files
     std::ofstream nums_output;
     std::string nums_output_str = output_dir + "/" + "input_nums.txt";
     nums_output.open(nums_output_str.c_str(), std::ofstream::out);     
     nums_output << "Number of valid input files: " << datasets.size() << endl;
     nums_output << "Total Positive Count: " << total_pos_count << endl;
     nums_output << "Total Frame Count: " << total_frame_count << endl;
     nums_output << "Total Annotated Frame Count: " << total_ann_frame_count << endl;
     nums_output.close();     

     // Fill up the frames map with the frame counts of all the data
     // sets. Later, we will randomly sample from this frames map to select the
     // test, validation, and training frames
     std::map<unsigned int,Frame> frames;
     int f = 0;
     for (std::vector<DataSet>::iterator it = datasets.begin();
          it != datasets.end(); it++) {

          for (int i = it->start_frame; i <= it->end_frame; i++) {
               Frame frame;
               frame.set_frame_type(Frame::train);
               frames[f] = frame;
               f++;
          }
     }
     
     int test_count = test_ratio * frames.size();

     boost::mt19937 gener;
     boost::random::uniform_int_distribution<> uniform(0,frames.size()-1);
     boost::variate_generator<boost::mt19937&,boost::random::uniform_int_distribution<> > rng(gener, uniform);

     if (seed_flag == 1) {
          rng.engine().seed(static_cast<unsigned int>(syllo::str2int(seed_str)));
          rng.distribution().reset();
     } else {
          rng.engine().seed(static_cast<unsigned int>(std::time(0)));
          rng.distribution().reset();
     }

     // Using the uniform random distribution, designate the final "test"
     // frames
     for (int i = 0; i < test_count; i++) {
          int num = rng();
          if (frames[num].frame_type() == Frame::train) {
               // Only use this random number if the frame wasn't set yet
               frames[num].set_frame_type(Frame::test);
          } else {
               // If this frame was already designated, don't count this num
               --i;
          }
     }

     // Reset the RNG to not use a seed
     rng.engine().seed(static_cast<unsigned int>(std::time(0)));
     rng.distribution().reset();

     // Using k-folds, remember that we removed the final test set
     int count_per_fold = (frames.size() - test_count) / k_folds;

     std::map<unsigned int,Frame> orig_frames = frames;
     std::map<unsigned int,Frame> frames_history = frames;

     for (int fold = 0; fold < k_folds; fold++) {
          frames = orig_frames; // reset on each fold

          // Designate count_per_fold frames as validation frames
          for (int i = 0; i < count_per_fold; i++) {
               unsigned int num = rng();
               if (frames_history[num].frame_type() == Frame::train) {
                    // Only use this random number if the frame wasn't set yet
                    frames[num].set_frame_type(Frame::validate);
                    frames_history[num].set_frame_type(Frame::validate);
               } else {
                    // If this frame was already designated, crawl up to the
                    // next available slot
                    while (true) {
                         num++;
                         if (num >= frames_history.size()) {
                              num = 0;
                         }

                         if (frames_history[num].frame_type() == Frame::train) {
                              // Found an open slot, set it, break
                              frames[num].set_frame_type(Frame::validate);
                              frames_history[num].set_frame_type(Frame::validate);
                              break;
                         }
                    }
               }
          }

          // Output the yaml files for these data sets
          f = 0;
          for (std::vector<DataSet>::iterator it = datasets.begin();
               it != datasets.end(); it++) {

               YAML::Emitter out;
               out << YAML::BeginSeq;

               for (int i = 0; i < it->frame_count; i++) {
                    out << YAML::BeginMap;
                    out << YAML::Key << "frame";
                    out << YAML::Value << syllo::int2str(i);
                    out << YAML::Key << "frame_type";

                    // Make sure the current frame number is within the
                    // start/end range
                    if (i >= it->start_frame && i <= it->end_frame) {
                         out << YAML::Value << frames[f].frame_type();
                         f++;
                    } else {
                         // unused if the frame is not within the start/end nums
                         out << YAML::Value << Frame::unused;
                    }
                    out << YAML::EndMap;
               }
               
               //for (int i = it->start_frame; i <= it->end_frame; i++) {
               //     out << YAML::BeginMap;
               //     out << YAML::Key << "frame";
               //     out << YAML::Value << syllo::int2str(i);
               //     out << YAML::Key << "frame_type";
               //     out << YAML::Value << frames[f].frame_type();
               //     out << YAML::EndMap;
               //     
               //     f++;
               //}
               
               out << YAML::EndSeq;

               std::string fold_dir = output_dir + "/fold-" + syllo::int2str(fold);

               boost::filesystem::path dir(fold_dir);
               boost::filesystem::create_directory(dir);

               std::string out_filename = fold_dir +  "/" + fs::path(it->filename).stem().string() + ".frame_types.yaml";
               std::ofstream output;
               output.open(out_filename.c_str(), std::ofstream::out);
               output << out.c_str();
               output.close();
          }
          cout << "Fold Generation Progress: " << fold+1 << " / " << k_folds << endl;
     }

     return 0;
}
