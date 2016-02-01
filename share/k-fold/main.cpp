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
          }
     std::string filename;
     int start_frame;
     int end_frame;
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
          }
          d.start_frame = 0;
          d.end_frame = stream.get_frame_count()-1;
     }
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

     while ((c = getopt (argc, argv, "k:s:o:y:")) != -1) {
          switch (c) {
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
          cout << "Output directory doesn't exist" << endl;
          return -1;
     }

     std::ifstream fin(yaml_file.c_str());
     YAML::Parser parser(fin);
     YAML::Node doc;
     parser.GetNextDocument(doc);

     std::vector<DataSet> datasets;

     // Check for "data" map
     if(const YAML::Node *data = doc.FindValue("data")) {
          for(unsigned i=0; i < data->size();i++) {
               DataSet d;
               (*data)[i] >> d;
               datasets.push_back(d);
          }
     } else {
          cout << "Invalid format. Missing data" << endl;
          return -1;
     }

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

     double test_ratio = 0.10;
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
               for (int i = it->start_frame; i <= it->end_frame; i++) {
                    out << YAML::BeginMap;
                    out << YAML::Key << "frame";
                    out << YAML::Value << syllo::int2str(i);
                    out << YAML::Key << "frame_type";
                    out << YAML::Value << frames[f].frame_type();
                    out << YAML::EndMap;

                    f++;
               }
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
