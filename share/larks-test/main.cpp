#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <opencv_workbench/LARKS/LARKS.h>
#include <opencv_workbench/utils/RMS.h>
#include <time.h>

using std::cout;
using std::endl;

#define LARKS_METHOD -1
#define CV_TM_SQDIFF_METHOD 1

int main(int argc, char *argv[])
{
     cout << "==========================================" << endl;          
     if (argc != 8) {
          cout << "Invalid arguments" << endl;
          cout << "usage: "<< endl;
          cout << "\t" << argv[0] << " video-file query-file labels method "
               << "start-thresh thresh-step end-thresh" << endl;
          return -1;
     }

     // Argument processing
     std::string target_video = argv[1];
     std::string template_file = argv[2];
     std::string labels_file = argv[3];
     int match_method = atoi(argv[4]);
     double threshold_start = atof(argv[5]);
     double threshold_step = atof(argv[6]);
     double threshold_stop = atof(argv[7]);

     // time string generation
     time_t rawtime;
     struct tm * timeinfo;
     char buffer [80];     
     time (&rawtime);
     timeinfo = localtime (&rawtime);     
     strftime (buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
     std::string time_str = std::string(buffer);

     std::string roc_fn;
     unsigned last_slash_pos = target_video.find_last_of("/");
     roc_fn = time_str + "_" + target_video.substr(last_slash_pos+1);
     roc_fn += "_" + std::string(argv[5]) + "-" + std::string(argv[6]) + "-" + std::string(argv[7]) + ".csv";
     cout << "ROC Output file: " << roc_fn << endl;
     
     std::ofstream roc_stream;
     roc_stream.open (roc_fn.c_str(), std::ofstream::out | std::ofstream::app);
     roc_stream << "# Method,Threshold,NumberOfFrames,FP,FN,TP,TN,TPR,FPR,TPRMSE" << endl;
     
     larks::LARKS larks;
     larks.startTraining();

     cv::Mat query;
     //query =
     //cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/fin.png",
     //CV_LOAD_IMAGE_COLOR); query =
     //cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/query.png",
     //CV_LOAD_IMAGE_COLOR);
     query = cv::imread(template_file, CV_LOAD_IMAGE_COLOR);
     //query =
     //cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/fin-2.png",
     //CV_LOAD_IMAGE_COLOR);
 
     if (!query.data) {
          cout <<  "Could not open or find the image" << std::endl;
          return -1;
     }
     int query_area = query.cols * query.rows;
     cv::imshow("query",query);
       
     larks.trainInstance("query",query);
     larks.endTraining("query");

     std::vector<std::string> models;
     models.push_back("query");
     
     larks.loadModels(models);
     
     //cv::Mat target; target =
     //cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/swim-away.png",
     //CV_LOAD_IMAGE_COLOR); target =
     //cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/test-1.png",
     //CV_LOAD_IMAGE_COLOR); if (!target.data) { cout << "Could not open or
     //find the image" << std::endl ; return -1; }
     
     //cv::VideoCapture cap("/home/syllogismrxs/Dropbox/video/target.avi");     
     //cv::VideoCapture cap("/home/syllogismrxs/Desktop/fin-target.avi");
     int target_area;
          
     double threshold = threshold_start; // 0.08
     while (threshold <= threshold_stop) {
          larks.set_threshold(threshold);   

          cv::VideoCapture cap(target_video);
          if(!cap.isOpened()) {
               cout << "Failed to open target video" << endl;
               return -1;
          }   

          // Open the file holding the labeled locations of the scuba diver's face
          //std::ifstream
          //labels("/home/syllogismrxs/repos/opencv-workbench/data/label/target.avi.scuba_face.label");
          std::ifstream labels(labels_file.c_str());
          if (!labels.is_open()) {
               cout << "Failed to open labels file." << endl;
               return -1;
          }

          int next_labeled_frame = 0;
          int x_pos = 0;
          int y_pos = 0;
     
          int frame_num = 0;
          int false_positives = 0;
          int false_negatives = 0;
          int true_positives = 0;

          RMS rms;

          int prev_detect_count = -1;
          for(;;) {

               std::string line;
          
               if(!std::getline(labels,line)) {
                    cout << "End of labels file" << endl;
               } else {
                    char * pch;
                    char * str = new char[line.length()+1];
                    strcpy(str,line.c_str());
                    pch = strtok (str,",");
                    int count = 0;
                    while (pch != NULL){
                         //printf ("%s\n",pch);
                         int num = atoi(pch);
                         switch(count) {
                         case 0:
                              next_labeled_frame = num;
                              break;
                         case 1:
                              x_pos = num;
                              break;
                         case 2:
                              y_pos = num;
                              break;
                         default:
                              cout << "Line error" << endl;
                         }
                         pch = strtok (NULL, ",");
                         count++;
                    }               
               }

               cv::Mat target;
               if (!cap.read(target)) {
                    break;
               }
               target_area = target.cols * target.rows;
                                             
               bool detection = false; // shared
               cv::Point pos_detected; // shared
               cv::Rect detected_rectangle; // shared
          
               if (match_method == LARKS_METHOD) {
                    larks.detect(target);      
                    if (prev_detect_count != larks.model_detections_[0].count && 
                        larks.model_detections_[0].detected) {
                         detection = true;
                         prev_detect_count = larks.model_detections_[0].count;
                         detected_rectangle = larks.model_detections_[0].rect;
                         pos_detected = larks.model_detections_[0].position;
                    }
               } else {                              
                    cv::Mat img_display;
                    target.copyTo( img_display );
               
                    /// Create the result matrix
                    cv::Mat result;
                    int result_cols =  target.cols - query.cols + 1;
                    int result_rows = target.rows - query.rows + 1;

                    result.create( result_cols, result_rows, CV_32FC1 );

                    /// Do the Matching and Normalize
                    cv::matchTemplate( target, query, result, match_method );
                    //cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

                    /// Localizing the best match with minMaxLoc
                    double minVal; 
                    double maxVal; 
                    cv::Point minLoc; 
                    cv::Point maxLoc;
                    cv::Point matchLoc;

                    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
                    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

                    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
                    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ) { 
                         matchLoc = minLoc; 
                         cout << minVal << endl;
                         detection = true;
                         if (minVal < 7.5e+07) {
                              detection = true;
                         } else {
                              detection = false;
                         }
                    } else { 
                         cout << maxVal << endl;
                         matchLoc = maxLoc;
                    
                         if (maxVal > 0.5) {
                              detection = true;
                         } else {
                              detection = false;
                         }
                    }

                    /// Show me what you got
                    cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + query.cols , matchLoc.y + query.rows ), cv::Scalar::all(0), 2, 8, 0 );
                    cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + query.cols , matchLoc.y + query.rows ), cv::Scalar::all(0), 2, 8, 0 );
               
                    detected_rectangle = cv::Rect(matchLoc.x, matchLoc.y, matchLoc.x + query.cols, matchLoc.y + query.rows);

                    cv::imshow( "img_display", img_display );
                    cv::imshow( "result", result );
               
                    pos_detected.x = matchLoc.x + query.cols/2;
                    pos_detected.y = matchLoc.y + query.rows/2;
               }

               if (frame_num == next_labeled_frame) {
                    cv::Point pos_truth(x_pos,y_pos);               
                    cv::circle(target,pos_truth,10,cv::Scalar(0,255,0),1,8,0);               
                              
                    cv::Point diff;
                    if (detection) {
                         cv::circle(target,pos_detected,20,cv::Scalar(255,255,255),1,8,0);

                         diff = pos_truth - pos_detected;
                    
                         if (!pos_truth.inside(detected_rectangle)) {
                              false_positives++;
                         } else {
                              // ground truth falls within detected position
                              // rectangle
                              true_positives++;
                         
                              rms.push(pos_truth, pos_detected);
                         }
                    } else {
                         // Missed a positive label
                         false_negatives++;
                    }
                    cv::imshow("truth",target);

               } else {
                    // If the current frame has a detection and it's not the next
                    // labeled frame, then we have a false positive
                    if (detection) {
                         false_positives++;
                    }
               }

               //if(cv::waitKey(30) >= 0) break;
               frame_num++;               
          } // end of single video
          
          cv::Point2f rms_error = rms.rms_error();
          double rms_error_norm = cv::norm(rms_error);          

          double true_negatives = (double)frame_num * (((double)target_area / (double)query_area) - 1);

          double true_positive_rate = (double)true_positives / ((double)true_positives + (double)false_positives);
          double false_positive_rate = (double)false_positives / ((double)false_positives + (double)true_negatives);

          cout << "==========================================" << endl;          
          cout << "Method: " << match_method << endl;
          cout << "Threshold: " << threshold << endl;
          cout << "False Positives: " << false_positives << endl;
          cout << "False Negatives: " << false_negatives << endl;
          cout << "True Negatives: " << true_negatives << endl;
          cout << "True Positives: " << true_positives << endl;
          cout << "True Positive rate: " << true_positive_rate << endl;
          cout << "False Positive rate: " << false_positive_rate << endl;
          cout << "True Postive RMS Error: " << rms_error_norm << endl;
          
          // Write data to output file
          roc_stream << match_method << "," << threshold << "," << frame_num << false_positives << "," << false_negatives
                     << "," << true_positives << "," << true_negatives << "," << true_positive_rate << "," 
                     << false_positive_rate << "," << rms_error_norm << endl;

          
          threshold += threshold_step;
          
     } // end of multiple threshold tests
     
     roc_stream.close();
     
     return 0;
}
