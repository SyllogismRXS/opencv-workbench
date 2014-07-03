#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <opencv_workbench/LARKS/LARKS.h>
#include <opencv_workbench/utils/RMS.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{

     larks::LARKS larks;
     larks.startTraining();

     cv::Mat query;
     //query = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/fin.png", CV_LOAD_IMAGE_COLOR);
     query = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/query.png", CV_LOAD_IMAGE_COLOR);
     //query = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/fin-2.png", CV_LOAD_IMAGE_COLOR);
 
     if (!query.data) {
          cout <<  "Could not open or find the image" << std::endl;
          return -1;
     }
       
     larks.trainInstance("query",query);
     larks.endTraining("query");

     std::vector<std::string> models;
     models.push_back("query");
     
     larks.loadModels(models);
     
     //cv::Mat target;
     //target = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/swim-away.png", CV_LOAD_IMAGE_COLOR);
     //target = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/test-1.png", CV_LOAD_IMAGE_COLOR);
     //if (!target.data) {
     //     cout <<  "Could not open or find the image" << std::endl ;
     //     return -1;
     //}
     
     cv::VideoCapture cap("/home/syllogismrxs/Dropbox/video/target.avi");
     //cv::VideoCapture cap("/home/syllogismrxs/Desktop/fin-target.avi");
     if(!cap.isOpened()) {
          cout << "Failed to open target video" << endl;
          return -1;
     }
         

     // Open the file holding the labeled locations of the scuba diver's face
     std::ifstream labels("/home/syllogismrxs/repos/opencv-workbench/data/label/target.avi.scuba_face.label");
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
          larks.detect(target);      

          bool detection = false;
          if (prev_detect_count != larks.model_detections_[0].count && larks.model_detections_[0].detected) {
               detection = true;
               prev_detect_count = larks.model_detections_[0].count;
          }

          if (frame_num == next_labeled_frame) {
               cv::Point pos_truth(x_pos,y_pos);               
               cv::circle(target,pos_truth,10,cv::Scalar(0,255,0),1,8,0);
               cv::imshow("truth",target);
               
               cv::Point pos_detected;
               cv::Point diff;
               double L2_norm;
               if (detection) {
                    pos_detected = larks.model_detections_[0].position;
                    diff = pos_truth - pos_detected;
                    L2_norm = cv::norm(diff);
                                        
                    if (!pos_truth.inside(larks.model_detections_[0].rect)) {
                         false_positives++;
                    } else {
                         // ground truth falls within detected position rectangle
                         true_positives++;
                         
                         //rms.push(pos_truth, );
                    }
               } else {
                    // Missed a positive label
                    false_negatives++;
               }

          } else {
               // If the current frame has a detection and it's not the next
               // labeled frame, then we have a false positive
               if (detection) {
                    false_positives++;
               }
          }

          if(cv::waitKey(30) >= 0) break;          

          frame_num++;          
     }               
     
     cout << "==========================================" << endl;
     cout << "False Positives: " << false_positives << endl;
     cout << "False Negatives: " << false_negatives << endl;
     cout << "True Positives: " << true_positives << endl;
     cout << "True Postive RMS Error: " << 99 << endl;

     return 0;
}
