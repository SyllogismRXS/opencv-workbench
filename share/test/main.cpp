#include <iostream>
#include <opencv_workbench/LARKS/LARKS.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{

     larks::LARKS larks;
     larks.startTraining();

     cv::Mat query;
     //query = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/fin.png", CV_LOAD_IMAGE_COLOR);
     query = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/query.png", CV_LOAD_IMAGE_COLOR);
 
     if (!query.data) {
          cout <<  "Could not open or find the image" << std::endl;
          return -1;
     }
       
     larks.trainInstance("query",query);
     larks.endTraining("query");

     std::vector<std::string> models;
     models.push_back("query");
     
     larks.loadModels(models);
     
     cv::Mat target;
     //target = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/swim-away.png", CV_LOAD_IMAGE_COLOR);
     target = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/test-1.png", CV_LOAD_IMAGE_COLOR);
     
     if (!target.data) {
          cout <<  "Could not open or find the image" << std::endl ;
          return -1;
     }       
     
     larks.detect(target);      
     
     cv::waitKey(0);
     return 0;
}
