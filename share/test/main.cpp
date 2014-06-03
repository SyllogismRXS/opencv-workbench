#include <iostream>
#include <opencv_workbench/LARKS/LARKS.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{

     cv::Mat query;
     query = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/query.png", CV_LOAD_IMAGE_COLOR);

     if (!query.data) {
          cout <<  "Could not open or find the image" << std::endl ;
          return -1;
     }


     cv::Mat fin_query;
     fin_query = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/fin.png", CV_LOAD_IMAGE_COLOR);

     if (!fin_query.data) {
          cout <<  "Could not open or find the image" << std::endl ;
          return -1;
     }

     larks::LARKS larks;
     larks.startTraining();
     larks.trainInstance("scuba_face",query);
     larks.endTraining("scuba_face");
     
     larks.trainInstance("fin",fin_query);
     larks.endTraining("fin");

     larks.loadModels();

     cv::Mat swim_away;
     swim_away = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/swim-away.png", CV_LOAD_IMAGE_COLOR);

     if (!swim_away.data) {
          cout <<  "Could not open or find the image" << std::endl ;
          return -1;
     }    
     
     larks.detect(swim_away);

///     cv::Mat img_gray;
///     cv::cvtColor(query,img_gray,CV_RGB2GRAY);
///     cv::Rect roi(0,0,img_gray.cols, img_gray.rows);
///     cv::Mat mask = cv::Mat::ones(img_gray.rows, img_gray.cols, CV_32F);
///
///     larks::LARKFeatureTemplates crt_tpl;
///     crt_tpl.computeFeatures(img_gray,roi,mask, 0, crt_tpl.pca, crt_tpl.cols);
///
///     
///     cv::Mat target;
///     target = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/test-1.png", CV_LOAD_IMAGE_COLOR);
///     
///     if (!target.data) {
///          cout <<  "Could not open or find the image" << std::endl ;
///          return -1;
///     }
///
///     cv::imshow("Target", target);
///     cv::Mat img_gray2;
///     cv::cvtColor(target, img_gray2,CV_RGB2GRAY);
///     cv::Rect roi2(0,0,img_gray2.cols,img_gray2.rows);
///     cv::Mat mask2 = cv::Mat::ones(img_gray2.rows, img_gray2.cols, CV_32F);
///     
///     larks::LARKFeatureTemplates crt_tpl2;
///     crt_tpl2.computeFeatures(img_gray2,roi2,mask2, 0, crt_tpl2.pca, crt_tpl2.cols);
///     
///     //larks::LARKS larks;
///     //
///     //std::vector<std::string> models;
///     //models.push_back("face");
///     //std::vector<double> thresholds;
///     //thresholds.push_back(0.7);
///     //
///     //larks.set_num_models(models.size());
///     //larks.set_models(models, models.size());
///     //larks.set_threshold(thresholds, thresholds.size());
///     //
///     //larks.detect(target);     
     
     cv::waitKey(0);                       
     return 0;
}
