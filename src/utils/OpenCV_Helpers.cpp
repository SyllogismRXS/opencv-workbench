#include "OpenCV_Helpers.h"

namespace wb{

     void show(const cv::Mat &img)
     {
          cv::imshow("DEBUG!",img);
          cv::waitKey(0);
     }

     void print_size(const cv::Mat &img)
     {
          //cout << img.rows << "x" < img.cols << endl;
          printf("%dx%d\n",img.rows,img.cols);
     }

     void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color, int d)
     {
          cv::line(img, cv::Point(center.x - d, center.y - d), cv::Point(center.x + d, center.y + d), color, 2, CV_AA, 0);
          cv::line(img, cv::Point(center.x + d, center.y - d), cv::Point(center.x - d, center.y + d), color, 2, CV_AA, 0 );
     }

     ////struct Cluster_t;
     //class Cluster;
     //typedef Cluster Cluster_t;
     //
     //typedef struct Point {
     //     cv::Point point;
     //     int value;
     //     bool assigned;
     //     float distance;
     //     Cluster_t *parent;
     //} Point_t;
     //
     //float distance(wb::Point_t p1, wb::Point_t p2)
     //{
     //     return sqrt( pow(p1.point.x-p2.point.x,2) + pow(p1.point.y-p2.point.y,2) );
     //}
     //
     //typedef struct Cluster {
     //     std::vector<wb::Point*> points;
     //} Cluster_t;     
     
     
}
