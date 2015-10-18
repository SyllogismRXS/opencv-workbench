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

     void show_nonzero(cv::Mat &img)
     {
          // accept only char type matrices
          CV_Assert(img.depth() != sizeof(uchar));

          cv::Mat img_show = cv::Mat::zeros(img.size(), CV_8UC3);

          //int channels = img.channels();

          int nRows = img.rows;
          int nCols = img.cols;// * channels;

          //if (img.isContinuous()) {
          //     nCols *= nRows;
          //     nRows = 1;
          //}

          int i,j;
          for( i = 0; i < nRows; ++i) {
               for ( j = 0; j < nCols; ++j) {
                    cv::Vec3b pix = img.at<cv::Vec3b>(i,j);
                    if (pix[0] == 0 && pix[1] == 0 && pix[2] == 0) {
                         img_show.at<cv::Vec3b>(i,j) = cv::Vec3b(255,0,0);
                    } else {
                         img_show.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,255);
                    }
               }
          }    
          cv::imshow("nonzeros", img_show);
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
