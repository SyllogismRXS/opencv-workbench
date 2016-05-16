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
     
     void arrowedLine(cv::Mat & img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color,
                      int thickness, int line_type, int shift, double tipLength)
     {
          const double tipSize = cv::norm(pt1-pt2)*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow

          cv::line(img, pt1, pt2, color, thickness, line_type, shift);

          const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );

          cv::Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
                      cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
          cv::line(img, p, pt2, color, thickness, line_type, shift);

          p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
          p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
          cv::line(img, p, pt2, color, thickness, line_type, shift);
     }

     void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color, int d)
     {
          cv::line(img, cv::Point(center.x - d, center.y - d), cv::Point(center.x + d, center.y + d), color, 1, CV_AA, 0);
          cv::line(img, cv::Point(center.x + d, center.y - d), cv::Point(center.x - d, center.y + d), color, 1, CV_AA, 0 );
     }

     void drawTriangle(cv::Mat &img, cv::Point &center, cv::Scalar color, int size)
     {
          int s = cvRound((double)size * sqrt(3.0) / 2.0);
          int d = cvRound(0.5 * (double)size);
          
          cv::Point pt1(center.x, center.y - size);
          cv::Point pt2(center.x+s, center.y+d);
          cv::Point pt3(center.x-s, center.y+d);
          
          cv::line(img, pt1, pt2, color, 1, CV_AA, 0);
          cv::line(img, pt2, pt3, color, 1, CV_AA, 0);
          cv::line(img, pt3, pt1, color, 1, CV_AA, 0);
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
