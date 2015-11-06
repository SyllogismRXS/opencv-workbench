#include <iostream>
#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/utils/ColorMaps.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input source>" << endl;
          return -1;
     }

     
     syllo::Stream stream;
     syllo::Status status = stream.open(argv[1]);
     
     if (status != syllo::Success) {
          cout << "Failed to open: " << argv[1] << endl;
          return -1;
     }

     cv::Mat original;
     while (stream.read(original)) {
          cv::Mat dst, cdst, gray;
          //cv::Canny(original, dst, 50, 200, 3);
          //cv::imshow("Canny", dst);
          Jet2Gray_matlab(original,gray);
          cv::threshold(gray, dst, 100, 255, cv::THRESH_TOZERO);
          cv::cvtColor(dst, cdst, CV_GRAY2BGR);
          cv::imshow("cvtcolor", cdst);
#if 1
          std::vector<cv::Vec2f> lines;
          cv::HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

          for (size_t i = 0; i < lines.size(); i++ ) {          
               float rho = lines[i][0], theta = lines[i][1];
               cv::Point pt1, pt2;
               double a = cos(theta), b = sin(theta);
               double x0 = a*rho, y0 = b*rho;
               pt1.x = cvRound(x0 + 1000*(-b));
               pt1.y = cvRound(y0 + 1000*(a));
               pt2.x = cvRound(x0 - 1000*(-b));
               pt2.y = cvRound(y0 - 1000*(a));
               cv::line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
          }
          // TODO: Find all lines that are "close" to our "expected" ground
          // plane orientation. "Average" the parameters of these lines and
          // compute a variance parameter around the expected line. Cluster?
#else
          std::vector<cv::Vec4i> lines;
          cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
          for( size_t i = 0; i < lines.size(); i++ ) {
               cv::Vec4i l = lines[i];
               cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
          }
#endif
          cv::imshow("source", original);
          cv::imshow("detected lines", cdst);
          
          if (cv::waitKey(10) == 'q') {
               break;
          }
     }

          return 0;
     }
