#include <iostream>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/wb/BlobProcess.h>
#include <opencv_workbench/utils/ColorMaps.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     
     cv::Mat gray;
     create_gradient(gray, 50,255);
     cv::resize(gray, gray, cv::Size(0,0),2,2,cv::INTER_NEAREST);
     cv::imshow("Gradient", gray);

     cv::Mat jet_matlab;
     Gray2Jet_matlab(gray,jet_matlab);
     cv::imshow("Jet Matlab", jet_matlab);

     cv::Mat jet_opencv;
     //Gray2Jet(gray,jet_opencv);
     cv::applyColorMap(gray, jet_opencv, cv::COLORMAP_JET);
     cv::imshow("Jet Opencv", jet_opencv);

     cv::Mat jet_2_gray;
     Jet2Gray_matlab(jet_matlab,jet_2_gray);
     cv::imshow("Jet to Gray Test",jet_2_gray);
     
     // Check
     for (int r = 0; r < gray.rows; r++) {
          for (int c = 0; c < gray.cols; c++) {
               if (gray.at<uchar>(r,c) != jet_2_gray.at<uchar>(r,c))  {
                    cout << "Mismatched grays" << endl;
               }
          }
     }
     
     cv::waitKey(0);
     return 0;
}
