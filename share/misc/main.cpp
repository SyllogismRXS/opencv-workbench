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
     
     cv::waitKey(0);
     return 0;
}
