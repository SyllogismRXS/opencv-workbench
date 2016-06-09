#include <iostream>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/wb/BlobProcess.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     if (argc < 1) {
          cout << "usage: " << argv[0] << " image-path" << endl;
          return -1;
     }
     
     cv::Mat img;
     img = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
     if (!img.data) {
          cout << "Failed to open image: " << argv[1] << endl;
          return -1;
     }

     cv::imshow("Input", img);

     cv::Mat gray;
     cv::cvtColor(img, gray, CV_BGR2GRAY);     
          
     std::vector<wb::Blob> blobs;
     int min_blob_size = 1;

     BlobProcess blob_process;
     cv::Mat frame_blob_img;
     blob_process.find_blobs(gray, frame_blob_img, blobs, min_blob_size, false);          

     cv::Mat blob_img;
     blob_process.overlay(blobs, gray, blob_img, BLOBS | IDS | RECTS);     
     
     // Resize gray and blob image and display
     cv::resize(gray, gray, cv::Size(0,0), 5, 5, cv::INTER_NEAREST);     
     cv::imshow("gray", gray);     
     cv::resize(blob_img, blob_img, cv::Size(0,0), 5, 5, cv::INTER_NEAREST);
     cv::imshow("blobs", blob_img);         

     cv::waitKey(0);
     
     return 0;
}
