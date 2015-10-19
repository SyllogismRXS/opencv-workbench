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
     cv::imshow("gray", gray);
          
     std::vector<wb::Blob> blobs;
     int min_blob_size = 30;

     BlobProcess blob_process;
     blob_process.find_blobs(gray, blobs, min_blob_size);     

     //cv::SimpleBlobDetector::Params params; 
     //params.minDistBetweenBlobs = 2.0;  // minimum 10 pixels between blobs
     //params.filterByArea = true;         // filter my blobs by area of blob
     //params.minArea = 5.0;              // min 20 pixels squared
     //params.maxArea = 500.0;             // max 500 pixels squared
     //cv::SimpleBlobDetector myBlobDetector(params);
     //std::vector<cv::KeyPoint> myBlobs;
     //myBlobDetector.detect(img, myBlobs);
     //
     //cv::Mat blobImg;    
     //cv::drawKeypoints(img, myBlobs, blobImg);
     //cv::imshow("CV::Blobs", blobImg);

     cv::waitKey(0);
     
     return 0;
}
