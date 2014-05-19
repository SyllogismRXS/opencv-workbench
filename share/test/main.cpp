#include <iostream>
#include <opencv_workbench/LARKS/LARKS.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{

     cv::Mat image;
     //image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
     image = cv::imread("/home/syllogismrxs/repos/opencv-workbench/data/images/test-1.png", CV_LOAD_IMAGE_COLOR);

     if (!image.data) {
          cout <<  "Could not open or find the image" << std::endl ;
          return -1;
     }

     cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
     cv::imshow( "Display window", image );

     cv::Mat gray;
     cv::cvtColor(image,gray,CV_RGB2GRAY);
         
     larks::LARKS larks;
     cv::Mat gradImg;
     larks.GradientImage(gray,gradImg);

     cv::namedWindow("Gradient", cv::WINDOW_AUTOSIZE);
     cv::imshow("Gradient", gradImg);

     cv::waitKey(0);                       

     return 0;
}
