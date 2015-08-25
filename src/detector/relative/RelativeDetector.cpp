#include <iostream>
#include "RelativeDetector.h"

using std::cout;
using std::endl;

RelativeDetector::RelativeDetector()
{
     cout << "RelativeDetector Constructor" << endl;
}

RelativeDetector::~RelativeDetector()
{
     cout << "RelativeDetector Destructor" << endl;
}

void RelativeDetector::print()
{
     cout << "I am the Relative Detector" << endl;
}

typedef struct Color{
    double r,g,b;
} Color_t;

Color_t GetColor(double v, double vmin, double vmax)
{
   Color_t c = {1.0,1.0,1.0}; // white
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}

int GetGray(Color_t c, double vmin, double vmax)
{
     return 200;
}

int scale(double val)
{
     return val * 255;
}

void Gray2Jet(cv::Mat& I, cv::Mat& jet)
{
     // accept only char type matrices
     CV_Assert(I.depth() != sizeof(uchar));

     int channels = I.channels();

     int nRows = I.rows;
     int nCols = I.cols * channels;

     jet = cv::Mat(I.size(), CV_8UC3);
     
     if (I.isContinuous())
     {
          nCols *= nRows;
          nRows = 1;
     }

     int i,j;
     uchar* p;     
     for( i = 0; i < nRows; ++i) {
          p = I.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               if (p[j] != 0) {
                    Color_t c = GetColor(p[j], 0, 255);
                    jet.at<cv::Vec3b>(i,j) = cv::Vec3b(c.b*255,c.g*255,c.r*255);               
               } else {
                    jet.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
               }
          }
     }     
}

void Jet2Gray(cv::Mat &I, cv::Mat &gray)
{
     gray = cv::Mat(I.size(), CV_8UC1);

     // accept only char type matrices
     CV_Assert(gray.depth() != sizeof(uchar));

     int channels = gray.channels();

     int nRows = gray.rows;
     int nCols = gray.cols * channels;

     if (gray.isContinuous())
     {
          nCols *= nRows;
          nRows = 1;
     }

     int i,j;
     uchar* p;     
     for( i = 0; i < nRows; ++i) {
          p = gray.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               cv::Vec3b pix = I.at<cv::Vec3b>(i,j);
               if (pix[0] == 0 && pix[1] == 0 && pix[2] == 0) {
                    p[j] = 0;
               } else {               
                    Color_t c;
                    c.b = pix[0];
                    c.g = pix[1];
                    c.r = pix[2];                   
                    p[j] = GetGray(c, 0, 255);
               }
          }
     }    
}

//Color_t c;
//c.b = (*it)[0];
//c.g = (*it)[1];
//c.r = (*it)[2];
//int value = GetGray(c, 0, 255);



int RelativeDetector::set_frame(int frame_number, const cv::Mat &original)
{
     cv::Mat original_w_tracks;
     cv::Mat gray;
     cv::Mat median;
     
     cv::imshow("original", original);

     original_w_tracks = original;
     
     cv::cvtColor(original, gray, CV_BGR2GRAY);     
     cv::imshow("gray", gray);

     cv::Mat jet;
     Gray2Jet(gray, jet);
     cv::imshow("my jet", jet);

     cv::Mat back;
     Jet2Gray(jet, back);
     cv::imshow("back", back);

     cv::Mat threshold;
     cv::threshold(gray, threshold, 200, 255, cv::THRESH_TOZERO);
     cv::imshow("thresh", threshold);

     // Compute median
     //cv::medianBlur(gray,median,1);
     
     //// Compute estimated gradient
     //cv::Mat grad_x, grad_y;
     //cv::Mat abs_grad_x, abs_grad_y;
     //cv::Mat grad;
     //cv::Mat grad_thresh;
     //
     //int scale = 1;
     //int delta = 0;
     //int ddepth = CV_16S;
     //
     ///// Gradient X
     //cv::Sobel( median, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
     ///// Gradient Y
     //cv::Sobel( median, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
     //
     //cv::convertScaleAbs( grad_x, abs_grad_x );
     //cv::convertScaleAbs( grad_y, abs_grad_y );
     //
     //cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
     //
     //// Only select gradients above a certain threshold
     //cv::threshold(grad, grad_thresh, 175, 255, cv::THRESH_TOZERO);
     //
     ////// cluster the points
     ////cv::Mat src = grad_thresh;
     ////cv::Mat samples(src.rows * src.cols, 1, CV_32F);
     ////for( int y = 0; y < src.rows; y++ ) {
     ////     for( int x = 0; x < src.cols; x++ ) {
     ////          //for( int z = 0; z < 3; z++) {
     ////          samples.at<float>(y + x*src.rows, 0) = src.at<uchar>(y,x); //src.at<cv::Vec3b>(y,x)[z];
     ////          //}
     ////     }
     ////}
     ////
     ////int clusterCount = 25;
     ////cv::Mat labels;
     ////int attempts = 3;
     ////cv::Mat centers;
     ////cv::kmeans(samples, clusterCount, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers );
     ////
     ////cv::Mat new_image( src.size(), src.type() );
     ////for( int y = 0; y < src.rows; y++ ) {
     ////     for( int x = 0; x < src.cols; x++ ) { 
     ////          int cluster_idx = labels.at<int>(y + x*src.rows,0);
     ////          new_image.at<uchar>(y,x) = centers.at<float>(cluster_idx);
     ////          //new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
     ////          //new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
     ////          //new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
     ////     }
     ////}     
     
     //////////////////////////////////////////////////////////////
     /// Tracking     
     tracks_.clear(); // clear out the tracks from previous loop
     
     ///////////////////////////////////////////////////
     // Display images
     ///////////////////////////////////////////////////
     if (!hide_windows_) {
          //cv::imshow("Original", original);
          //cv::imshow("gray", gray);
          //cv::imshow("median", median);
          //cv::imshow("gradient", grad);
          //cv::imshow("gradient threshold", grad_thresh);         
          //cv::imshow("cluster centers", centers);
     }
     
     return 0;
}

extern "C" {
     Detector *maker(){
          return new RelativeDetector;
     }

     class proxy {
     public:
          proxy(){
               // register the maker with the factory
               // causes static initialization error plugin_manager_.factory["blank_detector"] = maker;
               plugin_manager_.factory["relative_detector"] = maker;
          }
     };
     // our one instance of the proxy
     proxy p;
}
