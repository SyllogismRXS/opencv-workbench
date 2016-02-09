#include <stdio.h>      /* printf */
#include <math.h> 

#include "ColorMaps.h"

using std::cout;
using std::endl;

Color_t GetColor_matlab(double v, double vmin, double vmax)
{
     Color_t c = {255.0,255.0,255.0}; // white
     double dv;

     if (v < vmin) {
          v = vmin;
     } else if (v > vmax) {
          v = vmax;
     }
     
     dv = vmax - vmin;

     double mx = v*0.5*vmax / (0.125*vmax - vmin);
     
     if (v < (vmin + 0.125 * dv)) {
          c.r = vmin;
          c.g = vmin;
          c.b = mx + 0.5*vmax;
     } else if (v < (vmin + 0.375 * dv)) {
          c.r = vmin;
          c.g = mx - 0.5*vmax;
          c.b = vmax;
     } else if (v < (vmin + 0.625 * dv)) {
          c.r = mx - 1.5*vmax;
          c.g = vmax;
          c.b = -mx + (vmax - 1.5*(vmin-vmax));
     } else if (v < (vmin + 0.875 * dv)) {
          c.r = vmax;
          c.g = -mx + (vmax - 2.5*(vmin-vmax));
          c.b = vmin;
     } else {
          c.r = -mx + 4.5*vmax;
          c.g = vmin;
          c.b = vmin;
     }     
     return(c);
}

double GetGray_matlab(Color_t c, double vmin, double vmax)
{
     double v = 0;
     //double slope = 0.5*vmax / (0.125*vmax - vmin);
     //double slope = (vmax - 0.5*(vmax-vmin)) / (0.125*(vmax-vmin)-vmin);
     //double slope = 0.5*(vmax-vmin) / (0.125*(vmax-vmin)-vmin);
     //double slope = 0.5*(vmax-vmin) / (0.125*vmax - 1.125*vmin);
     double slope = 4;
     
     if (c.g == vmin && c.r == vmin) {     
          //v = (c.b - 0.5*vmax) / slope;
          v = (c.b - 0.5*(vmax-vmin)) / slope;
     } else if (c.b == vmax && c.r == vmin) {
          //v = (c.g + 0.5*vmax) / slope;          
          v = (c.g + 0.5*(vmax-vmin) - vmin) / slope;
     } else if (c.g == vmax) {
          v = (c.r + 1.5*(vmax-vmin) - vmin) / slope;
          //v = (c.r - vmax + 5.0/2.0*(vmax-vmin)) / slope;
     } else if (c.b == vmin && c.r == vmax) {
          //v = (c.g - ((vmax - 2.5*(vmin-vmax)))) / (-slope);
          v = (c.g - 3.5*(vmax-vmin) - vmin) / -slope;
     } else if (c.g == vmin && c.b == vmin) {
          //v = (c.r - 4.5*vmax) / (-slope);
          v = (c.r - 4.5*(vmax) + 0.5*vmin) / -slope;
     }
     return v;
}

void Gray2Jet_matlab(cv::Mat& I, cv::Mat& jet)
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
               //if (p[j] != 0) {
               Color_t c = GetColor_matlab(p[j], 0, 255);
               jet.at<cv::Vec3b>(i,j) = cv::Vec3b(round(c.b),round(c.g),
                                                  round(c.r));               
               //} else {
               //     cout << "Found a zero" << endl;
               //     jet.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
               //}
          }
     }     
}

void Jet2Gray_matlab(const cv::Mat &I, cv::Mat &gray)
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

#define SHOW_MASK 0
#if SHOW_MASK
     cv::Mat mask = cv::Mat::zeros(I.size(), CV_8UC3);
#endif     

     int i,j;
     uchar* p;     
     for( i = 0; i < nRows; ++i) {
          p = gray.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               cv::Vec3b pix = I.at<cv::Vec3b>(i,j);
               if (pix[0] == 0 && pix[1] == 0 && pix[2] == 0) {
                    p[j] = 0;
#if SHOW_MASK
                    mask.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,255);
#endif
               } else {               
#if SHOW_MASK
                    mask.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
#endif
                    Color_t c;
                    c.b = pix[0];
                    c.g = pix[1];
                    c.r = pix[2];                   
                    p[j] = round(GetGray_matlab(c, 0, 255));

#if SHOW_MASK
                    if (p[j] == 0) {
                         mask.at<cv::Vec3b>(i,j) = cv::Vec3b(0,255,0);
                    }
#endif
               }
          }
     }    

#if SHOW_MASK
     cv::imshow("conversion-zeros", mask);
#endif
}

Color_t GetColor(double v, double vmin, double vmax)
{
     Color_t c = {255.0,255.0,255.0}; // white
     //Color_t c = {1.0,1.0,1.0}; // white
     double dv;

     if (v < vmin) {
          v = vmin;
     } else if (v > vmax) {
          v = vmax;
     }
     
     dv = vmax - vmin;

     if (v < (vmin + 0.25 * dv)) {
          c.r = vmin;
          c.g = 4.0*v*(vmax-vmin)/vmax;
          c.b = vmax;
     } else if (v < (vmin + 0.5 * dv)) {
          c.r = vmin;
          c.g = vmax;          
          c.b = 4.0*v*(vmin-vmax)/vmax-2*(vmin-vmax);
     } else if (v < (vmin + 0.75 * dv)) {
          c.r = 4.0*v*(vmax-vmin)/vmax-2*(vmax-vmin);
          c.g = vmax;
          c.b = vmin;          
     } else {
          c.r = vmax;
          c.g = 4.0*v*(vmin-vmax)/vmax-4.0*(vmin-vmax);
          c.b = vmin;
     }

     return(c);
}


int GetGray(Color_t c, double vmin, double vmax)
{
     int v = 0;
     
     if (c.r == vmin && c.b == vmax) {
          v = c.g * vmax / (4.0*(vmax-vmin));
     } else if (c.r == vmin && c.g == vmax) {
          v = (c.b + 2*(vmin-vmax)) / (4.0*(vmin-vmax)/vmax);
     } else if (c.g == vmax && c.b == vmin) {
          v = (c.r + 2*(vmax-vmin)) / (4.0*(vmax-vmin)/vmax);
     } else {
          v = (c.g + 4.0*(vmin-vmax)) / (4.0*(vmin-vmax)/vmax);
     }
     return v;
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
                    jet.at<cv::Vec3b>(i,j) = cv::Vec3b(c.b,c.g,c.r);               
               } else {
                    jet.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
               }
          }
     }     
}

void Jet2Gray(const cv::Mat &I, cv::Mat &gray)
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


void create_gradient(cv::Mat &I, int rows, int cols)
{
     I = cv::Mat(rows, cols, CV_8UC1);

     int channels = I.channels();

     int nRows = I.rows;
     int nCols = I.cols * channels;
     
     //if (I.isContinuous())
     //{
     //     nCols *= nRows;
     //     nRows = 1;
     //}

     int i,j;
     uchar* p;     
     for( i = 0; i < nRows; ++i) {
          int value = 0;
          p = I.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               p[j] = value++;
          }          
     }    
     
     //cv::resize(I, I, cv::Size(0,0),3,3,cv::INTER_LINEAR);
}


bool equal(cv::Mat& I1, cv::Mat& I2)
{
     // accept only char type matrices
     CV_Assert(I1.depth() != sizeof(uchar));

     if (I1.rows != I2.rows || I1.cols != I2.cols || 
         I1.channels() != I2.channels()) {
          return false;
     }

     int channels = I1.channels();

     int nRows = I1.rows;
     int nCols = I1.cols * channels;     

     int i,j;
     uchar *p1;
     uchar *p2;     
     for( i = 0; i < nRows; ++i) {
          p1 = I1.ptr<uchar>(i);
          p2 = I2.ptr<uchar>(i);
          for ( j = 0; j < nCols; ++j) {
               if (p1[j] != p2[j]) {
                    printf("[%d][%d] : %d - %d\n", i, j, p1[j], p2[j]);
                    return false;
               }
          }     
     }
     return true;
}

//Color_t GetColor_2(double v, double vmin, double vmax)
//{
//     Color_t c = {1.0,1.0,1.0}; // white
//     double dv;
//
//     if (v < vmin)
//          v = vmin;
//     if (v > vmax)
//          v = vmax;
//     dv = vmax - vmin;
//
//     if (v < (vmin + 0.25 * dv)) {
//          c.r = 0;
//          c.g = 4 * (v - vmin) / dv;
//     } else if (v < (vmin + 0.5 * dv)) {
//          c.r = 0;
//          c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
//     } else if (v < (vmin + 0.75 * dv)) {
//          c.r = 4 * (v - vmin - 0.5 * dv) / dv;
//          c.b = 0;
//     } else {
//          c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
//          c.b = 0;
//     }
//
//     return(c);
//}

//void Gray2Jet_2(cv::Mat& I, cv::Mat& jet)
//{
//     // accept only char type matrices
//     CV_Assert(I.depth() != sizeof(uchar));
//
//     int channels = I.channels();
//
//     int nRows = I.rows;
//     int nCols = I.cols * channels;
//
//     jet = cv::Mat(I.size(), CV_8UC3);
//     
//     if (I.isContinuous())
//     {
//          nCols *= nRows;
//          nRows = 1;
//     }
//
//     int i,j;
//     uchar* p;     
//     for( i = 0; i < nRows; ++i) {
//          p = I.ptr<uchar>(i);
//          for ( j = 0; j < nCols; ++j) {
//               if (p[j] != 0) {
//                    Color_t c = GetColor_2(p[j], 0, 255);
//                    jet.at<cv::Vec3b>(i,j) = cv::Vec3b(c.b*255,c.g*255,c.r*255);               
//               } else {
//                    jet.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
//               }
//          }
//     }     
//}
