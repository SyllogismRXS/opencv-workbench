#ifndef _COLORMAPS_
#define _COLORMAPS_

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef struct Color{
    double r,g,b;
} Color_t;

// Matlab's versions of JET to GRAYSCALE
// Matlab uses the same JET version as BlueView sonar JET
Color_t GetColor_matlab(double v, double vmin, double vmax);
double GetGray_matlab(Color_t c, double vmin, double vmax);
void Jet2Gray_matlab(const cv::Mat &I, cv::Mat &gray);
void Gray2Jet_matlab(cv::Mat& I, cv::Mat& jet);

// "Traditional" JET conversions
Color_t GetColor(double v, double vmin, double vmax);
int GetGray(Color_t c, double vmin, double vmax);
void Gray2Jet(cv::Mat& I, cv::Mat& jet);
void Jet2Gray(const cv::Mat &I, cv::Mat &gray);

void create_gradient(cv::Mat &I, int rows, int cols);
bool equal(cv::Mat& I1, cv::Mat& I2);

#endif
