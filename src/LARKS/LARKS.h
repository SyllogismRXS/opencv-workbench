#ifndef _LARKS_H_
#define _LARKS_H_

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv_workbench/LARKS/LARKS.h>
#include "opencv2/highgui/highgui.hpp"

#include <boost/multi_array.hpp>
#include <vector>
#include <string>
#include <map>

namespace larks 
{
     using boost::multi_array;
     typedef multi_array<cv::Mat, 4> array_type4;
     typedef multi_array<cv::Mat, 3> array_type3;
     typedef multi_array<cv::Mat, 2> array_type2;
     typedef multi_array<cv::Mat, 1> array_type1;
     typedef multi_array<double, 3> double_type3;
     typedef multi_array<double, 4> double_type4;
     typedef multi_array<cv::Point, 3> Point_type3;
     typedef multi_array<cv::Point, 4> Point_type4;
     typedef multi_array<int,1> int_type1;

     class LARKS
     {
     public:
          void GradientImage(const cv::Mat& gray, cv::Mat& GradImage);
     protected:
     private:
     };

     class Saliency
     {
     public:
          Saliency(){};
          void ProtoObject(const cv::Mat &SaliencyMap, cv::Mat& thMap);
          void computeSaliency(const array_type1& LARK, const int wsize, 
                               const int psize, cv::Mat& SaliencyMap);
          
     };

     class Larkcomputation
     {

     public:
          Larkcomputation(){};
          void computeCovariance(const cv::Mat& gray, int wsize, 
                                 const int dfactor, cv::Mat& sC11, 
                                 cv::Mat& sC12, cv::Mat& sC22);
          
          void computeLARK(const int rows, const int cols, const int wsize, 
                           cv::Mat& sC11, cv::Mat& sC12, cv::Mat& sC22, 
                           array_type1& temp);
     };

     class LARKFeatureTemplates
     {
     public:

          cv::PCA pca;
          array_type1 QF;
          cv::Mat M;
          int cols;
          cv::Mat img;
          cv::Mat mas;
          
          LARKFeatureTemplates() { };

          void computeFeatures(const cv::Mat &whole_gray, const cv::Rect & roi,
                               const cv::Mat & mask,  int index, cv::PCA& pca1, 
                               int cols1);

          float MatrixCosineMeasure(array_type1& QF1, cv::Mat M1, 
                                    array_type1& QF, cv::Mat& M);
     };
}
#endif
