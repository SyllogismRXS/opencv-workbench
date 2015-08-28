#include "OpenCV_Helpers.h"

namespace syllo{

     void adaptive_threshold(cv::Mat &src, cv::Mat& dst, int &thresh, double ratio_low, double ratio_high, int thresh_step, int max_iter)
{
     // accept only char type matrices
     CV_Assert(src.depth() != sizeof(uchar));        
     bool ratio_achieved = false;     
     int iter_count = 0;
     //cout << "--------" << endl;
     do {     
          //cout << "Thresh: " << thresh << endl;
          cv::threshold(src, dst, thresh, 255, cv::THRESH_TOZERO);
          
          int channels = dst.channels();
          int nRows = dst.rows;
          int nCols = dst.cols * channels;

          if (dst.isContinuous())
          {
               nCols *= nRows;
               nRows = 1;
          }

          int count = 0;

          int i,j;
          uchar* p;               
          for( i = 0; i < nRows; ++i) {
               p = dst.ptr<uchar>(i);
               for ( j = 0; j < nCols; ++j) {
                    if (p[j] > 0) {
                         count++;
                    }
               }
          }

          // Use control systems for adaptive threshold?
          double ratio = (double)count / ((double)(nRows*nCols));
          //cout << "Ratio: " << ratio << endl;
          if (ratio > ratio_low && ratio < ratio_high) {
               ratio_achieved = true;
          } else if (ratio < ratio_low) {
               thresh -= thresh_step;
          } else if (ratio > ratio_high) {
               thresh += thresh_step;
          }
          iter_count++;
          
     }while(!ratio_achieved && iter_count < max_iter);
}

     void gradient_sobel(cv::Mat &src, cv::Mat &dst)
     {
          cv::Mat grad_x, grad_y;
          cv::Mat abs_grad_x, abs_grad_y;
          cv::Mat grad;
          
          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;
     
          /// Gradient X
          cv::Sobel( src, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
          /// Gradient Y
          cv::Sobel( src, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
     
          cv::convertScaleAbs( grad_x, abs_grad_x );
          cv::convertScaleAbs( grad_y, abs_grad_y );
     
          cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst );
     }

}
