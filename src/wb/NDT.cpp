#include <iostream>

#include "NDT.h"

using std::cout;
using std::endl;

NDT::NDT()
{
}

void NDT::set_frame(cv::Mat &src, cv::Mat &dst)
{
     dst = src.clone();
     cv::cvtColor(dst, dst, CV_GRAY2BGR);
     
     cell_length_ = 50;

     int rows = src.rows;
     int cols = src.cols;          
     
     // Divide entire region into equal cells.
     int cells_across = cols / cell_length_ + 1;
     int cells_down = rows / cell_length_ + 1;

     cv::copyMakeBorder(dst, dst, cell_length_/2, cell_length_/2, cell_length_/2, cell_length_/2, cv::BORDER_REPLICATE);
     
     cv::rectangle(dst, cv::Rect(cell_length_/2,cell_length_/2,cols,rows),cv::Scalar(255,0,0),1,8,0);
     
     for (int i = 0; i < cells_across; i++) { //x
          for (int j = 0; j < cells_down; j++) { //y
               cv::Rect rect = cv::Rect(i*cell_length_, j*cell_length_, cell_length_, cell_length_);
               cv::rectangle(dst, rect, cv::Scalar(255,255,255), 1, 8, 0);
          }
     }

     
}
