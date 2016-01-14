#include <iostream>
#include <vector>

#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/wb/NDT.h>

using std::cout;
using std::endl;

int num_segments(int nums, int cell_size, int shift)
{
     int even = 0;
     if (nums % cell_size == 0) {
          even = 1;
     }
     
     //int result = cvRound((double)nums/(double)cell_size + 0.000001);
     //int result = kevRound((double)nums/(double)cell_size); // NOT WORKING!
     int result = ceil((double)nums/(double)cell_size);
     return result + shift*even;     
}

bool contains(cv::Rect rect, cv::Point p)
{
     if (p.x >= rect.x && p.y >= rect.y && 
         p.x <= (rect.x+rect.width) && p.y <= (rect.y+rect.height)) {
          return true;
     }
     return false;
}

int main(int argc, char *argv[])
{

     // Cells must be divisible by 2
     
     int col_side = 200;
     int row_side = 301;          
     int cell_col_size_ = 10;
     int cell_row_size_ = 10;

     //int col_side = 199;
     //int row_side = 301;
     //int cell_col_size_ = 10;
     //int cell_row_size_ = 10;
     
     //int col_side = 50;
     //int row_side = 56;          
     //int cell_col_size_ = 10;
     //int cell_row_size_ = 10;     
     
     //int cell_col_size_ = 10;
     //int cell_row_size_ = 10;
     //int side = 50;              
     
     cv::Mat src = cv::Mat::zeros(row_side, col_side,CV_8UC1);
     src.at<uchar>(5,5) = 255;
     
     cv::Mat cell_img = cv::Mat::zeros(row_side, col_side,CV_8UC1);
     cv::Mat src_copy = src.clone();
     
     int rows = src.rows;
     int cols = src.cols;         

     std::vector< std::vector< std::vector<Cell> > > cells_;
     cells_.clear();
     cells_.resize(4);     
     
     int img_index = 0;     
     // Cycle through the four overlapping overlays     
     for (int row_shift = 0; row_shift < 2; row_shift++) {                    
          for (int col_shift = 0; col_shift < 2; col_shift++) {
               int num_cell_rows = num_segments(rows, cell_row_size_, row_shift);
               int num_cell_cols = num_segments(cols, cell_col_size_, col_shift);
               
               //cout << "img_index: " << img_index << endl;
               
               cell_img = cv::Mat::zeros(row_side, col_side,CV_8UC1);
               src_copy = src.clone();
               cv::Mat_<double> ndt_single = cv::Mat::zeros(src_copy.size(), CV_64F);                              
               
               //cout << "Resizing: " << num_cell_rows << endl;
               cells_[img_index].resize(num_cell_rows);
               int r_cell = 0;
               int r_idx = -row_shift*cell_row_size_/2.0;               
               //while (r_idx < src_copy.rows) { 
               while (r_cell < num_cell_rows) { 
                    
                    //cout << "--------" << endl;
                    //cout << "img_index: " << img_index << endl;
                    //cout << "num_cell_rows: " << num_cell_rows << endl;
                    //cout << "num_cell_cols: " << num_cell_cols << endl;
                    //cout << "r_cell:" << r_cell << endl;
                    //cout << "r_idx: " << r_idx << endl;
                    //cout << "rows: " << src_copy.rows << endl;
                    cells_[img_index][r_cell].resize(num_cell_cols);                    
                    
                    cv::Rect rect;
                    int c_idx = -col_shift*cell_col_size_/2.0;                    
                    int c_cell = 0;
                    //while (c_idx < src_copy.cols) {                         
                    while (c_cell < num_cell_cols) { 
                         rect = cv::Rect(c_idx, r_idx, cell_col_size_, cell_row_size_);
                         
                         if ( (rect.x + rect.width) > src_copy.cols) {
                              rect.width = src_copy.cols - rect.x;
                         }
               
                         if ( (rect.y + rect.height) > src_copy.rows) {
                              rect.height = src_copy.rows - rect.y;
                         }
               
                         if (rect.x < 0) {
                              rect.x = 0;
                              //rect.width = kevRound((double)cell_col_size_ / 2.0);
                              //rect.width = ceil((double)cell_col_size_ / 2.0);
                              rect.width = (double)cell_col_size_ / 2.0;
                         }
               
                         if (rect.y < 0) {
                              rect.y = 0;
                              //rect.height = kevRound((double)cell_row_size_ / 2.0);
                              rect.height = (double)cell_row_size_ / 2.0;
                         }
                         
                         cv::Rect rect_mat(0, 0, src_copy.cols, src_copy.rows);
                         bool is_inside = (rect & rect_mat) == rect;                         
                         if (!is_inside) {

                              cout << "img_index: " << img_index << endl;
                              cout << r_idx << "," << c_idx << endl;
                              cout << "rect: (" << rect.x << "," << rect.y << ") to ";
                              cout << "(" << rect.x+rect.width << "," << rect.y+rect.height << ")" << endl;
                              std::string tmp;
                              std::cin >> tmp;
                              
                              cout << "Not inside" << endl;
                              break;
                              //continue;
                         }                         
                                                  
                         //cout << "num_cell_rows: " << num_cell_rows << endl;
                         //cout << "num_cell_cols: " << num_cell_cols << endl;
                         //cout << "Rect: (" << rect.x << "," << rect.y << ") to ";
                         //cout << "(" << rect.x+rect.width << "," << rect.y+rect.height << ")" << endl;
                         
                         cv::rectangle(cell_img, rect, cv::Scalar(255,255,255), 1, 8, 0);                                                  
                         //cv::Mat roi(src_copy,rect);
                                                 
                         Cell cell;
                         cell.is_valid_ = false;
                         cell.rect_ = rect;
                         cells_[img_index][r_cell][c_cell] = cell;
                         c_cell++;                         
                         c_idx = rect.x + rect.width;
                    }
                    r_cell++;
                    r_idx = rect.y + rect.height;
               }               
               std::string cell_str = "cells" + syllo::int2str(img_index);
               //cv::imshow(cell_str, cell_img);
               //cv::waitKey(0);               
               img_index++;
          } // col_shift                   
     } // row_shift     

#if 1
     // Given a (row,col), can we find the appropriate cell in each of the four
     // overlapping grids?
     
     int errors = 0;
     for (int r = 0 ; r < src_copy.rows; r++) {
          for (int c = 0; c < src_copy.cols; c++) {               
               int find_row = r;
               int find_col = c;     
     
               img_index = 0;
               for (int row_shift = 0; row_shift < 2; row_shift++) {                    
                    for (int col_shift = 0; col_shift < 2; col_shift++) {                                                  
                         unsigned int r_map = floor(((double)find_row + (double)row_shift*(double)cell_row_size_/2.0) / (double)cell_row_size_);
                         unsigned int c_map = floor(((double)find_col + (double)col_shift*(double)cell_col_size_/2.0) / (double)cell_col_size_);                         
     
                         if (r_map < cells_[img_index].size() && c_map < cells_[img_index][r_map].size()) {
                              cv::Rect rect = cells_[img_index][r_map][c_map].rect_;                                                                           
                              
                              if (!contains(rect, cv::Point(find_col, find_row))) {
                                   cout << "---------------------" << endl;
                                   cout << "DOESN'T CONTAIN" << endl;
                                   cout << "Rows: " << src_copy.rows << endl;
                                   cout << "Cols: " << src_copy.cols << endl;
                                   cout << "find_col: " << find_col << endl;
                                   cout << "find_row: " << find_row << endl;
                                   cout << "r_map: " << r_map << endl;
                                   cout << "c_map: " << c_map << endl;
                                   cout << "Img: " << img_index;
                                   cout << " , rect: (" << rect.x << "," << rect.y << ") to ";
                                   cout << "(" << rect.x+rect.width << "," << rect.y+rect.height << ")" << endl;
                                   errors++;
                                   std::string str;
                                   std::cin >> str;
                              }                                                       
                         } else {                              
                              //cout << "------" << endl;
                              //cout << "RMAP / CMAP OUT OF BOUNDS" << endl;
                              //cout << "Img: " << img_index << endl;
                              //cout << "Rows: " << src_copy.rows << endl;
                              //cout << "Cols: " << src_copy.cols << endl;
                              //cout << "Row shift: " << row_shift << endl;
                              //cout << "Col shift: " << col_shift << endl;
                              //cout << "col_even: " << col_even << endl;
                              //cout << "row_even: " << row_even << endl;
                              //cout << "find_col: " << find_col << endl;
                              //cout << "find_row: " << find_row << endl;
                              //cout << "rmap / cmap out" << endl;
                              //cout << "rmap: " << r_map << endl;
                              //cout << "cmap: " << c_map << endl;
                              //std::string str;
                              //std::cin >> str;
                         }                                                  
                         img_index++;
                    }          
               }
          }
     }
     
     if (errors == 0) {
          cout << "No errors" << endl;
     } else {
          cout << "Errors: " << errors << endl;
     }   
#endif          
     return 0;
}
