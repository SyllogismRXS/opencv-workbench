#include <iostream>

#include "NDT.h"

using std::cout;
using std::endl;

NDT::NDT()
{
     cell_col_size_ = 10;
     cell_row_size_ = 10;
}

void NDT::set_frame(cv::Mat &src, cv::Mat &dst)
{         
     int rows = src.rows;
     int cols = src.cols;          
     
     // Divide entire region into equal cells.
     int cells_across = cols / cell_col_size_;
     int cells_down = rows / cell_col_size_;
     
     cv::Mat_<double> ndt_total = cv::Mat::zeros(src.size(), CV_64F);

     cells_.resize(4);
     
     int img_index = 0;
     for (int row_shift = 0; row_shift < 2; row_shift++) {
          for (int col_shift = 0; col_shift < 2; col_shift++) {                         

               dst = src.clone();     
               cv::cvtColor(dst, dst, CV_GRAY2BGR);               
               
               cv::Mat_<double> ndt_single = cv::Mat::zeros(src.size(), CV_64F);               
               for (int i = 0; i < cells_across; i++) { //x
                    std::vector<Cell> cells;
                    for (int j = 0; j < cells_down; j++) { //y                         
                         
                         cv::Rect rect = cv::Rect(i*cell_col_size_+col_shift*cell_col_size_/2, j*cell_row_size_+row_shift*cell_row_size_/2, cell_col_size_, cell_row_size_);
                         cv::rectangle(dst, rect, cv::Scalar(255,255,255), 1, 8, 0);                              

                         cv::Rect rect_mat(0, 0, src.cols, src.rows);
                         bool is_inside = (rect & rect_mat) == rect;                         
                         if (!is_inside) {
                              break;
                         }                                                 

                         cv::Mat roi(src,rect);                         

                         // Calculate mean location of points
                         int count = 0;
                         cv::Point2f sum_position(0,0);
                         int sum_value = 0;
                         for(int i = 0; i < roi.rows; i++) {
                              for(int j = 0; j < roi.cols; j++) {               
                                   if (roi.at<uchar>(i,j) != 0) {
                                        sum_position.x += j;
                                        sum_position.y += i;
                                        sum_value += roi.at<uchar>(i,j);
                                        count++;
                                   }
                              }
                         }

                         cv::Point2f mean_position;
                         mean_position.x = sum_position.x / (double)count;
                         mean_position.y = sum_position.y / (double)count;               
               
                         cv::Mat_<double> mean(2,1,CV_64F);
                         mean << mean_position.x, mean_position.y;

                         // Calculate covariance matrix               
                         cv::Mat_<double> sum_covar_position = cv::Mat_<double>::zeros(2,2);
                         for(int i = 0; i < roi.rows; i++) {
                              for(int j = 0; j < roi.cols; j++) {               
                                   if (roi.at<uchar>(i,j) != 0) {
                                        //sum_covar_position.x += pow(j-mean_position.x,2);
                                        //sum_covar_position.y += pow(i-mean_position.y,2);
                                        //sum_covar_
                                        //sum_covar_position += pow();
                                        cv::Mat_<double> point(2,1,CV_64F);
                                        point << j, i;

                                        cv::Mat_<double> diff(2,1,CV_64F);
                                        diff = (point-mean);
                                        cv::Mat_<double> diff_tranpose(1,2,CV_64F);
                                        cv::transpose(diff,diff_tranpose);
                                        sum_covar_position += diff * diff_tranpose;
                                   }
                              }
                         }

                         //cv::Point2f covar_position;
                         //covar_position.x = 1.0 / ((double)count-1.0) * sum_covar_position.x;
                         //covar_position.y = 1.0 / ((double)count-1.0) * sum_covar_position.y;
               
               
                         Cell cell;
                         cell.is_valid_ = false;
                         
                         if (count > 3) {
                              cv::Mat_<double> covar_position(2,2);
                              covar_position = sum_covar_position / ((double)count-1.0);
                              cv::Mat_<double> covar_inverse = covar_position.inv();
                              
                              cell.mean_ = mean;
                              cell.covar_ = covar_position;
                              cell.is_valid_ = true;                                                  
                    
                              //cout << "Mean: (" << mean_position.x << "," << mean_position.y << ")" << endl;
                              //cout << "Covar: (" << covar_position.x << "," << covar_position.y << ")" << endl;
                              //  
                    
                              //cout << "Mean = " << mean << endl;
                              //cout << "Covar = " << covar_position << endl;
                    
                              cv::Mat_<double> dist_img = cv::Mat::zeros(roi.size(), CV_64F);
                              for(int i = 0; i < roi.rows; i++) {
                                   for(int j = 0; j < roi.cols; j++) {               
                                        //double x_val, y_val;
                                        //x_val = 1/(sqrt(2*3.141*covar_position.x)) * exp(-pow(j-mean_position.x,2)/(2*covar_position.x));
                                        //y_val = 1/(sqrt(2*3.141*covar_position.y)) * exp(-pow(i-mean_position.y,2)/(2*covar_position.y));
                                        //dist_img.at<uchar>(i,j) = (x_val + y_val)*255;
                              
                                        cv::Mat_<double> point(2,1,CV_64F);
                                        point << j, i;
                              
                                        cv::Mat_<double> diff(2,1,CV_64F);
                                        diff = (point-mean);
                              
                                        cv::Mat_<double> diff_tranpose(1,2,CV_64F);
                                        cv::transpose(diff,diff_tranpose);
                              
                                        cv::Mat_<double> numerator(1,1);
                                        numerator = diff_tranpose*covar_inverse*diff;

                                        //cout << "numerator = " << numerator << endl;
                                        //cout << "p(x) = " << 255*(exp(-numerator(0,0)/2))  << endl;
                              
                                        dist_img.at<double>(i,j) = exp(-numerator(0,0)/2);
                                   }
                              }                    
                              cv::normalize(dist_img, dist_img, 0, 255, cv::NORM_MINMAX, CV_64F);
                              cv::Mat roi_dst(ndt_single, rect);
                              dist_img.copyTo(roi_dst);                                                           

                              //cv::normalize(dist_img, dist_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                              //cv::imshow("SINGLE NDT",ndt_single);
                    
                              //int scale = 10;
                              //cv::resize(dist_img, dist_img, cv::Size(0,0),scale,scale,cv::INTER_NEAREST);
                              //cv::resize(roi, roi, cv::Size(0,0),scale,scale,cv::INTER_NEAREST);                    
                              //cv::cvtColor(roi,roi,CV_GRAY2BGR);
                              //cv::cvtColor(dist_img,dist_img,CV_GRAY2BGR);
                              //cv::circle(roi, cv::Point(cvRound(mean_position.x*scale), cvRound(mean_position.y*scale)), 1, cv::Scalar(0,255,0), -1, 8, 0);
                              //cv::circle(dist_img, cv::Point(cvRound(mean_position.x*scale), cvRound(mean_position.y*scale)), 1, cv::Scalar(0,255,0), -1, 8, 0);                    
                              //cv::imshow("dist img", dist_img);
                              //cv::imshow("points", roi);
                              //cv::waitKey(0);
                         }
                         cells.push_back(cell);
                    }
                    cells_[img_index].push_back(cells);                    
               }                              
               
               ndt_total += ndt_single;
               //cout << "Row Shift: " << row_shift << endl;
               //cout << "Col Shift: " << col_shift << endl;
               //cv::imshow("GRID",dst);               

               cv::Mat ndt_single_norm;
               cv::normalize(ndt_single, ndt_single_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
               //cv::imshow("SINGLE NDT",ndt_single_norm);               
               //cv::waitKey(0);
               
               img_index++;               
          }
     }     

     cv::Mat ndt_total_norm;
     cv::normalize(ndt_total, ndt_total_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
     cv::imshow("NDT",ndt_total_norm);

     // Compute Jacobian and Hessian
     double phi = 0;
     double x = 0, y = 0;
     cv::Mat_<double> JT(2,3);
     JT << 1, 0, -x*sin(phi)-y*cos(phi),
           0, 1, x*cos(phi)-y*sin(phi);

     cv::Mat_<double> H(3,2);
     
     
     // Save the currently computed cells in the previous cells container
     cells_.swap(prev_cells_);
}
