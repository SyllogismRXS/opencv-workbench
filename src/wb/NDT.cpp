#include <iostream>

#include "NDT.h"
#include <opencv_workbench/syllo/syllo.h>

using std::cout;
using std::endl;

#define DEBUG 0

NDT::NDT()
{
     cell_col_size_ = 20;//20;//10;
     cell_row_size_ = 20;//20;//10;    
     
     x_est_ = 1;
     y_est_ = 1;
     phi_est_ = 3.14159265359/2;
}

Eigen::MatrixXd T_map(Eigen::MatrixXd &pos, Eigen::MatrixXd &p)
{
     double tx = p(0,0);
     double ty = p(1,0);
     double phi = p(2,0);
     
     Eigen::MatrixXd image(2,1);     
     
     Eigen::MatrixXd rot(2,2);
     rot << cos(phi), -sin(phi), sin(phi), cos(phi);
     
     Eigen::MatrixXd translate(2,1);
     //translate << tx , ty;
     translate << tx, ty;
     
     image = (rot * pos) + translate;
     
     return image;
}

int num_segments(int nums, int cell_size, int shift)
{
     int even = 0;
     if (nums % cell_size == 0) {
          even = 1;
     }
     
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

double get_bearing(syllo::Stream *stream, int r, int c)
{
     // Get actual range and bearing
     // Left side of sonar has negative bearing
     // Right side of sonar has positive bearing
     // Straight up the center has 0 bearing
     double result = stream->pixel_bearing(r,c) * 0.017453293; // pi/180
     //return result += 0.392699082; // add 22.5 degrees to get rid of negative bearings
     return result;
}

void NDT::set_frame(cv::Mat &src, cv::Mat &dst, syllo::Stream *stream)
{          
     cells_.clear();
     cells_.resize(4);

     int rows = src.rows;
     int cols = src.cols; 

     cv::Mat src_copy;
     cv::Mat cell_img;
     cv::Mat_<double> ndt_total = cv::Mat::zeros(src.size(), CV_64F);
     
     //int r = 200; int c = 200;
     //double bearing = get_bearing(stream, r,c);
     //double range = stream->pixel_range(r,c); //row, col
     //cv::Point recovered = stream->get_pixel(range, bearing);
     //std::string str;
     //std::cin >> str;
          
     int img_index = 0;     
     // Cycle through the four overlapping overlays     
     for (int row_shift = 0; row_shift < 2; row_shift++) {                    
          for (int col_shift = 0; col_shift < 2; col_shift++) {
               int num_cell_rows = num_segments(rows, cell_row_size_, row_shift);
               int num_cell_cols = num_segments(cols, cell_col_size_, col_shift);
               
               cell_img = src.clone();
               src_copy = src.clone();
               
               cv::Mat_<double> ndt_single = cv::Mat::zeros(src_copy.size(), CV_64F);
               
               cells_[img_index].resize(num_cell_rows);
               
               int r_cell = 0;
               int r_idx = -row_shift*cell_row_size_/2.0;               
               while (r_cell < num_cell_rows) { 
                    
                    cells_[img_index][r_cell].resize(num_cell_cols);                    
                    
                    cv::Rect rect;
                    int c_idx = -col_shift*cell_col_size_/2.0;                    
                    int c_cell = 0;
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
                              rect.width = (double)cell_col_size_ / 2.0;
                         }
               
                         if (rect.y < 0) {
                              rect.y = 0;
                              rect.height = (double)cell_row_size_ / 2.0;
                         }
                         
                         cv::Rect rect_mat(0, 0, src_copy.cols, src_copy.rows);
                         bool is_inside = (rect & rect_mat) == rect;                         
                         if (!is_inside) {

                              cout << "===================================================" << endl;
                              cout << "               ERRRRRRRRORRRRR" << endl;
                              cout << "img_index: " << img_index << endl;
                              cout << r_idx << "," << c_idx << endl;
                              cout << "rect: (" << rect.x << "," << rect.y << ") to ";
                              cout << "(" << rect.x+rect.width << "," << rect.y+rect.height << ")" << endl;                                                            
                              cout << "Not inside" << endl;
                              break;
                              //continue;
                         }

                         cv::Mat roi(src_copy,rect);                         
                         cv::rectangle(cell_img, rect, cv::Scalar(255,255,255), 1, 8, 0);
                         
                         // Calculate mean location of points
                         int count = 0;
                         cv::Point2f sum_position(0,0);
                         int sum_value = 0;
                         cv::Point2f sum_cart(0,0);
                          
                         for(int r = 0; r < roi.rows; r++) {
                              for(int c = 0; c < roi.cols; c++) {               
                                   if (roi.at<uchar>(r,c) != 0) {
                                        sum_position.x += c;
                                        sum_position.y += r;
                                        sum_value += roi.at<uchar>(r,c);
 
                                        // Get actual range and bearing
                                        double x, y;
                                        double range = stream->pixel_range(rect.y + r, rect.x + c); //row, col
                                        //double bearing = stream->pixel_bearing(rect.y + r, rect.x + c) * 0.017453293; // pi/180
                                        double bearing = get_bearing(stream, rect.y + r, rect.x + c);
                                        
                                        x = range*cos(bearing);
                                        y = range*sin(bearing);
 
                                        sum_cart.x += x;
                                        sum_cart.y += y;
                                         
                                        count++;
                                   }
                              }
                         }
 
                         cv::Point2f mean_position;
                         mean_position.x = sum_position.x / (double)count;
                         mean_position.y = sum_position.y / (double)count;               
                
                         Eigen::MatrixXd mean(2,1);                                                 
                         mean << mean_position.x, mean_position.y;
 
                         Eigen::MatrixXd mean_cart(2,1);
                         mean_cart(0,0) = sum_cart.x / (double)count;
                         mean_cart(1,0) = sum_cart.y / (double)count;
 
                         // Calculate covariance matrix                                        
                         Eigen::MatrixXd sum_covar_position = Eigen::MatrixXd::Zero(2,2);
                         sum_covar_position = Eigen::MatrixXd::Constant(2,2,0);
 
                         Eigen::MatrixXd sum_covar_pos_cart = Eigen::MatrixXd::Zero(2,2);
 
                         for(int r = 0; r < roi.rows; r++) {
                              for(int c = 0; c < roi.cols; c++) {               
                                   if (roi.at<uchar>(r,c) != 0) {
                                        Eigen::MatrixXd point(2,1);
                                        point << c, r;
 
                                        Eigen::MatrixXd diff(2,1);
                                        diff = point-mean;
                                        sum_covar_position += diff * diff.transpose();
 
                                        // Get actual range and bearing
                                        double x, y;
                                        double range = stream->pixel_range(rect.y + r, rect.x + c); //row, col
                                        //double bearing = stream->pixel_bearing(rect.y + r, rect.x + c) * 0.017453293; // pi/180
                                        double bearing = get_bearing(stream, rect.y + r, rect.x + c);
                                        x = range*cos(bearing);
                                        y = range*sin(bearing);
 
                                        Eigen::MatrixXd point_cart(2,1);
                                        point_cart << x , y;
                                         
                                        Eigen::MatrixXd diff_cart(2,1);
                                        diff_cart = point_cart - mean_cart;
 
                                        sum_covar_pos_cart += (diff_cart * diff_cart.transpose());
                                   }
                              }
                         }                                        
                
                         Cell cell;
                         cell.is_valid_ = false;
                          
                         if (count > 3) {
                              Eigen::MatrixXd covar_position(2,2);
                              covar_position = sum_covar_position / ((double)count-1.0);
                               
                              Eigen::MatrixXd covar_position_cart(2,2);
                              covar_position_cart = sum_covar_pos_cart / ((double)count-1.0);
                               
                              cell.mean_idx_ = mean;
                              cell.covar_idx_ = covar_position;
                              cell.mean_ = mean_cart;
                              cell.covar_ = covar_position_cart;
                               
                              cell.is_valid_ = true;                                                                                                                                 
                     
                              cv::Mat_<double> dist_img = cv::Mat::zeros(roi.size(), CV_64F);
                              for(int r = 0; r < roi.rows; r++) {
                                   for(int c = 0; c < roi.cols; c++) {                                                       
                               
                                        Eigen::MatrixXd point(2,1);
                                        point << c, r;
                               
                                        Eigen::MatrixXd diff(2,1);
                                        diff = (point-mean);
                               
                                        Eigen::MatrixXd numerator(1,1);
                                        numerator = diff.transpose()*covar_position.inverse()*diff;                                        
                                        
                                        dist_img.at<double>(r,c) = exp(-numerator(0,0)/2);
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
               
               ndt_total += ndt_single;
               img_index++;
          } // col_shift                   
     } // row_shift     

     //cv::Mat ndt_total_norm;
     cv::normalize(ndt_total, dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);
     //cv::imshow("NDT",ndt_total_norm);

     if (prev_cells_.size() > 0) {
          
          // Compute Jacobian
          Eigen::MatrixXd JT(2,3);
          JT << 1, 0, -x_est_*sin(phi_est_)-y_est_*cos(phi_est_),
               0, 1, x_est_*cos(phi_est_)-y_est_*sin(phi_est_);
          
          Eigen::MatrixXd JT_0(2,1);
          Eigen::MatrixXd JT_1(2,1);
          Eigen::MatrixXd JT_2(2,1);
          
          JT_0 << 1,0;
          JT_1 << 0,1;
          JT_2 << -x_est_*sin(phi_est_)-y_est_*cos(phi_est_), x_est_*cos(phi_est_)-y_est_*sin(phi_est_);
                   
          Eigen::MatrixXd H_0(2,1);
          Eigen::MatrixXd H_1(2,1);
          
          H_0 << 0,0;
          H_1 << -x_est_*cos(phi_est_) + y_est_*sin(phi_est_), -x_est_*sin(phi_est_) - y_est_*cos(phi_est_);
          
          Eigen::MatrixXd parameters(3,1);     
          parameters << 0,0,0;  // tx, ty, phi
                    
          double score_champ = -1e9;
          Eigen::MatrixXd parameters_champ(3,1);                    
          for (int iter = 0; iter < 5; iter++) {     
               cout << "------------" << endl;
               
               cv::Mat iter_img = prev_ndt_img_.clone();
               cv::cvtColor(iter_img, iter_img, CV_GRAY2BGR);
               
               double score = 0;                         
               Eigen::MatrixXd gradient_total = Eigen::MatrixXd::Constant(3,1,0);
               Eigen::MatrixXd H_total = Eigen::MatrixXd::Constant(3,3,0);
               int H_count = 0;
                
               // For each non-zero value in newest scan...               
               for (int r = 0 ; r < src_copy.rows; r++) {
                    for (int c = 0; c < src_copy.cols; c++) {               
                         if (src_copy.at<uchar>(r,c) != 0) {                         
                              // Get actual range and bearing
                              // Left side of sonar has negative bearing
                              // Right side of sonar has positive bearing
                              // Straight up the center has 0 bearing
                              //
                              // I want the x/y coordinate system to have x be in the
                              // forward direction away from the sonar and the y to be
                              // the side to side direction.
                              double x, y;
                              double range = stream->pixel_range(r,c); //row, col
                              //double bearing = stream->pixel_bearing(r,c) * 0.017453293; // pi/180
                              double bearing = get_bearing(stream,r,c);
           
                              //cout << "Range: " << range << endl;
                              //cout << "Bearing: " << bearing << endl;
                      
                              x = range*cos(bearing);
                              y = range*sin(bearing);
                                         
                              //cout << "Cartesian: (" << x << " , " << y << ")" << endl;
                      
                              Eigen::MatrixXd sample_point(2,1);
                              sample_point << x,y;
            
                              Eigen::MatrixXd point_mapped(2,1); //x'_i
                              //parameters << 0,0, -22.5*iter*3.14159/180.0;
                              point_mapped = T_map(sample_point, parameters);                              

                              point_mapped << point_mapped(0,0) , point_mapped(1,0);
                              
                              //cout << "Mapped Point: (" << point_mapped(0,0) << " , " << point_mapped(1,0) << ")" << endl;
                               
                              //Convert to polar...
                              double range_map = sqrt( pow(point_mapped(0,0),2) + pow(point_mapped(1,0),2) );
                              double bearing_map = atan2(point_mapped(1,0) , point_mapped(0,0));  
                              
                              //bearing_map -= 0.392699082;
           
                              //// ////////////////  
                              //cout << "---------" << endl;
                              //cout << "Rows: " << src_copy.rows << endl;
                              //cout << "Cols: " << src_copy.cols << endl;
                              //cout << "Checking: " << range_map << " , " << bearing_map << endl;                    
                              cv::Point recovered = stream->get_pixel(range_map, bearing_map);
                              //cout << "Found at pixel: " << recovered.x << " , " << recovered.y << endl;
                              //range = stream->pixel_range(recovered.y,recovered.x);                    
                              //bearing = get_bearing(stream, recovered.y,recovered.x);
                              //cout << "Check: " << range << " , " << bearing << endl;
                              // ////////////
                              
                              if (recovered.x < iter_img.cols && recovered.y < iter_img.rows && recovered.x >= 0 && recovered.y >= 0) {
                                   iter_img.at<cv::Vec3b>(recovered.y,recovered.x) = cv::Vec3b(0,255,0);
                              } else {
                                   cout << "Recovered outside of bounds==========> " << recovered << endl;
                                   cout << "range_map: " << range_map << endl;
                                   cout << "bearing_map: " << bearing_map << endl;
                                   cout << "recovered: " << recovered << endl;
                              }
                                               
                              // Need to determine previous covariance and mean from this cell
                              Eigen::MatrixXd cov(2,2);
                              cov << 1,0,1,0;
                      
                              Eigen::MatrixXd cov_inv = cov.inverse();
                      
                              Eigen::MatrixXd prev_mean(2,1);
                              prev_mean << 1,1;
                               
                              int find_row = recovered.y;
                              int find_col = recovered.x;
                              img_index = 0;
                              for (int row_shift = 0; row_shift < 2; row_shift++) {                    
                                   for (int col_shift = 0; col_shift < 2; col_shift++) {                                                  
                                        unsigned int r_map = floor(((double)find_row + (double)row_shift*(double)cell_row_size_/2.0) / (double)cell_row_size_);
                                        unsigned int c_map = floor(((double)find_col + (double)col_shift*(double)cell_col_size_/2.0) / (double)cell_col_size_);                         
                                         
                                        if (r_map < prev_cells_[img_index].size() && c_map < prev_cells_[img_index][r_map].size()) {
                                             cv::Rect rect = prev_cells_[img_index][r_map][c_map].rect_;
                                              
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
                                                  //std::string str;
                                                  //std::cin >> str;
                                                  continue;
                                             }
                                              
                                             // Determine if this cell had a valid statistic
                                             // If not, skip this one, add one to H_count?
                                             if (prev_cells_[img_index][r_map][c_map].is_valid_) {                                                  
                                                  Eigen::MatrixXd q(2,1);
#if 1
                                                  // Range/bearing based : X/Y
                                                  cov = prev_cells_[img_index][r_map][c_map].covar_;
                                                  prev_mean = prev_cells_[img_index][r_map][c_map].mean_;
                                                  cov_inv = cov.inverse();
                                                  q = point_mapped - prev_mean;
#else                                                  
                                                  cov = prev_cells_[img_index][r_map][c_map].covar_idx_;
                                                  prev_mean = prev_cells_[img_index][r_map][c_map].mean_idx_;
                                                  cov_inv = cov.inverse();
                                                  q << rect.x-find_col , rect.y-find_row;
                                                  q = q - prev_mean;
#endif                                                                                                   
                                                  Eigen::MatrixXd q_t = q.transpose();

                                                  double s = exp( -1/2 * (q_t * cov_inv * q)(0,0));
                                                                                                    
                                                  score += s;
                                                  
                                                  Eigen::MatrixXd qt_num = (q_t*cov_inv*q); 
                                                                                                    
                                                  double exp_num = exp( -qt_num(0,0) / 2 );
                                                  
                                                  Eigen::MatrixXd gradient(3,1);
                                                  gradient(0,0) = (q_t*cov_inv*JT_0*exp_num)(0,0);
                                                  gradient(1,0) = (q_t*cov_inv*JT_1*exp_num)(0,0);
                                                  gradient(2,0) = (q_t*cov_inv*JT_2*exp_num)(0,0);
                                               
                                                  Eigen::MatrixXd H = Eigen::MatrixXd::Constant(3,3,0);
                                                  H(0,0) = exp_num * ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_0)(0,0);
                                                  H(0,1) = exp_num * ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_0)(0,0);
                                                  H(0,2) = exp_num * ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_0 + JT_2.transpose()*cov_inv*JT_0)(0,0);
                                                  
                                                  H(1,0) = exp_num * ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_1)(0,0);
                                                  H(1,1) = exp_num * ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_1)(0,0);
                                                  H(1,2) = exp_num * ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_0 + JT_2.transpose()*cov_inv*JT_1)(0,0);
                                                  
                                                  H(2,0) = exp_num * ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_2)(0,0);
                                                  H(2,1) = exp_num * ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_2)(0,0);
                                                  H(2,2) = exp_num * ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_1 + JT_2.transpose()*cov_inv*JT_2)(0,0);
           
                                                  gradient_total += gradient;
                                                  H_total += H;

#if DEBUG
                                                  cout << "==================> Valid: " << endl;                                             
                                                  cout << "Previous Row/Col: " << r << " , " << c << endl;
                                                  cout << "New Row/Col: " << find_row << " , " << find_col << endl;
                                                  cout << "Covar: " << cov << endl;
                                                  cout << "cov_inv: " << endl << cov_inv << endl;
                                                  cout << "prev_mean: " << prev_mean << endl;
                                                  cout << "cov_inv: " << cov_inv << endl;
                                                  cout << "Diff: " << endl << q << endl;
                                                  cout << "qt_num: " << endl << qt_num << endl;
                                                  cout << "s: " << s << endl;
                                                  cout << "exp_num: " << exp_num << endl;
#endif
                                                  H_count++;
                                             } else {
                                                  // TODO: Add one to H_count?
                                             }
                                             //H_count++;
                                                                                            
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
                                   } // col_shift 
                              } // row_shift                                            
                         } // check for non-zero
                    } // cols
               } // rows
               
               if (H_count > 0) {                           
                    Eigen::MatrixXd H = H_total / (double)H_count; // TODO: 4 grids?
                    Eigen::MatrixXd gradient = gradient_total / (double)H_count;
                    Eigen::MatrixXd H_inv = H.inverse();                                        
#if DEBUG
                    cout << "Gradient: " << endl << gradient << endl;
                    cout << "H: " << endl << H << endl;                                        
                    cout << "H.inverse: " << H_inv << endl;
                    cout << "H.inverse()*gradient: " << H_inv*gradient << endl;
#endif

                    //Eigen::MatrixXd B(3,3);
                    //B << 1,0,0,
                    //     0,1,0,
                    //     0,0,1;
                    //
                    //Eigen::MatrixXd H_slow = H.fullPivLu().solve(B);
                    //cout << "inverse of H: " << H_slow << endl;
                    
                    if (H.determinant() != 0) {                    
                         parameters = parameters - H_inv*gradient; 
                    } else {
                         cout << "}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}} H ISN'T INVERTIBLE" << endl;
                    }                      
                    //cout << "Parameters: " << endl << parameters << endl;                    
                    cout << "Iteration: " << iter << " - Score: " << score;
                    cout << " - Params: " << parameters(0,0) << " , " << parameters(1,0) << " , " << parameters(2,0) << endl;

                    if (score >= score_champ) {
                         parameters_champ = parameters;
                         score_champ = score;
                         cout << "Saving: " << parameters_champ << endl;
                    }
                    
               } // H_count > 0
               else {
                    cout << "Small H_count: " << H_count << endl;
               }

               cv::imshow("Iteration Img", iter_img);
               cv::waitKey(10);
          } // iteration

          cout << "Final Params: " << parameters_champ(0,0) << " , " << parameters_champ(1,0) << " , " << parameters_champ(2,0) << endl;
          
          // Update X, Y, and Phi
          x_est_ += parameters_champ(0,0);
          y_est_ += parameters_champ(1,0);
          phi_est_ += parameters_champ(2,0);
           
          cout << "x_est: " << x_est_ << endl;
          cout << "y_est: " << y_est_ << endl;
          cout << "phi_est: " << phi_est_ << " - " << phi_est_ * 180.0 / 3.14159265359 << endl;
     } //prev_cells_.size() > 0 check     
     
     prev_ndt_img_ = dst.clone();
     cells_.swap(prev_cells_);
}
