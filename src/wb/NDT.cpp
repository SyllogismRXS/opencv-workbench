#include <iostream>

#include "NDT.h"

using std::cout;
using std::endl;

NDT::NDT()
{
     cell_col_size_ = 10;
     cell_row_size_ = 10;    
}

Eigen::MatrixXd T_map(Eigen::MatrixXd &pos, Eigen::MatrixXd &p)
{
     double tx = p(0,0);
     double ty = p(1,0);
     double phi = p(2,0);
     
     //cv::Mat_<double> image(2,1);
     Eigen::MatrixXd image(2,1);     
     
     //cv::Mat_<double> rot(2,2);
     Eigen::MatrixXd rot(2,2);
     rot << cos(phi), -sin(phi), sin(phi), cos(phi);
     
     //cv::Mat_<double> translate(2,1);
     Eigen::MatrixXd translate(2,1);
     translate << tx , ty;
     
     image = rot * pos + translate;
     
     return image;
}

void NDT::set_frame(cv::Mat &src, cv::Mat &dst, syllo::Stream *stream)
{         
     cells_.clear();
     cells_.resize(4);
     
     int rows = src.rows;
     int cols = src.cols; 

     cv::Mat src_copy = src.clone();
     
     // Divide entire region into equal cells.
     int cells_across = cols / cell_col_size_;
     int cells_down = rows / cell_row_size_;
     
     cv::Mat_<double> ndt_total = cv::Mat::zeros(src.size(), CV_64F);     
     
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
                         cv::Point2f sum_cart(0,0);
                         
                         for(int r = 0; r < roi.rows; r++) {
                              for(int c = 0; c < roi.cols; c++) {               
                                   if (roi.at<uchar>(r,c) != 0) {
                                        sum_position.x += c;
                                        sum_position.y += r;
                                        sum_value += roi.at<uchar>(r,c);

                                        // Get actual range and bearing
                                        double x, y;
                                        double range = stream->pixel_range(r,c); //row, col
                                        double bearing = stream->pixel_bearing(r,c) * 0.017453293; // pi/180
                                        y = -range*cos(bearing);
                                        x = range*sin(bearing);

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

                         //sum_covar_position = Eigen::MatrixXd::Zero();
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
                                        double range = stream->pixel_range(r,c); //row, col
                                        double bearing = stream->pixel_bearing(r,c) * 0.017453293; // pi/180
                                        y = -range*cos(bearing);
                                        x = range*sin(bearing);

                                        Eigen::MatrixXd point_cart(2,1);
                                        point_cart << x , y;
                                        
                                        Eigen::MatrixXd diff_cart(2,1);
                                        diff_cart = point_cart - mean_cart;

                                        sum_covar_pos_cart += diff_cart * diff_cart.transpose();                                        
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
                         cells.push_back(cell);
                    }
                    cells_[img_index].push_back(cells);                    
               }                              
               
               ndt_total += ndt_single;

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
     
     
     ///////////////////
     
     //cv::Mat_<double> p(3,1,CV_64F); // vector of params to estimate
     Eigen::MatrixXd p(3,1);     
     p << 0,0,0;  // tx, ty, phi
     
     // Compute Jacobian
     double x_est = 0;
     double y_est = 0;
     double phi_est = 0;
     //cv::Mat_<double> JT(2,3,CV_64F);
     Eigen::MatrixXd JT(2,3);
     JT << 1, 0, -x_est*sin(phi_est)-y_est*cos(phi_est),
          0, 1, x_est*cos(phi_est)-y_est*sin(phi_est);
     
     Eigen::MatrixXd JT_0(2,1);
     Eigen::MatrixXd JT_1(2,1);
     Eigen::MatrixXd JT_2(2,1);
     
     JT_0 << 1,0;
     JT_1 << 0,1;
     JT_2 << -x_est*sin(phi_est)-y_est*cos(phi_est), x_est*cos(phi_est)-y_est*sin(phi_est);
     
     Eigen::MatrixXd H_0(2,1);
     Eigen::MatrixXd H_1(2,1);
     
     H_0 << 0,0;
     H_1 << -x_est*cos(phi_est) + y_est*sin(phi_est), -x_est*sin(phi_est) - y_est*cos(phi_est);
         
     int used_col_cells = cells_across * cell_col_size_;
     int used_row_cells = cells_down * cell_row_size_;
     
     // For each non-zero value in newest scan...
     for (int r = 0; r < src_copy.rows; r++) {
          for (int c = 0; c < src_copy.cols; c++) {      
               if (src_copy.at<uchar>(r,c) != 0) {               
                    
                    // Get actual range and bearing
                    double x, y;
                    double range = stream->pixel_range(r,c); //row, col
                    double bearing = stream->pixel_bearing(r,c) * 0.017453293; // pi/180
                    y = -range*cos(bearing);
                    x = range*sin(bearing);                    

                    Eigen::MatrixXd sample_point(2,1);
                    sample_point << x , y;
          
                    Eigen::MatrixXd point_mapped(2,1);
                    point_mapped = T_map(sample_point, p);

                    //Convert to polar...
                    double range_map = sqrt( pow(point_mapped(0,0),2) + pow(point_mapped(1,0),2) );
                    double bearing_map = atan2(point_mapped(1,0) , point_mapped(0,0));                                        

                    ////////////////
                    //cout << "-----------" << endl;
                    //cout << "Checking: " << range_map << " , " << bearing_map << endl;                    
                    cv::Point recovered = stream->get_pixel(range_map, bearing_map);
                    //cout << "Found at pixel: " << recovered.x << " , " << recovered.y << endl;
                    //range = stream->pixel_range(recovered.y,recovered.x);                    
                    //bearing = stream->pixel_bearing(recovered.y,recovered.x) * 0.017453293;
                    //cout << "Check: " << range << " , " << bearing << endl;
                    ////////////
     
                    int r_map = (double)recovered.y / (double)used_row_cells * (double)cells_down;
                    int c_map = (double)recovered.x / (double)used_col_cells * (double)cells_across;                                       
                    
                    // TODO: Calculate probabilty of this point being here..
                    
                    for (int i = 0; i < 4; i++) {                         
                         prev_cells_[i][r_map][c_map];
                    }
                    
                    // Need to determine previous covariance and mean from this cell
                    Eigen::MatrixXd cov(2,2);
                    cov << 1, 0, 1, 0;
     
                    Eigen::MatrixXd cov_inv = cov.inverse();
                    
                    Eigen::MatrixXd prev_mean(2,1);
                    prev_mean << 1, 1;
     
                    Eigen::MatrixXd q(2,1);
                    q = point_mapped - prev_mean;          
     
                    Eigen::MatrixXd q_t = q.transpose();
          
                    double exp_num = exp( (-q_t*cov_inv*q)(0,0) / 2 );
          
                    Eigen::MatrixXd gradient(3,1);
                    gradient(0,0) = (q_t*cov_inv*JT_0*exp_num)(0,0);
                    gradient(1,0) = (q_t*cov_inv*JT_1*exp_num)(0,0);
                    gradient(2,0) = (q_t*cov_inv*JT_2*exp_num)(0,0);
     
                    Eigen::MatrixXd H = Eigen::MatrixXd::Constant(3,3,exp_num);
                    H(0,0) *= ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_0)(0,0);
                    H(0,1) *= ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_0)(0,0);
                    H(0,2) *= ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_0 + JT_2.transpose()*cov_inv*JT_0)(0,0);
          
                    H(1,0) *= ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_1)(0,0);
                    H(1,1) *= ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_1)(0,0);
                    H(1,2) *= ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_0 + JT_2.transpose()*cov_inv*JT_1)(0,0);
          
                    H(2,0) *= ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_2)(0,0);
                    H(2,1) *= ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_2)(0,0);
                    H(2,2) *= ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_1 + JT_2.transpose()*cov_inv*JT_2)(0,0);
     
                    Eigen::MatrixXd p_next(3,1);
                    p_next = p - H.inverse()*gradient; 
               }      
          }
     }
                      
     // Save the currently computed cells in the previous cells container
     cells_.swap(prev_cells_);
}
