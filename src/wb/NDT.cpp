#include <iostream>
#include <cmath>

#include "NDT.h"
#include <opencv_workbench/syllo/syllo.h>

using std::cout;
using std::endl;

#define USE_SHIFT 1

#if USE_SHIFT
#define SHIFT 2
#else
#define SHIFT 1
#endif

#define PRIORITY_ROTATE 1

// Use cartesian (X/Y) (=1) or (row,column) (0)
#define USE_XY 1

#define DEBUG 0

NDT::NDT()
{
     cell_col_size_ = 20;//10;//30;//5;//20;//20;//20;//10;
     cell_row_size_ = 20;//10;//30;//5;//20;//20;//20;//10;

     frame_ = 0;

     xy_est_ = Eigen::MatrixXd(2,1);
     xy_est_ << 0,0;
     phi_est_ = 0;
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

void cart2polar(double x, double y, double &range, double &bearing)
{
     range = sqrt( pow(x,2) + pow(y,2) );
     bearing = atan2(y,x);
}

cv::Point2f get_cart(syllo::Stream *stream, int r, int c)
{
     // Get actual range and bearing
     cv::Point2f point;
     double range = stream->pixel_range(r,c); //row, col
     double bearing = stream->pixel_bearing(r,c) * 0.017453293; // pi/180

     point.x = range*sin(bearing);
     point.y = range*cos(bearing);
     return point;
}

cv::Point cart_2_rc(syllo::Stream *stream, cv::Point2f cart);
cv::Point cart_2_rc(syllo::Stream *stream, Eigen::MatrixXd cart);

cv::Point cart_2_rc(syllo::Stream *stream, Eigen::MatrixXd cart)
{
     cv::Point2f p(cart(0,0),cart(1,0));
     return cart_2_rc(stream, p);
}

cv::Point cart_2_rc(syllo::Stream *stream, cv::Point2f cart)
{
     double range, bearing;
     cart2polar(cart.x, cart.y, range, bearing);
     return stream->get_pixel(range, bearing);
}

void NDT::set_frame(cv::Mat &src, cv::Mat &dst, syllo::Stream *stream)
{
     // dst = src.clone();
     // for (int r = 0 ; r < src.rows; r++) {
     //      for (int c = 0; c < src.cols; c++) {
     //           dst = src.clone();
     //
     //           cout << "----------" << endl;
     //           cout << "row: " << r << endl;
     //           cout << "col: " << c << endl;
     //
     //           Eigen::MatrixXd parameters(3,1);
     //           parameters << 0,0,0;
     //
     //           // Get the cartesian from row/col
     //           cv::Point2f cart = get_cart(stream, r, c);
     //           cout << "Cart: " << cart << endl;
     //
     //           // Use T_map to transform the points
     //           Eigen::MatrixXd point_mapped(2,1); //x'_i
     //           Eigen::MatrixXd sample_point(2,1);
     //           sample_point << cart.x, cart.y;
     //           point_mapped = T_map(sample_point, parameters);
     //
     //           cart.x = point_mapped(0,0);
     //           cart.y = point_mapped(1,0);
     //
     //           // Get the
     //           double range, bearing;
     //           cart2polar(cart.x, cart.y, range, bearing);
     //           cv::Point recovered = stream->get_pixel(range, bearing);
     //
     //           cout << "----------" << endl;
     //           cout << "row: " << r << endl;
     //           cout << "col: " << c << endl;
     //           cout << "cart: " << cart << endl;
     //           cout << "recovered: " << recovered << endl;
     //
     //           if (recovered.x != c || recovered.y != r) {
     //                cout << "======> Mismatch" << endl;
     //           }
     //
     //           cv::circle(dst, cv::Point(c,r), 5, 255, 1, 8, 0);
     //           cv::imshow("Cur Pix", dst);
     //           cv::waitKey(0);
     //      }
     // }
     // return;

     // if (frame_ % 5 != 0) {
     //      dst = src.clone();
     //      frame_++;
     //      return;
     // }
     // frame_++;

     cells_.clear();
     cells_.resize(4);

     int rows = src.rows;
     int cols = src.cols;

     cv::Mat src_copy;
     cv::Mat cell_img;
     cv::Mat_<double> ndt_total = cv::Mat::zeros(src.size(), CV_64F);

     int img_index = 0;
     // Cycle through the four overlapping overlays
     for (int row_shift = 0; row_shift < SHIFT; row_shift++) {
          for (int col_shift = 0; col_shift < SHIFT; col_shift++) {
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
                                        cv::Point2f cart = get_cart(stream, rect.y + r, rect.x + c);
                                        sum_cart.x += cart.x;
                                        sum_cart.y += cart.y;

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

                         Eigen::MatrixXd sum_covar_pos_cart = Eigen::MatrixXd::Zero(2,2);

                         for(int r = 0; r < roi.rows; r++) {
                              for(int c = 0; c < roi.cols; c++) {
                                   if (roi.at<uchar>(r,c) != 0) {
                                        Eigen::MatrixXd point(2,1);
                                        point << c, r;

                                        Eigen::MatrixXd diff(2,1);
                                        diff = point-mean;
                                        sum_covar_position += (diff * diff.transpose());

                                        // Get actual range and bearing
                                        cv::Point2f cart = get_cart(stream, rect.y + r, rect.x + c);

                                        Eigen::MatrixXd point_cart(2,1);
                                        point_cart << cart.x , cart.y;

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

#if USE_XY
                              double eig_scale = 0.01;
#else
                              double eig_scale = 0.1;
#endif

                              // Check to see if smaller eigenvalue is at least
                              // 0.001 times the larger eigenvalue
                              Eigen::VectorXcd eigvals = covar_position_cart.eigenvalues();
                              double eig1 = eigvals(0).real();
                              double eig2 = eigvals(1).real();

                              if (eig1 < eig2) {
                                   if (eig1 < eig2*eig_scale) {
                                        eig1 = eig2*eig_scale;

                                        Eigen::EigenSolver<Eigen::MatrixXd> es(covar_position_cart);
                                        Eigen::MatrixXcd ev = es.eigenvectors();
                                        Eigen::MatrixXd ev_real = ev.real();

                                        Eigen::MatrixXd eigs_corrected(2,2);
                                        eigs_corrected << eig1, 0 , 0, eig2;
                                        covar_position_cart = ev_real * eigs_corrected * ev_real.inverse();
                                        cell.mod_eigs_ = true;
                                   }

                              } else {
                                   if (eig2 < eig1*eig_scale) {
                                        eig2 = eig1*eig_scale;

                                        Eigen::EigenSolver<Eigen::MatrixXd> es(covar_position_cart);
                                        Eigen::MatrixXcd ev = es.eigenvectors();
                                        Eigen::MatrixXd ev_real = ev.real();

                                        Eigen::MatrixXd eigs_corrected(2,2);
                                        eigs_corrected << eig1, 0 , 0, eig2;
                                        covar_position_cart = ev_real * eigs_corrected * ev_real.inverse();
                                        cell.mod_eigs_ = true;
                                   }
                              }

                              ////////////
                              // Check to see if smaller eigenvalue is at least
                              // eig_scale times the larger eigenvalue
                              eigvals = covar_position.eigenvalues();
                              eig1 = eigvals(0).real();
                              eig2 = eigvals(1).real();

                              if (eig1 < eig2) {
                                   if (eig1 < eig2*eig_scale) {
                                        eig1 = eig2*eig_scale;

                                        Eigen::EigenSolver<Eigen::MatrixXd> es(covar_position);
                                        Eigen::MatrixXcd ev = es.eigenvectors();
                                        Eigen::MatrixXd ev_real = ev.real();

                                        Eigen::MatrixXd eigs_corrected(2,2);
                                        eigs_corrected << eig1, 0 , 0, eig2;
                                        covar_position = ev_real * eigs_corrected * ev_real.inverse();
                                        cell.mod_eigs_ = true;
                                   }

                              } else {
                                   if (eig2 < eig1*eig_scale) {
                                        eig2 = eig1*eig_scale;

                                        Eigen::EigenSolver<Eigen::MatrixXd> es(covar_position);
                                        Eigen::MatrixXcd ev = es.eigenvectors();
                                        Eigen::MatrixXd ev_real = ev.real();

                                        Eigen::MatrixXd eigs_corrected(2,2);
                                        eigs_corrected << eig1, 0 , 0, eig2;
                                        covar_position = ev_real * eigs_corrected * ev_real.inverse();
                                        cell.mod_eigs_ = true;
                                   }
                              }
                              ////////////

                              cell.mean_idx_ = mean;
                              cell.covar_idx_ = covar_position;
                              cell.mean_ = mean_cart;
                              cell.covar_ = covar_position_cart;

                              cell.is_valid_ = true;

                              cv::Mat_<double> dist_img = cv::Mat::zeros(roi.size(), CV_64F);
                              for(int r = 0; r < roi.rows; r++) {
                                   for(int c = 0; c < roi.cols; c++) {
#if USE_XY
                                        // Using cartesian coordinates to draw NDT image;
                                        cv::Point2f cart = get_cart(stream, rect.y + r, rect.x + c);

                                        Eigen::MatrixXd point(2,1);
                                        point << cart.x , cart.y;

                                        Eigen::MatrixXd diff(2,1);
                                        diff = (point-mean_cart);

                                        Eigen::MatrixXd numerator(1,1);
                                        numerator = diff.transpose()*covar_position_cart.inverse()*diff;

#else
                                        // Using r / c to draw NDT image
                                        Eigen::MatrixXd point(2,1);
                                        point << c, r;

                                        Eigen::MatrixXd diff(2,1);
                                        diff = (point-mean);

                                        Eigen::MatrixXd numerator(1,1);
                                        numerator = diff.transpose()*covar_position.inverse()*diff;
#endif
                                        dist_img.at<double>(r,c) = exp(-numerator(0,0)/2);
                                   }
                              }

                              cv::normalize(dist_img, dist_img, 0, 255, cv::NORM_MINMAX, CV_64F);
                              cv::Mat roi_dst(ndt_single, rect);
                              dist_img.copyTo(roi_dst);
#if 0
                              // Plot the roi generation: points and covariance
                              cv::Mat cov_w_points = dist_img.clone();
                              cv::normalize(cov_w_points, cov_w_points, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                              //cout << "cov_w_points: " << endl << cov_w_points << endl;
                              cv::cvtColor(cov_w_points, cov_w_points, CV_GRAY2BGR);
                              for(int r = 0; r < roi.rows; r++) {
                                   for(int c = 0; c < roi.cols; c++) {
                                        if (roi.at<uchar>(r,c) != 0) {
                                             cov_w_points.at<cv::Vec3b>(r,c) = cv::Vec3b(0,255,0);
                                        }
                                   }
                              }

                              // Visualize distribution creation
                              double scale = 10;
                              cv::resize(dist_img, dist_img, cv::Size(0,0),scale,scale,cv::INTER_NEAREST);
                              cv::imshow("cov image", dist_img);

                              cv::resize(cov_w_points, cov_w_points, cv::Size(0,0),scale,scale,cv::INTER_NEAREST);
                              cv::imshow("cov with points", cov_w_points);
                              cv::waitKey(0);
#endif
                         }

                         cell.rect_ = rect;
                         cells_[img_index][r_cell][c_cell] = cell;
                         c_cell++;
                         c_idx = rect.x + rect.width;
                    }
                    r_cell++;
                    r_idx = rect.y + rect.height;
               }
               ndt_total += ndt_single;
               img_index++;
          } // col_shift
     } // row_shift

     //cv::Mat ndt_total_norm;
     cv::normalize(ndt_total, dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);

//      if (prev_cells_.size() > 0) {
//           Eigen::MatrixXd parameters(3,1);
//           parameters << 0,0,-.001;  // tx, ty, phi
// 
//           cv::Point2f transl_offset;
//           double lambda = 0.1;
//           double score_rotation = 0;
//           for (int iter = 0; iter < 5; iter++) {
//                cout << "========================================" << endl;
//                cout << "Iteration: " << iter << endl;
//                cout << "Parameters: " << parameters << endl;
// 
//                Eigen::MatrixXd JT_0(2,1);
//                Eigen::MatrixXd JT_1(2,1);
//                Eigen::MatrixXd JT_2(2,1);
// 
//                Eigen::MatrixXd H_0(2,1);
//                Eigen::MatrixXd H_1(2,1);
// 
// #if PRIORITY_ROTATE
//                JT_0 << 0,0;
//                JT_1 << 0,0;
//                JT_2 << -sin(parameters(2,0))-cos(parameters(2,0)),
//                     cos(parameters(2,0))-sin(parameters(2,0));
// 
//                H_0 << 0,0;
//                H_1 << -cos(parameters(2,0)) + sin(parameters(2,0)),
//                     -sin(parameters(2,0)) - cos(parameters(2,0));
// #else
//                JT_0 << 1,0;
//                JT_1 << 0,1;
//                JT_2 << -parameters(0,0)*sin(parameters(2,0))-parameters(1,0)*cos(parameters(2,0)),
//                         parameters(0,0)*cos(parameters(2,0))-parameters(1,0)*sin(parameters(2,0));
// 
//                H_0 << 0,0;
//                H_1 << -parameters(0,0)*cos(parameters(2,0)) + parameters(1,0)*sin(parameters(2,0)),
//                       -parameters(0,0)*sin(parameters(2,0)) - parameters(1,0)*cos(parameters(2,0));
// #endif
//                cv::Mat line_img = prev_img_.clone();
//                cv::cvtColor(line_img, line_img, CV_GRAY2BGR);
// 
//                cv::Mat iter_img = prev_img_.clone();
//                cv::cvtColor(iter_img, iter_img, CV_GRAY2BGR);
// 
//                double score = 0;
//                Eigen::MatrixXd gradient_total = Eigen::MatrixXd::Constant(3,1,0);
//                Eigen::MatrixXd H_total = Eigen::MatrixXd::Constant(3,3,0);
//                int H_count = 0;
// 
//                // For each non-zero value in newest scan...
//                for (int r = 0 ; r < src_copy.rows; r++) {
//                     for (int c = 0; c < src_copy.cols; c++) {
//                          if (src_copy.at<uchar>(r,c) != 0) {
//                               // Get actual range and bearing
//                               // Left side of sonar has negative bearing
//                               // Right side of sonar has positive bearing
//                               // Straight up the center has 0 bearing
//                               //
//                               // I want the x/y coordinate system to have x be in the
//                               // forward direction away from the sonar and the y to be
//                               // the side to side direction.
// 
//                               cv::Point2f cart = get_cart(stream,r,c);
// 
//                               Eigen::MatrixXd sample_point(2,1);
//                               sample_point << cart.x, cart.y;
// 
//                               Eigen::MatrixXd point_mapped(2,1); //x'_i
//                               point_mapped = T_map(sample_point, parameters);
//                               cart.x = point_mapped(0,0);
//                               cart.y = point_mapped(1,0);
// 
//                               double range_map, bearing_map;
//                               cart2polar(cart.x, cart.y, range_map, bearing_map);
// 
//                               cv::Point recovered = stream->get_pixel(range_map, bearing_map);
// 
//                               if (recovered.x < iter_img.cols && recovered.y < iter_img.rows && recovered.x >= 0 && recovered.y >= 0) {
//                                    iter_img.at<cv::Vec3b>(recovered.y,recovered.x) = cv::Vec3b(0,255,0);
//                               } else {
//                                    // TODO: look into why this is happening
//                                    cout << "ERROR - Recovered outside of bounds==========> " << recovered << endl;
//                                    cout << "range_map: " << range_map << endl;
//                                    cout << "bearing_map: " << bearing_map << endl;
//                               }
//                               cv::line(line_img, cv::Point(c,r), recovered, cv::Scalar(255,0,0), 1, 8, 0);
//                               cv::circle(line_img, cv::Point(c,r), 1, cv::Scalar(0,255,0), 1, 8, 0);
//                               cv::circle(line_img, recovered, 1, cv::Scalar(0,0,255), 1, 8, 0);
// 
//                               int find_row = recovered.y;
//                               int find_col = recovered.x;
//                               img_index = 0;
//                               for (int row_shift = 0; row_shift < SHIFT; row_shift++) {
//                                    for (int col_shift = 0; col_shift < SHIFT; col_shift++) {
//                                         unsigned int r_map = floor(((double)find_row + (double)row_shift*(double)cell_row_size_/2.0) / (double)cell_row_size_);
//                                         unsigned int c_map = floor(((double)find_col + (double)col_shift*(double)cell_col_size_/2.0) / (double)cell_col_size_);
// 
//                                         if (r_map < prev_cells_[img_index].size() && c_map < prev_cells_[img_index][r_map].size()) {
//                                              cv::Rect rect = prev_cells_[img_index][r_map][c_map].rect_;
// 
//                                              if (!contains(rect, cv::Point(find_col, find_row))) {
//                                                   cout << "---------------------" << endl;
//                                                   cout << "DOESN'T CONTAIN" << endl;
//                                                   cout << "Rows: " << src_copy.rows << endl;
//                                                   cout << "Cols: " << src_copy.cols << endl;
//                                                   cout << "find_col: " << find_col << endl;
//                                                   cout << "find_row: " << find_row << endl;
//                                                   cout << "r_map: " << r_map << endl;
//                                                   cout << "c_map: " << c_map << endl;
//                                                   cout << "Img: " << img_index;
//                                                   cout << " , rect: (" << rect.x << "," << rect.y << ") to ";
//                                                   cout << "(" << rect.x+rect.width << "," << rect.y+rect.height << ")" << endl;
//                                                   continue;
//                                              }
// 
//                                              // Determine if this cell had a valid statistic
//                                              // If not, skip this one, add one to H_count?
//                                              if (prev_cells_[img_index][r_map][c_map].is_valid_) {
// 
//                                                   Eigen::MatrixXd cov(2,2);
//                                                   Eigen::MatrixXd cov_inv(2,2);
//                                                   Eigen::MatrixXd prev_mean(2,1);
// 
//                                                   Eigen::MatrixXd q(2,1);
// 
//                                                   Eigen::MatrixXd prev_mean_idx = prev_cells_[img_index][r_map][c_map].mean_idx_;
//                                                   cv::Point prev_mean_pt(cvRound(prev_mean_idx(0,0)),cvRound(prev_mean_idx(1,0)));
// #if USE_XY
//                                                   // Range/bearing based : X/Y
//                                                   cov = prev_cells_[img_index][r_map][c_map].covar_;
//                                                   prev_mean = prev_cells_[img_index][r_map][c_map].mean_;
//                                                   cov_inv = cov.inverse();
//                                                   q = point_mapped - prev_mean;
// 
// #else
//                                                   cov = prev_cells_[img_index][r_map][c_map].covar_idx_;
//                                                   prev_mean = prev_cells_[img_index][r_map][c_map].mean_idx_;
//                                                   cov_inv = cov.inverse();
//                                                   q << recovered.x-rect.x, recovered.y-rect.y;
//                                                   q = q - prev_mean;
// #endif
//                                                   Eigen::MatrixXd q_t = q.transpose();
// 
//                                                   Eigen::MatrixXd qt_num = (-q_t*cov_inv*q);
//                                                   double exp_num = exp( qt_num(0,0) / 2 );
// 
//                                                   Eigen::MatrixXd gradient(3,1);
//                                                   gradient(0,0) = (q_t*cov_inv*JT_0*exp_num)(0,0);
//                                                   gradient(1,0) = (q_t*cov_inv*JT_1*exp_num)(0,0);
//                                                   gradient(2,0) = (q_t*cov_inv*JT_2*exp_num)(0,0);
// 
//                                                   Eigen::MatrixXd H = Eigen::MatrixXd::Constant(3,3,0);
//                                                   H(0,0) = exp_num * ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_0)(0,0);
//                                                   H(0,1) = exp_num * ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_0)(0,0);
//                                                   H(0,2) = exp_num * ( (q_t*cov_inv*JT_0)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_0 + JT_2.transpose()*cov_inv*JT_0)(0,0);
// 
//                                                   H(1,0) = exp_num * ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_1)(0,0);
//                                                   H(1,1) = exp_num * ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_1)(0,0);
//                                                   H(1,2) = exp_num * ( (q_t*cov_inv*JT_1)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_0 + JT_2.transpose()*cov_inv*JT_1)(0,0);
// 
//                                                   H(2,0) = exp_num * ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_0) + q_t*cov_inv*H_0 + JT_0.transpose()*cov_inv*JT_2)(0,0);
//                                                   H(2,1) = exp_num * ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_1) + q_t*cov_inv*H_0 + JT_1.transpose()*cov_inv*JT_2)(0,0);
//                                                   H(2,2) = exp_num * ( (q_t*cov_inv*JT_2)*(-q_t*cov_inv*JT_2) + q_t*cov_inv*H_1 + JT_2.transpose()*cov_inv*JT_2)(0,0);
// 
//                                                   gradient_total += gradient;
//                                                   H_total += H;
//                                                   H_count++;
//                                                   score += exp_num;
// 
// #if 0
//                                                   int count = 0;
//                                                   cv::Point2f sum_position(0,0);
//                                                   double champ_value = -1e9;
//                                                   cv::Point champ_point(0,0);
//                                                   cv::Mat_<double> rect_img = cv::Mat::zeros(rect.height,rect.width, CV_64F);
//                                                   cv::Mat roi (prev_img_, rect);
//                                                   for (int r_y = 0; r_y < roi.rows; r_y++) {
//                                                        for (int c_x = 0; c_x < roi.cols; c_x++) {
// #if USE_XY
//                                                             // Using cartesian coordinates to draw NDT image;
//                                                             cv::Point2f cart = get_cart(stream, rect.y + r_y , rect.x + c_x);
// 
//                                                             Eigen::MatrixXd point(2,1);
//                                                             point << cart.x , cart.y;
// 
//                                                             if (roi.at<uchar>(r_y, c_x) != 0) {
//                                                                  sum_position.x += cart.x;
//                                                                  sum_position.y += cart.y;
//                                                                  count++;
//                                                             }
// 
//                                                             Eigen::MatrixXd diff(2,1);
//                                                             diff = (point-prev_mean);
// 
//                                                             Eigen::MatrixXd qt_num = (-diff.transpose()*cov_inv*diff);
//                                                             double exp_num = exp( qt_num(0,0) / 2.0 );
//                                                             rect_img.at<double>(r_y,c_x) = exp_num;
// 
//                                                             if (exp_num > champ_value) {
//                                                                  champ_value = exp_num;
//                                                                  champ_point = cv::Point(c_x, r_y);
//                                                             }
// #else
//                                                             // Using r / c to draw NDT image
//                                                             Eigen::MatrixXd point(2,1);
//                                                             point << c_x, r_y;
// 
//                                                             Eigen::MatrixXd diff(2,1);
//                                                             diff = (point-prev_mean);
// 
//                                                             Eigen::MatrixXd numerator(1,1);
//                                                             numerator = diff.transpose()*cov_inv*diff;
//                                                             rect_img.at<double>(r_y,c_x) = exp(-numerator(0,0)/2);
// #endif
//                                                        }
//                                                   }
//                                                   Eigen::MatrixXd delta_parameters;
//                                                   delta_parameters = H.colPivHouseholderQr().solve(-gradient);
// 
//                                                   // Plot a single Hessian generation
//                                                   cout << "===============" << endl;
//                                                   cout << "Delta Params: " << delta_parameters << endl;
//                                                   cv::Point2f mean_pos (sum_position.x/(double)count, sum_position.y/(double)count);
//                                                   cout << "Row: " << r << endl;
//                                                   cout << "Col: " << c << endl;
//                                                   cout << "Rect: " << rect << endl;
//                                                   cout << "Cart Point: " << endl << point_mapped << endl;
//                                                   cout << "Point: " << "(" << recovered.x-rect.x << "," << recovered.y-rect.y << ")" << endl;
//                                                   cout << "PrevMean: " << endl << prev_mean << endl;
//                                                   cout << "prev_mean_idx: " << prev_mean_idx << endl;
//                                                   cout << "PrevMean Point " << "(" << prev_mean_pt.x << "," << prev_mean_pt.y << ")" << endl;
//                                                   cout << "PrevCov: " << endl << cov << endl;
//                                                   cout << "Score: " << exp_num << endl;
//                                                   cout << "Gradient: " << endl << gradient << endl;
//                                                   cout << "Hessian: " << endl << H << endl;
//                                                   cout << "eig(H): " << endl << H.eigenvalues() << endl;
// 
//                                                   //// Plot this ROI, with previous covariance and new point.
//                                                   int scale = 10;
//                                                   cv::Mat rect_img_norm;
//                                                   cv::Mat rect_img_norm_copy;
//                                                   cv::normalize(rect_img, rect_img_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
//                                                   rect_img_norm_copy = rect_img_norm.clone();
//                                                   cv::cvtColor(rect_img_norm, rect_img_norm,CV_GRAY2BGR);
// 
//                                                   // Current Point: Green
//                                                   rect_img_norm.at<cv::Vec3b>(recovered.y-rect.y,recovered.x-rect.x) = cv::Vec3b(0,255,0);
// 
//                                                   // Recalculated Mean: Red
//                                                   cv::Point recalc_mean_pos = cart_2_rc(stream, mean_pos);
//                                                   recalc_mean_pos = cv::Point(recalc_mean_pos.x-rect.x,recalc_mean_pos.y-rect.y);
//                                                   rect_img_norm.at<cv::Vec3b>(recalc_mean_pos) = cv::Vec3b(0,0,255);
// 
//                                                   // Previous Mean: Blue
//                                                   rect_img_norm.at<cv::Vec3b>(prev_mean_pt) = cv::Vec3b(255,0,0);
// 
//                                                   cv::resize(rect_img_norm, rect_img_norm, cv::Size(0,0),scale,scale,cv::INTER_NEAREST);
//                                                   cv::resize(rect_img_norm_copy, rect_img_norm_copy, cv::Size(0,0),scale,scale,cv::INTER_NEAREST);
// 
//                                                   cv::Mat around_img = prev_ndt_img_.clone();
//                                                   cv::cvtColor(around_img, around_img, CV_GRAY2BGR);
//                                                   cv::circle(around_img, recovered, 5, cv::Scalar(0,255,0), 1, 8, 0);
//                                                   around_img = cv::Mat(around_img, cv::Rect(recovered.x-50, recovered.y-50, 100, 100));
//                                                   cv::resize(around_img, around_img, cv::Size(0,0),3,3,cv::INTER_NEAREST);
// 
// 
//                                                   cv::imshow("Curr Block", around_img);
//                                                   cv::imshow("rect_img_norm_copy", rect_img_norm_copy);
//                                                   cv::imshow("rect_img", rect_img_norm);
// 
//                                                   cv::waitKey(0);
// #endif
// 
// #if DEBUG
//                                                   cout << "==================> Valid: " << endl;
//                                                   cout << "Previous Row/Col: " << r << " , " << c << endl;
//                                                   cout << "New Row/Col: " << find_row << " , " << find_col << endl;
//                                                   cout << "Covar: " << cov << endl;
//                                                   cout << "cov_inv: " << endl << cov_inv << endl;
//                                                   cout << "prev_mean: " << prev_mean << endl;
//                                                   cout << "cov_inv: " << cov_inv << endl;
//                                                   cout << "Diff: " << endl << q << endl;
//                                                   cout << "qt_num: " << endl << qt_num << endl;
//                                                   cout << "s: " << s << endl;
//                                                   cout << "exp_num: " << exp_num << endl;
// #endif
//                                              }
//                                         }
//                                         img_index++;
//                                    } // col_shift
//                               } // row_shift
//                          } // check for non-zero
//                     } // cols
//                } // rows
// 
//                std::string iter_img_name = "Iteration: " + syllo::int2str(iter);
//                cv::imshow(iter_img_name, iter_img);
// 
//                cout << "Score: " << score << endl;
// 
//                if (H_count > 0) {
// 
//                     Eigen::MatrixXd gradient = gradient_total / (double)H_count;
//                     Eigen::MatrixXd H = H_total / (double)H_count;
// 
//                     // Determine if H is positive definite:
//                     Eigen::VectorXcd eigvals = H.eigenvalues();
//                     std::vector<double> eigs;
//                     eigs.resize(3);
//                     bool contains_non_positive = false;
//                     double smallest_value = 1e9;
//                     for (int i = 0; i < 3; i++) {
//                          eigs[i] = eigvals(i).real();
// 
//                          if (eigs[i] <= 0) {
//                               contains_non_positive = true;
//                          }
// 
//                          if (eigs[i] < smallest_value) {
//                               smallest_value = eigs[i];
//                          }
//                     }
// 
// #if 1
//                     if (contains_non_positive) {
//                          Eigen::EigenSolver<Eigen::MatrixXd> es(H);
//                          Eigen::MatrixXcd ev = es.eigenvectors();
//                          Eigen::MatrixXd ev_real = ev.real();
// 
//                          double inc_value = std::abs(smallest_value);
// 
//                          Eigen::MatrixXd eigs_corrected(3,3);
//                          eigs_corrected << eigs[0] + inc_value, 0 , 0 ,
//                               0 , eigs[1] + inc_value, 0,
//                               0,   0, eigs[2] + inc_value;
// 
//                          H = ev_real * eigs_corrected * ev_real.inverse();
//                     }
// #endif
// 
//                     //cout << "H = " << endl << H << endl;
//                     //cout << "g = " << endl << gradient << endl;
// 
//                     H = H + lambda * Eigen::MatrixXd::Identity(3,3);
// 
//                     Eigen::MatrixXd dp;
//                     dp = H.colPivHouseholderQr().solve(-gradient);
//                     Eigen::MatrixXd lm_parameters = parameters + dp;
// 
// #if PRIORITY_ROTATE
//                     // Lock tx, ty at zero for now.
//                     parameters(0,0) = 0;
//                     parameters(1,0) = 0;
//                     lm_parameters(0,0) = 0;
//                     lm_parameters(1,0) = 0;
// #endif
// 
//                     // Recalculate score only!
//                     cout << "Scoring New Parameters: " << lm_parameters << endl;
//                     double new_score = 0;
//                     cv::Mat line_img = prev_img_.clone();
//                     cv::cvtColor(line_img, line_img, CV_GRAY2BGR);
//                     cv::Point2f transl_offset_sum(0,0);
//                     int transl_offset_count = 0;
//                     ////////////////////////////////
//                     // For each non-zero value in newest scan...
//                     for (int r = 0 ; r < src_copy.rows; r++) {
//                          for (int c = 0; c < src_copy.cols; c++) {
//                               if (src_copy.at<uchar>(r,c) != 0) {
//                                    cv::Point2f cart = get_cart(stream,r,c);
// 
//                                    Eigen::MatrixXd sample_point(2,1);
//                                    sample_point << cart.x, cart.y;
// 
//                                    Eigen::MatrixXd point_mapped(2,1); //x'_i
//                                    point_mapped = T_map(sample_point, lm_parameters);
//                                    cart.x = point_mapped(0,0);
//                                    cart.y = point_mapped(1,0);
// 
//                                    double range_map, bearing_map;
//                                    cart2polar(cart.x, cart.y, range_map, bearing_map);
// 
//                                    cv::Point recovered = stream->get_pixel(range_map, bearing_map);
// 
//                                    if (recovered.x < iter_img.cols && recovered.y < iter_img.rows && recovered.x >= 0 && recovered.y >= 0) {
//                                         iter_img.at<cv::Vec3b>(recovered.y,recovered.x) = cv::Vec3b(0,255,0);
//                                    } else {
//                                         cout << "Scoring: Recovered outside of bounds==========> " << recovered << endl;
//                                         //cout << "range_map: " << range_map << endl;
//                                         //cout << "bearing_map: " << bearing_map << endl;
//                                    }
//                                    cv::line(line_img, cv::Point(c,r), recovered, cv::Scalar(255,0,0), 1, 8, 0);
//                                    cv::circle(line_img, cv::Point(c,r), 1, cv::Scalar(0,255,0), 1, 8, 0);
//                                    cv::circle(line_img, recovered, 1, cv::Scalar(0,0,255), 1, 8, 0);
// 
//                                    int find_row = recovered.y;
//                                    int find_col = recovered.x;
//                                    img_index = 0;
//                                    for (int row_shift = 0; row_shift < SHIFT; row_shift++) {
//                                         for (int col_shift = 0; col_shift < SHIFT; col_shift++) {
//                                              unsigned int r_map = floor(((double)find_row + (double)row_shift*(double)cell_row_size_/2.0) / (double)cell_row_size_);
//                                              unsigned int c_map = floor(((double)find_col + (double)col_shift*(double)cell_col_size_/2.0) / (double)cell_col_size_);
// 
//                                              if (r_map < prev_cells_[img_index].size() && c_map < prev_cells_[img_index][r_map].size()) {
//                                                   cv::Rect rect = prev_cells_[img_index][r_map][c_map].rect_;
// 
//                                                   if (!contains(rect, cv::Point(find_col, find_row))) {
//                                                        cout << "---------------------" << endl;
//                                                        cout << "DOESN'T CONTAIN" << endl;
//                                                        cout << "Rows: " << src_copy.rows << endl;
//                                                        cout << "Cols: " << src_copy.cols << endl;
//                                                        cout << "find_col: " << find_col << endl;
//                                                        cout << "find_row: " << find_row << endl;
//                                                        cout << "r_map: " << r_map << endl;
//                                                        cout << "c_map: " << c_map << endl;
//                                                        cout << "Img: " << img_index;
//                                                        cout << " , rect: (" << rect.x << "," << rect.y << ") to ";
//                                                        cout << "(" << rect.x+rect.width << "," << rect.y+rect.height << ")" << endl;
//                                                        //std::string str;
//                                                        //std::cin >> str;
//                                                        continue;
//                                                   }
// 
//                                                   // Determine if this cell had a valid statistic
//                                                   // If not, skip this one, add one to H_count?
//                                                   if (prev_cells_[img_index][r_map][c_map].is_valid_) {
// 
//                                                        Eigen::MatrixXd cov(2,2);
//                                                        Eigen::MatrixXd cov_inv(2,2);
//                                                        Eigen::MatrixXd prev_mean(2,1);
// 
//                                                        Eigen::MatrixXd q(2,1);
// 
//                                                        Eigen::MatrixXd prev_mean_idx = prev_cells_[img_index][r_map][c_map].mean_idx_;
//                                                        cv::Point prev_mean_pt(cvRound(prev_mean_idx(0,0)),cvRound(prev_mean_idx(1,0)));
// #if USE_XY
//                                                        // Range/bearing based : X/Y
//                                                        cov = prev_cells_[img_index][r_map][c_map].covar_;
//                                                        prev_mean = prev_cells_[img_index][r_map][c_map].mean_;
//                                                        cov_inv = cov.inverse();
//                                                        q = point_mapped - prev_mean;
// #else
//                                                        cov = prev_cells_[img_index][r_map][c_map].covar_idx_;
//                                                        prev_mean = prev_cells_[img_index][r_map][c_map].mean_idx_;
//                                                        cov_inv = cov.inverse();
//                                                        q << recovered.x-rect.x, recovered.y-rect.y;
//                                                        q = q - prev_mean;
// #endif
//                                                        Eigen::MatrixXd diff(2,1);
//                                                        diff = prev_mean - point_mapped;
//                                                        transl_offset_sum += cv::Point2f(diff(0,0),diff(1,0));
//                                                        transl_offset_count++;
// 
//                                                        Eigen::MatrixXd q_t = q.transpose();
// 
//                                                        Eigen::MatrixXd qt_num = (-q_t*cov_inv*q);
//                                                        double exp_num = exp( qt_num(0,0) / 2 );
//                                                        new_score += exp_num;
//                                                   }
//                                              }
//                                              img_index++;
//                                         } // col_shift
//                                    } // row_shift
//                               } // check for non-zero
//                          } // cols
//                     } // rows
// 
//                     std::string line_img_name = "Line: " + syllo::int2str(iter);
//                     cv::imshow(line_img_name, line_img);
// 
//                     if (new_score > score) {
//                          lambda = lambda / 10.0;
//                          parameters = lm_parameters;
// 
//                          score_rotation = new_score;
// 
//                          transl_offset.x = transl_offset_sum.x / (double)transl_offset_count;
//                          transl_offset.y = transl_offset_sum.y / (double)transl_offset_count;
//                          cout << "=====> New Score: " << new_score << endl;
//                          cout << "Translation Offset: " << transl_offset << endl;
//                          cout << "Translation Offset Sum: " << transl_offset_sum << endl;
//                          cout << "transl_offset_count: " << transl_offset_count << endl;
//                     } else {
//                          lambda = lambda * 10;
//                     }
//                     cout << "lambda: " << lambda << endl;
//                }
//           } // iteration
// #if PRIORITY_ROTATE
// 
//           // Now that we have the "optimal" rotation and a calculation of the
//           // average offset, apply the offset and calculate the final score
//           // Recalculate score only!
// 
//           Eigen::MatrixXd lm_parameters;
//           lm_parameters = parameters;
//           lm_parameters(0,0) = transl_offset.x;
//           lm_parameters(1,0) = transl_offset.y;
// 
//           cout << "Scoring Final Parameters: " << lm_parameters << endl;
//           double final_score = 0;
//           cv::Mat line_img = prev_img_.clone();
//           cv::cvtColor(line_img, line_img, CV_GRAY2BGR);
//           cv::Mat iter_img = prev_img_.clone();
//           cv::cvtColor(iter_img, iter_img, CV_GRAY2BGR);
//           ////////////////////////////////
//           // For each non-zero value in newest scan...
//           for (int r = 0 ; r < src_copy.rows; r++) {
//                for (int c = 0; c < src_copy.cols; c++) {
//                     if (src_copy.at<uchar>(r,c) != 0) {
//                          cv::Point2f cart = get_cart(stream,r,c);
// 
//                          Eigen::MatrixXd sample_point(2,1);
//                          sample_point << cart.x, cart.y;
// 
//                          Eigen::MatrixXd point_mapped(2,1); //x'_i
//                          point_mapped = T_map(sample_point, lm_parameters);
//                          cart.x = point_mapped(0,0);
//                          cart.y = point_mapped(1,0);
// 
//                          double range_map, bearing_map;
//                          cart2polar(cart.x, cart.y, range_map, bearing_map);
// 
//                          cv::Point recovered = stream->get_pixel(range_map, bearing_map);
// 
//                          if (recovered.x < iter_img.cols && recovered.y < iter_img.rows && recovered.x >= 0 && recovered.y >= 0) {
//                               iter_img.at<cv::Vec3b>(recovered.y,recovered.x) = cv::Vec3b(0,255,0);
//                          } else {
//                               cout << "Scoring: Recovered outside of bounds==========> " << recovered << endl;
//                               //cout << "range_map: " << range_map << endl;
//                               //cout << "bearing_map: " << bearing_map << endl;
//                          }
//                          cv::line(line_img, cv::Point(c,r), recovered, cv::Scalar(255,0,0), 1, 8, 0);
//                          cv::circle(line_img, cv::Point(c,r), 1, cv::Scalar(0,255,0), 1, 8, 0);
//                          cv::circle(line_img, recovered, 1, cv::Scalar(0,0,255), 1, 8, 0);
// 
//                          int find_row = recovered.y;
//                          int find_col = recovered.x;
//                          img_index = 0;
//                          for (int row_shift = 0; row_shift < SHIFT; row_shift++) {
//                               for (int col_shift = 0; col_shift < SHIFT; col_shift++) {
//                                    unsigned int r_map = floor(((double)find_row + (double)row_shift*(double)cell_row_size_/2.0) / (double)cell_row_size_);
//                                    unsigned int c_map = floor(((double)find_col + (double)col_shift*(double)cell_col_size_/2.0) / (double)cell_col_size_);
// 
//                                    if (r_map < prev_cells_[img_index].size() && c_map < prev_cells_[img_index][r_map].size()) {
//                                         cv::Rect rect = prev_cells_[img_index][r_map][c_map].rect_;
// 
//                                         if (!contains(rect, cv::Point(find_col, find_row))) {
//                                              cout << "---------------------" << endl;
//                                              cout << "DOESN'T CONTAIN" << endl;
//                                              cout << "Rows: " << src_copy.rows << endl;
//                                              cout << "Cols: " << src_copy.cols << endl;
//                                              cout << "find_col: " << find_col << endl;
//                                              cout << "find_row: " << find_row << endl;
//                                              cout << "r_map: " << r_map << endl;
//                                              cout << "c_map: " << c_map << endl;
//                                              cout << "Img: " << img_index;
//                                              cout << " , rect: (" << rect.x << "," << rect.y << ") to ";
//                                              cout << "(" << rect.x+rect.width << "," << rect.y+rect.height << ")" << endl;
//                                              //std::string str;
//                                              //std::cin >> str;
//                                              continue;
//                                         }
// 
//                                         // Determine if this cell had a valid statistic
//                                         // If not, skip this one, add one to H_count?
//                                         if (prev_cells_[img_index][r_map][c_map].is_valid_) {
// 
//                                              Eigen::MatrixXd cov(2,2);
//                                              Eigen::MatrixXd cov_inv(2,2);
//                                              Eigen::MatrixXd prev_mean(2,1);
// 
//                                              Eigen::MatrixXd q(2,1);
// 
//                                              Eigen::MatrixXd prev_mean_idx = prev_cells_[img_index][r_map][c_map].mean_idx_;
//                                              cv::Point prev_mean_pt(cvRound(prev_mean_idx(0,0)),cvRound(prev_mean_idx(1,0)));
// #if USE_XY
//                                              // Range/bearing based : X/Y
//                                              cov = prev_cells_[img_index][r_map][c_map].covar_;
//                                              prev_mean = prev_cells_[img_index][r_map][c_map].mean_;
//                                              cov_inv = cov.inverse();
//                                              q = point_mapped - prev_mean;
// 
// #else
//                                              cov = prev_cells_[img_index][r_map][c_map].covar_idx_;
//                                              prev_mean = prev_cells_[img_index][r_map][c_map].mean_idx_;
//                                              cov_inv = cov.inverse();
//                                              q << recovered.x-rect.x, recovered.y-rect.y;
//                                              q = q - prev_mean;
// #endif
//                                              Eigen::MatrixXd q_t = q.transpose();
// 
//                                              Eigen::MatrixXd qt_num = (-q_t*cov_inv*q);
//                                              double exp_num = exp( qt_num(0,0) / 2 );
//                                              final_score += exp_num;
//                                         }
//                                    }
//                                    img_index++;
//                               } // col_shift
//                          } // row_shift
//                     } // check for non-zero
//                } // cols
//           } // rows
// 
//                                              cv::imshow("Final Points Image", iter_img);
//                                              cv::imshow("Final Line Image" , line_img);
// 
// #endif // PRIORITY_ROTATE
// 
//                                              if (final_score > score_rotation) {
//                                              parameters = lm_parameters;
//                                              cout << "==================> Final Score is higher" << endl;
//                                         }
// 
//           cout << "-------" << endl;
//           cout << "Final Parameters: " << parameters << endl;
//           cout << "Translation (Final) Score: " << final_score << endl;
// 
//           xy_est_ = T_map(xy_est_, parameters);
//           phi_est_ = phi_est_ + parameters(2,0);
// 
//           cout << "Adjusted x_est: " << xy_est_(0,0) << endl;
//           cout << "Adjusted y_est: " << xy_est_(1,0) << endl;
//           cout << "Adjusted phi_est: " << phi_est_ << endl;
//           cout << "Adjusted phi_est (degrees):" << phi_est_ * 180.0 / 3.14159265359 << endl;
//      } //prev_cells_.size() > 0 check
// 
//      prev_ndt_img_ = dst.clone();
//      prev_img_ = src.clone();
//      cells_.swap(prev_cells_);
}
