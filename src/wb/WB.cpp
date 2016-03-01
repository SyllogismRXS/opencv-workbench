#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/wb/WB.h>
#include <opencv_workbench/wb/Point.h>
#include <opencv_workbench/wb/Cluster.h>

#include <list>
#include <vector>

using std::cout;
using std::endl;

namespace wb {     

     #define PI 3.14159265359

     double gaussian_probability(Eigen::MatrixXd &x, 
                                 Eigen::MatrixXd &u, 
                                 Eigen::MatrixXd &cov)
     {
          double n = u.rows();
          double deter = cov.determinant();
          Eigen::MatrixXd cov_inv = cov.inverse();
          Eigen::MatrixXd diff = x - u;          
          
          double norm = 1.0 / ( pow(2*PI,n/2.0) * pow(deter,0.5) );
          //cout << "norm: " << norm << endl;
          Eigen::MatrixXd numerator = diff.transpose() * cov_inv * diff;
          //cout << "numerator: " << numerator << endl;
          return norm * exp(-0.5 * numerator(0,0));
     }
     
     void cluster_points(cv::Mat &src, std::list<wb::Cluster*> &clusters, 
                         int thresh, float gate, int min_cluster_size)
     {             
          CV_Assert(src.depth() != sizeof(uchar));
          int channels = src.channels();
          int nRows = src.rows;
          int nCols = src.cols * channels;          

          // Find all points that are greater than some threshold (zero, usually).
          // Organize into array of points. (cv::Point, value)
          std::vector<wb::Point> points;
          int i,j;
          uchar* p;                         
          for( i = 0; i < nRows; ++i) {
               p = src.ptr<uchar>(i);
               for ( j = 0; j < nCols; ++j) {
                    if (p[j] > thresh) {
                         wb::Point point;
                         point.set_position(cv::Point(j,i));
                         point.set_value(p[j]);
                         points.push_back(point);
                    }
               }
          }

          // Outer-loop through all points
          int cluster_id = 1;        
          std::vector<wb::Point>::iterator it1 = points.begin();
          for (; it1 != points.end(); it1++) {
               std::vector<wb::Point>::iterator it2 = points.begin();
               for (; it2 != points.end(); it2++) {     

                    // Skip if this is the same point:
                    if (it1->position() == it2->position()) {
                         continue;
                    }
                  
                    // Calculate distance between all outer-loop points and
                    // inner-loop points
                    float dist = it1->distance(*it2);
                    if ( dist <= gate) {
                         // Only assign this pixel to the new cluster if it 
                         // hasn't been assigned or if it's old distance is
                         // greater than the new distance
                         if (!(it2->assigned()) || it2->distance() >= dist ) {
                              // If the outer loop's point hasn't been assigned
                              // to a cluster yet, create one.
                              if (!(it1->assigned())) {
                                   Cluster *c = new Cluster;
                                   c->set_id(cluster_id++);
                                 
                                   it1->set_assigned(true);
                                   it1->set_distance(dist);
                                   it1->set_parent(c);

                                   // If it2 is assigned make sure to remove it
                                   // from old parent
                                   if (it2->assigned()) {
                                        Cluster *c_temp = it2->parent();
                                        if (c_temp != NULL) {
                                             c_temp->remove_point(*it2);
                                        } else {
                                             cout << "Point is assigned, but has no parent" << endl;
                                             cout << "Point: " << it2->position() << endl;
                                        }
                                   }
                                 
                                   it2->set_assigned(true);
                                   it2->set_distance(dist);
                                   it2->set_parent(c);
                                 
                                   c->add_point(*it1);
                                   c->add_point(*it2);
                                   clusters.push_back(c);
                              } else {
                                   // Assign the inner loop's point to the
                                   // outer loop's cluster
                                   if (dist <= it1->distance()) {
                                        it1->set_distance(dist);
                                   }
                                 
                                   if (it2->assigned()) {
                                        Cluster *c = it2->parent();
                                        if (c != NULL) {
                                             c->remove_point(*it2);
                                        } else {
                                             cout << "Point is assigned, but has no parent" << endl;
                                             cout << "Point: " << it2->position() << endl;
                                        }
                                   }                                    
                                 
                                   it2->set_assigned(true);
                                   it2->set_distance(dist);
                                   it2->set_parent(it1->parent());
                                 
                                   it1->parent()->add_point(*it2);
                              }
                         } 
                    }
               }
          }          
        
          // Loop through all clusters. remove small clusters, draw points in 
          // remaining clusters on image
          cv::Mat cluster_img = cv::Mat::zeros(src.size(), src.type());
          std::list<wb::Cluster*>::iterator it = clusters.begin();
          int cluster_color_count = 2;
          while(it != clusters.end()) {
               // Remove small clusters
               if ((*it)->size() < min_cluster_size) {
                    delete *it;
                    it = clusters.erase(it);
                    //} else {
                    //it++;
               }
               else {
                    std::vector<Point> points = (*it)->points();
                    std::vector<Point>::iterator it_p = points.begin();                                                            
                  
                    for (; it_p != points.end(); it_p++) {
                         cluster_img.at<uchar>(it_p->position().y, it_p->position().x) = cluster_color_count;
                    }                                                 
                    cluster_color_count += 10;
                    it++;
               }
          }    

          //cout << "Cluster count: " << clusters.size() << endl;
        
          cv::imshow("clusters", cluster_img);          
          cv::Mat cluster_display;
          cv::normalize(cluster_img, cluster_display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
          cv::applyColorMap(cluster_display, cluster_display, cv::COLORMAP_JET);          
         
          // Draw cluster labels
          for (it = clusters.begin(); it != clusters.end(); it++) {
               //Point *pref = (*it)->points().front();                                   
               //cv::Point circle_point = cv::Point(pref->position().y, pref->position().x);
               (*it)->compute_metrics();
               cv::Point circle_point = (*it)->centroid();
               //cout << "--------" << endl;
               //cout << circle_point.x << endl;
               //cout << circle_point.y << endl;
           
               std::ostringstream convert;
               convert << (*it)->id();
               const std::string& text = convert.str();
               cv::circle(cluster_display, circle_point, 1, cv::Scalar(255,255,255), -1, 8, 0);
               cv::rectangle(cluster_display, (*it)->bbox().rectangle(), cv::Scalar(100,100,100), 1, 8, 0);
               cv::putText(cluster_display, text, circle_point, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1, 8, false);                                                            
          }

           
#if 0
          bool wait = false;
          // Do any points in a cluster, belong in another cluster?
          it = clusters.begin();
          for (; it != clusters.end(); it++) {                              
               std::list<wb::Cluster*>::iterator it2 = clusters.begin();
               for (; it2 != clusters.end(); it2++) {
                    // Continue if looking at same cluster
                    if (it == it2) {
                         continue;
                    }                    
                    std::vector<wb::Point>::iterator it_points1 = (*it)->points().begin();
                    for (; it_points1 != (*it)->points().end(); it_points1++) {
                         std::vector<wb::Point>::iterator it_points2 = (*it2)->points().begin();
                         for (; it_points2 != (*it2)->points().end(); it_points2++) {
                              if ( it_points1->position() == it_points2->position()) {
                                   cout << "----------------" << endl;
                                   cout << "Cluster IDs: " << (*it)->id() << ", " << (*it2)->id() << endl;
                                   cout << "Point1: " << it_points1->position().x << ", " << it_points1->position().y << endl;
                                   cout << "Point2: " << it_points2->position().x << ", " << it_points2->position().y << endl;
                                   // cluster_display.at<uchar>((*it_points2)->position().y,(*it_points2)->position().x)(0) = 0;
                                   cluster_display.at<cv::Vec3b>(it_points2->position().y,it_points2->position().x) = cv::Vec3b(0,0,0);
                                   
                                   cv::circle(cluster_display, cv::Point(it_points2->position().x,it_points2->position().y), 5, cv::Scalar(0,0,255), 1, 8, 0);
                                   wait = true;                                   
                              }
                         }
                    }
               }
          }
#endif

          cv::imshow("clusters display", cluster_display);
           
          //cv::Mat large;
          //cv::resize(cluster_display, large, cv::Size(0,0), 2, 2, cv::INTER_LINEAR );
          //cv::imshow("large", large);
           
          //if (wait) { 
          //     cv::waitKey(0);
          //}                   
     }

     void draw_clusters(cv::Mat &src, cv::Mat &dst, 
                        std::list<wb::Cluster*> &clusters)
     {
          dst = src.clone();
          
          std::list<wb::Cluster*>::iterator it = clusters.begin();
          for(; it != clusters.end(); it++) {
               
               std::vector<Point> points = (*it)->points();
               std::vector<Point>::iterator it_p = points.begin();                                                            
                    
               for (; it_p != points.end(); it_p++) {
                    //dst.at<cv::Vec3b>((*it_p)->position().y, (*it_p)->position().x) = cv::Vec3b(0,255,0);
               }                                                 
          }

          //cv::Mat cluster_display;
          //cv::normalize(cluster_img, cluster_display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
          //cv::applyColorMap(cluster_display, cluster_display, cv::COLORMAP_JET);          
          
          // Draw cluster labels
          for (it = clusters.begin(); it != clusters.end(); it++) {
               (*it)->compute_metrics();
               cv::Point circle_point = (*it)->centroid();
               //cout << "--------" << endl;
               //cout << circle_point.x << endl;
               //cout << circle_point.y << endl;
          
               std::ostringstream convert;
               convert << (*it)->id();
               const std::string& text = convert.str();
               cv::circle(dst, circle_point, 1, cv::Scalar(255,255,255), -1, 8, 0);
               cv::rectangle(dst, (*it)->bbox().rectangle(), cv::Scalar(100,100,100), 1, 8, 0);
               cv::putText(dst, text, circle_point, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1, 8, false);
          }
     }   

     void showHistogram(const cv::Mat& img, cv::Mat &mask)
     {
          int bins = 256;             // number of bins
          int nc = img.channels();    // number of channels

          std::vector<cv::Mat> hist(nc);       // histogram arrays

          // Initalize histogram arrays
          for (unsigned int i = 0; i < hist.size(); i++) {
               hist[i] = cv::Mat::zeros(1, bins, CV_32SC1);
          }          
          
          // Calculate the histogram of the image
          for (int i = 0; i < img.rows; i++) {
               for (int j = 0; j < img.cols; j++) {
                    for (int k = 0; k < nc; k++) {
                         if (mask.at<uchar>(i,j) != 0) {
                              uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<cv::Vec3b>(i,j)[k];
                              hist[k].at<int>(val) += 1;
                         }
                    }
               }
          }

          // For each histogram arrays, obtain the maximum (peak) value
          // Needed to normalize the display later
          int hmax[3] = {0,0,0};
          for (int i = 0; i < nc; i++) {
               for (int j = 0; j < bins-1; j++) {
                    hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
               }
          }

          const char* wname[3] = { "blue", "green", "red" };
          cv::Scalar colors[3] = { cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255) };

          std::vector<cv::Mat> canvas(nc);

          // Display each histogram in a canvas
          for (int i = 0; i < nc; i++) {
               canvas[i] = cv::Mat::ones(125, bins, CV_8UC3);

               for (int j = 0, rows = canvas[i].rows; j < bins-1; j++) {
                    cv::line(
                         canvas[i], 
                         cv::Point(j, rows), 
                         cv::Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])), 
                         nc == 1 ? cv::Scalar(200,200,200) : colors[i], 
                         1, 8, 0
                         );                    
               }
               cv::imshow(nc == 1 ? "value" : wname[i], canvas[i]);
          }      
#if 0    
          cv::Mat maxes = img.clone();
          cv::cvtColor(maxes,maxes,CV_GRAY2BGR);
          // Highlight the pixels that part of the largest histogram bin
          for (int i = 0; i < img.rows; i++) {
               for (int j = 0; j < img.cols; j++) {
                    for (int k = 0; k < nc; k++) {
                         if (mask.at<uchar>(i,j) != 0) {
                              uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<cv::Vec3b>(i,j)[k];
                              if (hist[k].at<int>(val) == hmax[0]) {
                                   maxes.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,255);
                              }
                         }
                    }
               }
          }
          cv::imshow("maxes-c", maxes);       
#endif          
     }

     void opencv_histogram(cv::Mat &src)
     {
          /// Separate the image in 3 places ( B, G and R )
          std::vector<cv::Mat> bgr_planes;
          cv::split( src, bgr_planes );

          /// Establish the number of bins
          int histSize = 256;

          /// Set the ranges ( for B,G,R) )
          float range[] = { 0, 256 } ;
          const float* histRange = { range };

          bool uniform = true; bool accumulate = false;

          cv::Mat b_hist, g_hist, r_hist;

          /// Compute the histograms:
          cv::calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
          cv::calcHist( &bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
          cv::calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

          // Draw the histograms for B, G and R
          int hist_w = 512; int hist_h = 400;
          int bin_w = cvRound( (double) hist_w/histSize );

          cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

          /// Normalize the result to [ 0, histImage.rows ]
          cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
          cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
          cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

          /// Draw for each channel
          for (int i = 1; i < histSize; i++) {
               cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                         cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                         cv::Scalar( 255, 0, 0), 2, 8, 0  );
               cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                         cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                         cv::Scalar( 0, 255, 0), 2, 8, 0  );
               cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                         cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                         cv::Scalar( 0, 0, 255), 2, 8, 0  );
          }

          /// Display
          cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
          cv::imshow("calcHist Demo", histImage );
     }

     void get_sonar_mask(const cv::Mat &jet, cv::Mat &mask)
     {
          mask = cv::Mat(jet.size(), CV_8UC1);

          // accept only char type matrices
          CV_Assert(mask.depth() != sizeof(uchar));

          int channels = mask.channels();
          int nRows = mask.rows;
          int nCols = mask.cols * channels;

          if (mask.isContinuous()) {
               nCols *= nRows;
               nRows = 1;
          }

          int i,j;
          uchar* p;     
          for (i = 0; i < nRows; ++i) {
               p = mask.ptr<uchar>(i);
               for (j = 0; j < nCols; ++j) {
                    cv::Vec3b pix = jet.at<cv::Vec3b>(i,j);
                    if (pix[0] == 0 && pix[1] == 0 && pix[2] == 0) {
                         p[j] = 0;
                    } else {
                         p[j] = 1;
                    }
               }
          }
     }
     
     void adaptive_threshold(cv::Mat &src, cv::Mat& dst, int &thresh, double ratio_low, double ratio_high, int thresh_step, int max_iter)
     {
          // accept only char type matrices
          CV_Assert(src.depth() != sizeof(uchar));        
          bool ratio_achieved = false;     
          int iter_count = 0;
          //cout << "--------" << endl;
          double ratio = 0;
          do {                              
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
               ratio = 0;
               if (count != 0) {
                    ratio = (double)count / ((double)(nRows*nCols));
               }
               
               //cout << "Ratio: " << ratio << endl;
               if (ratio > ratio_low && ratio < ratio_high) {
                    ratio_achieved = true;
               } else if (ratio < ratio_low) {
                    thresh -= thresh_step;
               } else if (ratio > ratio_high) {
                    thresh += thresh_step;
               }               

               if (thresh < 0) {
                    thresh = 0;
                    ratio_achieved = true;
               } else if (thresh > 255) {
                    ratio_achieved = true;
                    thresh = 255;
               }

               //cout << "Trying: Thresh: " << thresh << endl;
               cv::threshold(src, dst, thresh, 255, cv::THRESH_TOZERO);              
               
               iter_count++;          

          }while(!ratio_achieved && iter_count < max_iter);

          #if 1
               std::fstream file;
               file.open("/home/syllogismrxs/temp/adap-thresh.txt", std::fstream::in | std::fstream::out | std::fstream::app);
               file << ratio_low << " " << ratio_high << " " << ratio << " " << thresh << endl;
               file.close();
#endif
     }

#define USE_SCHARR 0
     void gradient_sobel(cv::Mat &src, cv::Mat &dst)
     {
          cv::Mat grad_x, grad_y;
          cv::Mat abs_grad_x, abs_grad_y;
          cv::Mat grad;
          
          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;
     
#if USE_SCHARR
          cv::Scharr(src, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT);
          cv::Scharr(src, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT);
#else
          cv::Sobel(src, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
          cv::Sobel(src, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
#endif                         
          cv::convertScaleAbs( grad_x, abs_grad_x );
          cv::convertScaleAbs( grad_y, abs_grad_y );
     
          cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst );
     }

     void gradient_simple(cv::Mat &src, cv::Mat &dst)
     {
          CV_Assert(src.depth() != sizeof(uchar));
          
          int ksize = 15;
          
          cv::Mat I;
          cv::copyMakeBorder(src, I, 0, 0, ksize/2, ksize/2, cv::BORDER_REPLICATE);
          
          dst = cv::Mat::zeros(I.size(), I.type());

          int channels = I.channels();
          int nRows = I.rows;
          int nCols = I.cols * channels;          

          int i,j;
          uchar *p;
          uchar *d;
          for( i = 0; i < nRows; ++i) {
               p = I.ptr<uchar>(i);
               d = dst.ptr<uchar>(i); 
               for ( j = 0; j < nCols; ++j) {
                    if (j >= ksize) {
                         int sum = 0;
                         int count = 0;
                         for (int k = 1; k <= ksize; k++) {
                              sum += p[j-k];
                              sum += I.ptr<uchar>(i-k)[j];
                              count++;
                         }
                         double mean = (double)sum / ((double)count*2.0);
                         d[j] = abs(p[j] - mean);
                    } else {
                         d[j] = p[j];
                    }
               }
          }
          //cv::Rect rect = cv::Rect(ksize/2, ksize/2, src.rows, src.cols);
          //dst = cv::Mat(dst, rect);          
     }


     double nonzero_ratio(cv::Mat &img, cv::Mat &mask)
     {
          int nRows = img.rows;
          int nCols = img.cols;

          // Compute Ratio
          int count = 0;
          int count_in_mask = 0;
          int i,j;
          uchar *p, *m;               
          for( i = 0; i < nRows; ++i) {
               p = img.ptr<uchar>(i);
               m = mask.ptr<uchar>(i);                    
               for ( j = 0; j < nCols; ++j) {
                    if (p[j] > 0 && m[j] > 0) {
                         count++;
                    }
                    if (m[j] > 0) {
                         count_in_mask++;
                    }
               }
          }

          double ratio = 0;
          if (count != 0) {
               ratio = (double)count / (double)count_in_mask;
          }
          return ratio;
     }

     void mean_stddev(cv::Mat &img, cv::Mat &mask, double &mean, double &std)
     {
          cv::Mat mean_mat, stddev_mat;          
          cv::meanStdDev(img, mean_mat, stddev_mat, mask);
          mean = mean_mat.at<double>(0,0);
          std = stddev_mat.at<double>(0,0);
     }
     
     // Second version
     void adaptive_threshold(cv::Mat &src, cv::Mat& dst, int &thresh, 
                             double desired_ratio, int max_iter, cv::Mat &mask)
     {
          //cout << "==============================" << endl;

          // accept only char type matrices
          CV_Assert(src.depth() != sizeof(uchar));          

          bool ratio_achieved = false;     
          int iter_count = 0;
          cv::Mat src_copy = src.clone();
          
          // // First compute the image properties          
          // // Compute mean and stdev before threshold
          // double prev_mean, prev_std, prev_ratio;
          // 
          // mean_stddev(src_copy,mask,prev_mean,prev_std);          
          // prev_ratio = nonzero_ratio(src_copy,mask);
          // cout << "Mean: " << prev_mean << endl;
          // cout << "StdDev: " << prev_std << endl;   
          // cout << "Ratio: " << prev_ratio << endl;                    
          
          do {                                             
               //cout << "-----------------" << endl;
               //cout << "Thresh: " << thresh << endl;
               src_copy = src.clone();              

               // Apply threshold
               cv::threshold(src_copy, src_copy, thresh, 255, cv::THRESH_TOZERO);

               // First compute the image properties          
               // Compute mean and stdev before threshold
               //double mean, std;
               double ratio;          
               //mean_stddev(src_copy,mask,mean,std);          
               ratio = nonzero_ratio(src_copy,mask);
               //cout << "Mean: " << mean << endl;
               //cout << "StdDev: " << std << endl;   
               //cout << "Ratio: " << ratio << endl;
               //
               //cout << "Change Mean: " << mean-prev_mean << endl;
               //cout << "Change Std: " << std-prev_std << endl;               
               //cout << "Change Ratio: " << ratio-prev_ratio << endl;
               
               double ratio_error = ratio - desired_ratio;
               if (std::abs(ratio_error) < 0.00001) { 
                    ratio_achieved = true;                    
               }
               //cout << "Ratio Error: " << ratio_error << endl;

               thresh += ratio_error * 1e3;

               if (thresh < 0) {
                    thresh = 0;                    
               } else if (thresh > 255) {                    
                    thresh = 255;
               }

               //cv::imshow("Src Copy", src_copy);
               //cv::waitKey(0);

               //prev_mean = mean;
               //prev_std = std;
               //prev_ratio = ratio;
               
               iter_count++;                                        
          }while(!ratio_achieved && iter_count < max_iter);         
          dst = src_copy;
     }

     using namespace cv;
     
     double getThreshVal_Otsu_8u( Mat& _src , cv::Mat &_mask )
     {
          Size size = _src.size();
          int step = (int) _src.step;
          
          const int N = 256;
          int i, j, h[N] = {0};
          int in_mask_count = 0;
          for( i = 0; i < size.height; i++ )
          {
               const uchar* src = _src.ptr() + step*i;
               const uchar* mask = _mask.ptr() + step*i;
               j = 0;
               for( ; j < size.width; j++ ) {
                    if (mask[j] > 0) {
                         h[src[j]]++;
                         in_mask_count++;
                    }
               }
          }

          //double mu = 0, scale = 1./(size.width*size.height);
          double mu = 0, scale = 1./(double)in_mask_count;
          for( i = 0; i < N; i++ ) {
               mu += i*(double)h[i];
          }

          mu *= scale;
          double mu1 = 0, q1 = 0;
          double max_sigma = 0, max_val = 0;

          for( i = 0; i < N; i++ )
          {
               double p_i, q2, mu2, sigma;

               p_i = h[i]*scale;
               mu1 *= q1;
               q1 += p_i;
               q2 = 1. - q1;

               if( std::min(q1,q2) < FLT_EPSILON || std::max(q1,q2) > 1. - FLT_EPSILON )
                    continue;

               mu1 = (mu1 + i*p_i)/q1;
               mu2 = (mu - q1*mu1)/q2;
               sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
               if( sigma > max_sigma )
               {
                    max_sigma = sigma;
                    max_val = i;
               }
          }

          return max_val;
     }

     void multi_otsu(cv::Mat &_src, cv::Mat &_dst, cv::Mat &_mask, cv::Mat &_first)
     {
          // Compute first level of OTSU's method
          double val = wb::getThreshVal_Otsu_8u(_src, _mask);
          cv::Mat thresh_img1;
          cv::threshold(_src, thresh_img1, val, 255, cv::THRESH_TOZERO);

          //cv::threshold(_src, _first, val, 255, THRESH_TOZERO_INV);          
          _first = thresh_img1;

          //cv::imshow("First OTSU", thresh_img1);
          //cv::waitKey(0);

          // Get New Mask
          int nRows = _mask.rows;
          int nCols = _mask.cols;
          
          cv::Mat mask_2 = cv::Mat(_mask.size(), CV_8UC1);
          int i,j;
          uchar *p, *m;
          for (i = 0; i < nRows; ++i) {
               p = thresh_img1.ptr<uchar>(i);
               m = mask_2.ptr<uchar>(i);
               for (j = 0; j < nCols; ++j) {
                    if (p[j] == 0) {
                         m[j] = 0;
                    } else {
                         m[j] = 1;
                    }                                        
               }
          }

          //cv::imshow("Second mask", mask_2*255);
          //cv::waitKey(0);
          
          // Second OTSU Level
          cv::Mat second_otsu;
          val = wb::getThreshVal_Otsu_8u(thresh_img1, mask_2);
          cv::threshold(thresh_img1, second_otsu, val, 255, cv::THRESH_TOZERO);

          _dst = second_otsu.clone();

          ///////////
          // AGAIN
          // Get New Mask                    
          mask_2 = cv::Mat(_mask.size(), CV_8UC1);
          //int i,j;
          //uchar *p, *m;
          for (i = 0; i < nRows; ++i) {
               p = second_otsu.ptr<uchar>(i);
               m = mask_2.ptr<uchar>(i);
               for (j = 0; j < nCols; ++j) {
                    if (p[j] == 0) {
                         m[j] = 0;
                    } else {
                         m[j] = 1;
                    }                                        
               }
          }

          //cv::imshow("Second mask", mask_2*255);
          //cv::waitKey(0);
          
          // Second OTSU Level
          cv::Mat third_otsu;
          val = wb::getThreshVal_Otsu_8u(second_otsu, mask_2);
          cv::threshold(second_otsu, third_otsu, val, 255, cv::THRESH_TOZERO);

          _dst = third_otsu.clone();
          
          //cv::imshow("Second OTSU", _dst);
          //cv::waitKey(0);
     }
}

// KMEAN OPENCV Clustering
///// cluster the points
///cv::Mat src = thresh;//grad_thresh;
///cv::Mat samples(src.rows * src.cols, 1, CV_32F);
///for( int y = 0; y < src.rows; y++ ) {
///     for( int x = 0; x < src.cols; x++ ) {
///          //for( int z = 0; z < 3; z++) {
///          samples.at<float>(y + x*src.rows, 0) = src.at<uchar>(y,x); //src.at<cv::Vec3b>(y,x)[z];
///          //}
///     }
///}
///
///int clusterCount = 15;
///cv::Mat labels;
///int attempts = 2;
///cv::Mat centers;
///cv::kmeans(samples, clusterCount, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers );
///
///cv::Mat new_image( src.size(), src.type() );
///for( int y = 0; y < src.rows; y++ ) {
///     for( int x = 0; x < src.cols; x++ ) { 
///          int cluster_idx = labels.at<int>(y + x*src.rows,0);
///          new_image.at<uchar>(y,x) = centers.at<float>(cluster_idx);
///          //new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
///          //new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
///          //new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
///     }
///}
///
///cv::normalize(new_image, new_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
///cv::applyColorMap(new_image, new_image, cv::COLORMAP_JET);
/////cv::imshow("centers", centers);
///cv::imshow("clusters", new_image);    
