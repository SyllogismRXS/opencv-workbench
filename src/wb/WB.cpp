#include <iostream>
#include <sstream>

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

     
     
     void adaptive_threshold(cv::Mat &src, cv::Mat& dst, int &thresh, double ratio_low, double ratio_high, int thresh_step, int max_iter)
     {
          // accept only char type matrices
          CV_Assert(src.depth() != sizeof(uchar));        
          bool ratio_achieved = false;     
          int iter_count = 0;
          //cout << "--------" << endl;
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
               double ratio = (double)count / ((double)(nRows*nCols));
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
