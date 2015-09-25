#include <iostream>

#include <opencv_workbench/wb/Point.h>

#include "BlobProcess.h"

using std::cout;
using std::endl;

BlobProcess::BlobProcess()
{
     min_blob_size_ = 30;
}

uchar BlobProcess::valueAt(cv::Mat &img, int row, int col)
{
     if (col >= 0 && col < img.cols) {
	  if (row >= 0 && row < img.rows) {
	       return img.at<uchar>(row,col);
	  }
     } 
     return 0;
}
     
uchar BlobProcess::findMin(uchar NE, uchar N, uchar NW, uchar W)
{
     int minChamp = 9999;

     if (NE == 0 && N == 0 && NW == 0 && W == 0) {
	  return 0;
     }

     if(NE != 0 && NE < minChamp) {
	  minChamp = NE;
     }

     if(N != 0 && N < minChamp) {
	  minChamp = N;
     }

     if(NW != 0 && NW < minChamp) {
	  minChamp = NW;
     }

     if(W != 0 && W < minChamp) {
	  minChamp = W;
     }

     return minChamp;
}

void BlobProcess::labelNeighbors(cv::Mat &img, std::vector<uchar> &labelTable, 
                                 uchar label, int i, int j)
{
     uchar value;

     value = valueAt(img,i-1,j+1);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i-1,j+1) = label;
     }

     value = valueAt(img,i-1,j);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i-1,j) = label;
     }

     value = valueAt(img,i-1,j-1);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i-1,j-1) = label;
     }

     value = valueAt(img,i,j-1);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i,j-1) = label;
     }		    
}

int BlobProcess::process_frame(cv::Mat &input)
{
     blobs_.clear();
     
     std::vector<uchar> labelTable;
     labelTable.push_back(0);

     cv::Mat img;
     input.copyTo(img);
     
     uchar label = 1;

     uchar NE, N, NW, W;

     for( int i = 0; i < img.rows; ++i) {
          for( int j = 0; j < img.cols; ++j ) {
               if (img.at<uchar>(i,j) != 0) {
                    NE = valueAt(img,i-1,j+1);
                    N  = valueAt(img,i-1,j);
                    NW = valueAt(img,i-1,j-1);
                    W  = valueAt(img,i,j-1);
                    
                    uchar value = findMin(NE,N,NW,W);

                    if (value == 0) {
                         img.at<uchar>(i,j) = label;
                         labelTable.push_back(label);
                         label++;
                    } else {
                         img.at<uchar>(i,j) = value;
                         labelNeighbors(img, labelTable, value, i, j);
                    }
               }
          }
     }
          
     // Second pass to fix connected components that have different labels
     for( int i = 0; i < img.rows; ++i) {
          for( int j = 0; j < img.cols; ++j ) {
               if (img.at<uchar>(i,j) != 0) {
                    int id = labelTable[img.at<uchar>(i,j)];
                    //img.at<uchar>(i,j) = id;

                    // If the ID is new, add it to the blob map
                    if (blobs_.count(id) == 0) {
                         wb::Blob blob(id);
                         blobs_[id] = blob;
                    }
                    wb::Point p;
                    p.set_position(cv::Point(j,i));
                    blobs_[id].add_point(p);
               }
          }
     }     

     //////////////////////////////////////////////////////////////////////////
     // Remove small blobs
     //////////////////////////////////////////////////////////////////////////
     std::map<int,wb::Blob>::iterator it = blobs_.begin();
     while(it != blobs_.end()) {
          if (it->second.size() < min_blob_size_) {
               //delete it;
               //it = blobs_.erase(it);
               blobs_.erase(it++);
          } else {
               it->second.compute_metrics();
               it++;
          }
     }    

     //std::map<int,syllo::Blob>::iterator it;
     //for(it=blobs_.begin();it!=blobs_.end();it++)
     //{
     //     printf("id: %d \t size:%d\n",it->first,it->second.getSize());
     //}

     //output = reduced;
     return blobs_.size();
}

void BlobProcess::overlay_blobs(cv::Mat &src, cv::Mat &dst)
{
     cv::Mat color;
     cv::cvtColor(src, color, CV_GRAY2BGR);     
     dst = color;
     
     std::map<int,wb::Blob>::iterator it = blobs_.begin();
     for(; it != blobs_.end(); it++) {
          // Draw all blob points in the image
          std::vector<wb::Point> points = it->second.points();
          std::vector<wb::Point>::iterator it_points = points.begin();
          for(; it_points != points.end(); it_points++) {                    
               dst.at<cv::Vec3b>(it_points->y(), it_points->x()) = cv::Vec3b(20,255,57);
          }

          cv::Point centroid_point = it->second.centroid();
          cv::Rect rect = it->second.rectangle();
               
          std::ostringstream convert;
          convert << it->second.id();               
          const std::string& text = convert.str();
          
          cv::circle(dst, centroid_point, 1, cv::Scalar(255,255,255), -1, 8, 0);
          cv::rectangle(dst, rect, cv::Scalar(255,255,255), 1, 8, 0);
          cv::putText(dst, text, cv::Point(rect.x-3,rect.y-3), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 1, 8, false);
     }
}
