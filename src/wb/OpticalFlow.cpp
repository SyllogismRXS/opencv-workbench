#include <iostream>

#include "OpticalFlow.h"

using std::cout;
using std::endl;

OpticalFlow::OpticalFlow()
{
     needToInit = true;
     MAX_COUNT = 500;
     termcrit = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
     subPixWinSize = cv::Size(10,10);
     winSize = cv::Size(31,31);
}

void OpticalFlow::drawOptFlowMap(const cv::Mat &flow, cv::Mat &cflowmap, int step,
                                 double scale, cv::Scalar color)
{
     cv::Mat_<cv::Vec2f> _flow = flow;
     for( int y = 0; y < flow.rows; y += step) {
	  for( int x = 0; x < flow.cols; x += step) {
	       cv::line(cflowmap, cv::Point(x,y), cv::Point(x + _flow(y,x)[0] , y + _flow(y,x)[1]), color, 1, 8, 0);
	       //cv::circle(cflowmap, cv::Point(x,y), 1, color, 1, 8, 0);
	  }
     }
}

int OpticalFlow::dense_flow(const cv::Mat &input_gray, cv::Mat &output)
{
     if (prev_gray.size() == input_gray.size()) {
          cv::Mat flow;
          cv::calcOpticalFlowFarneback(prev_gray, input_gray, flow, 0.5, 3, 5, 3, 5, 1.2, 0);     
          output = cv::Mat::ones(flow.size(), input_gray.type());
          drawOptFlowMap(flow, output, 10, 1, cv::Scalar(255,255,0));     
     } else {
          output = cv::Mat::ones(input_gray.size(), input_gray.type());
     }
     prev_gray = input_gray.clone();
     
     return 0;
}

int OpticalFlow::sparse_flow(cv::Mat &input_gray, cv::Mat &output)
{
     output = input_gray.clone();
          
     if( needToInit ) {
          // automatic initialization
          cv::goodFeaturesToTrack(input_gray, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
          cv::cornerSubPix(input_gray, points[1], subPixWinSize, cv::Size(-1,-1), termcrit);
     } else if( !points[0].empty() ) {
          std::vector<uchar> status;
          std::vector<float> err;
          if(prev_gray.empty())
               input_gray.copyTo(prev_gray);
               
          cv::calcOpticalFlowPyrLK(prev_gray, input_gray, points[0], points[1], status, err, winSize,
                                   3, termcrit, 0, 0.001);
          size_t i, k;
          for( i = k = 0; i < points[1].size(); i++ )
          {                              
               if( !status[i] ) {
                    continue;
               }
                    
               points[1][k++] = points[1][i];
               cv::circle( output, points[1][i], 3, cv::Scalar(0,255,0), -1, 8);
          }
          points[1].resize(k);
     }          

     needToInit = false;
     
     std::swap(points[1], points[0]);
     prev_gray = input_gray.clone();

     return 0;
}

