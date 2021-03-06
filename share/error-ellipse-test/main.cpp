#include <iostream>
#include <cmath>

//You need OpenCV for this demo
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/utils/Ellipse.h>

cv::RotatedRect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat);

using std::endl;
using std::cout;

int main()
{
		
     //Covariance matrix of our data	
     cv::Mat covmat = (cv::Mat_<double>(2,2) << 500.5886, 400.6111, 400.6111, 500.7801);	
	
     //The mean of our data
     cv::Point2f mean(100,150);

     //Calculate the error ellipse for a given confidence intervanl
     //cv::RotatedRect ellipse = getErrorEllipse(2.4477, mean, covmat); // 95%
     //cv::RotatedRect ellipse = getErrorEllipse(3.0349, mean, covmat); // 99%
     cv::RotatedRect ellipse = getErrorEllipse(2.4860, mean, covmat); // 95.45%

     //Show the result
     cv::Mat visualizeimage(240, 320, CV_8UC3, cv::Scalar::all(0));	
     cv::ellipse(visualizeimage, ellipse, cv::Scalar::all(255), 2);
     
     ////////////////////////////////////////////////////////////
     //// Create second to test:
     Eigen::MatrixXf F,B,H,Q,R,x,P;
     
     H.resize(2,4);
     H << 1, 0, 0, 0,
          0, 1, 0, 0;
     
     P.resize(4,4);
     //P << 500.5886, 400.6111, 400.6111, 500.7801;
     P << 500.5886, 400.6111, 0, 0,
          400.6111, 500.7801, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

     R.resize(2,2);
     R << 0,0,0,0;
     
     x.resize(4,1);
     x << 100,150,0,0;
     
     syllo::KalmanFilter kf_(F,B,H,Q,R);
     kf_.init(x,P);
          
     //Ellipse ell = kf_.error_ellipse(0.9545);
     Ellipse ell = kf_.error_ellipse(0.9545);
     cv::Point center = ell.center();
     cv::Size axes(ell.axes()(0), ell.axes()(1));
     cv::ellipse(visualizeimage, center, axes, ell.angle(), 0, 360, cv::Scalar(255,0,0), 2, 8, 0);
     
     cv::imshow("EllipseDemo", visualizeimage);

     // Test each pixel in image to see if it's "within bounds"
     // Column : x, Row: y     
     cv::Mat test = cv::Mat::zeros(visualizeimage.rows, visualizeimage.cols, CV_8UC3);
     for (int r = 0; r < test.rows; r++) {
          for (int c = 0; c < test.cols; c++) {               
               Eigen::MatrixXf Zm(2,1);
               Zm << c,r;
               
               cv::Vec3b color;
               
               if (kf_.is_within_region(Zm,0.9545)) {                    
                    // GREEN IS GOOD!
                    color = cv::Vec3b(0,255,0);
               } else {
                    // RED IS BAD!
                    color = cv::Vec3b(0,0,255);                   
               }
               test.at<cv::Vec3b>(r,c) = color;
          }
     }
     cv::ellipse(test, center, axes, ell.angle(), 0, 360, cv::Scalar(255,255,255), 1, 8, 0);
     cv::imshow("Validate", test);
        
     cv::waitKey();
}

cv::RotatedRect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat){
	
     //Get the eigenvalues and eigenvectors
     cv::Mat eigenvalues, eigenvectors;
     cv::eigen(covmat, true, eigenvalues, eigenvectors);

     //Calculate the angle between the largest eigenvector and the x-axis
     double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

     //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
     if(angle < 0)
          angle += 6.28318530718;

     //Conver to degrees instead of radians
     angle = 180*angle/3.14159265359;

     //Calculate the size of the minor and major axes
     double halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(0));
     double halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(1));

     //Return the oriented ellipse
     //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
     return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), angle);

}
