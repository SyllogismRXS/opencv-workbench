#include <iostream>
#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/utils/ColorMaps.h>

using std::cout;
using std::endl;

#define PI 3.14159265359

void draw_line(cv::Mat &img, double rho, double theta, cv::Scalar color, 
               int thickness)
{
     cv::Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     cv::line( img, pt1, pt2, color, thickness, CV_AA);
}

double norm_pi(double x)
{
     while (x < 0)   x += 2*PI;
     while (x > 2*PI) x -= 2*PI;
     return x;
}

double below_90(double x)
{
     while (x > PI/2.0) {
          x -= PI;
     }
     return x;
}

int main(int argc, char *argv[])
{
     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input source>" << endl;
          return -1;
     }

     
     syllo::Stream stream;
     syllo::Status status = stream.open(argv[1]);
     
     if (status != syllo::Success) {
          cout << "Failed to open: " << argv[1] << endl;
          return -1;
     }

     cv::Mat original;
     while (stream.read(original)) {
          cv::Mat dst, cdst, gray;
          //cv::Canny(original, dst, 50, 200, 3);
          //cv::imshow("Canny", dst);
          Jet2Gray_matlab(original,gray);
          cv::threshold(gray, dst, 100, 255, cv::THRESH_TOZERO);
          cv::cvtColor(dst, cdst, CV_GRAY2BGR);          
#if 1
          std::vector<cv::Vec2f> lines;
          cv::HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

          double theta_sum = 0;
          double rho_sum = 0;
          std::vector<cv::Vec2f> samples;
          int count = 0;
          
          //cout<< "-----------" << endl;
          for (size_t i = 0; i < lines.size(); i++ ) {          
               float rho = lines[i][0], theta = norm_pi(lines[i][1]);
               //cout << "rho: " << rho << endl;
               //cout << "theta: " << lines[i][1]*180.0/PI << " -> " << theta*180.0/PI << endl;                              
               if ( (theta > (160.0*PI/180.0) && theta <= (180.0*PI/180.0)) || (theta >= 0 && theta < (20*PI/180.0))) {

                    //double theta_diff;
                    //if (theta > 90*3.14159265359/180.0) {
                    //     theta_diff = 3.14159265359 - theta;
                    //} else {
                    //     theta_diff = theta;
                    //}
                    
                    //theta_sum += theta_diff; // difference from 180 degrees
                    //cout << "theta: " << theta*PI/180.0 << " -> " << below_90(theta)*PI/180 << endl;
                    
                    theta_sum += below_90(theta);
                    rho_sum += abs(rho);
                    samples.push_back(cv::Vec2f(abs(rho), below_90(theta)));
                    count++;

                    draw_line(cdst, rho, theta, cv::Scalar(0,0,255), 1);
               } else {
                    draw_line(cdst, rho, theta, cv::Scalar(255,0,0), 1);
               }                              
          }
          // TODO: Find all lines that are "close" to our "expected" ground
          // plane orientation. "Average" the parameters of these lines and
          // compute a variance parameter around the expected line. Cluster?
          
          if (count != 0) {
               // Draw "averaged" ground plane equation
               double theta_avg = theta_sum / (double)count;
               double rho_avg = rho_sum / (double)count;

               double theta_diff_sum = 0;
               double rho_diff_sum = 0;
               
               std::vector<cv::Vec2f>::iterator it;               
               for(it = samples.begin(); it != samples.end(); it++) {
                    float rho = (*it)[0], theta = (*it)[1];
                    rho_diff_sum += pow(rho-rho_avg,2);
                    theta_diff_sum += pow(theta-theta_avg,2);

                    //cout << "rho: " << rho << endl;
               }

               double rho_std = sqrt( rho_diff_sum / (double)(samples.size()));
               double theta_std = sqrt( theta_diff_sum / (double)(samples.size()));                              

               draw_line(cdst, rho_avg, theta_avg, cv::Scalar(255,0,255), 2);
               draw_line(original, rho_avg, theta_avg, cv::Scalar(0,0,255), 2);               

               double rho_plus = rho_avg + rho_std/2;
               double rho_minus = rho_avg - rho_std/2;

               // Draw error bars
               draw_line(original, rho_plus, theta_avg, cv::Scalar(0,255,0), 2);               
               draw_line(original, rho_minus, theta_avg, cv::Scalar(0,255,0), 2);               
 
               //cout << "R    : Avg=" << rho_avg << "\tstd=" << rho_std << endl;
               //cout << "Theta: Avg=" << theta_avg << "\tstd=" << theta_std << endl;               
          }
          
#else
          std::vector<cv::Vec4i> lines;
          cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
          for( size_t i = 0; i < lines.size(); i++ ) {
               cv::Vec4i l = lines[i];
               cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
          }
#endif
          cv::imshow("source", original);
          cv::imshow("detected lines", cdst);
          
          if (cv::waitKey(50) == 'q') {
               break;
          }
     }

          return 0;
     }
