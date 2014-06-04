#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <opencv_workbench/LARKS/LARKS.h>

#include <opencv_workbench/utils/OpenCV_Helpers.h>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
	
using namespace boost::filesystem;

namespace larks {

     using std::cout;
     using std::endl;
     
     void Larkcomputation::computeCovariance(const cv::Mat& gray, int wsize, 
                                             const int dfactor, cv::Mat& sC11, 
                                             cv::Mat& sC12, cv::Mat& sC22)
     {
          double t = (double) cv::getTickCount();
          int rsize = 2;

          //cv::Mat_<float> gray1(gray.rows + rsize * 2, gray.cols + rsize * 2);
          cv::Mat gray1(gray.rows + rsize * 2, gray.cols + rsize * 2, CV_32F);

          cv::copyMakeBorder(gray, gray1, rsize, rsize, rsize, rsize,
                             cv::BORDER_REPLICATE);          

          //waitKey(0);
          // Gradient image
          float Y[] = { 0.0102, 0.0118, 0, -0.0118, -0.0102, 0.1060, 0.1225, 0,
                        -0.1225, -0.1060, 0.2316, 0.2675, 0, -0.2675, -0.2316, 
                        0.1060, 0.1225, 0, -0.1225, -0.1060, 0.0102, 0.0118, 0, 
                        -0.0118, -0.0102 };

          float X[] = { 0.0102, 0.1060, 0.2316, 0.1060, 0.0102, 0.0118, 0.1225,
                        0.2675, 0.1225, 0.0118, 0, 0, 0, 0, 0, -0.0118, -0.1225,
                        -0.2675, -0.1225, -0.0118, -0.0102, -0.1060, -0.2316, 
                        -0.1060, -0.0102 };

                    
          cv::Mat_<float> gray_f(gray1);
          
          cv::Mat_<float> GXimg(gray_f.rows, gray_f.cols);
          cv::Mat_<float> GYimg(gray_f.rows, gray_f.cols);
          cv::Mat_<float> GX(5, 5);
          cv::Mat_<float> GY(5, 5);

          for (int i = 0; i < 5; i++) {
               for (int j = 0; j < 5; j++) {
                    GX(i, j) = X[i * 5 + j];
                    GY(i, j) = Y[i * 5 + j];
               }
          } 
          
          cv::filter2D(gray_f, GXimg, GXimg.depth(), GX);
          cv::filter2D(gray_f, GYimg, GYimg.depth(), GY);          
          
          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          //	cout << "filter2D: " << t << endl;

          t = (double) cv::getTickCount();

          GXimg = GXimg / 255;

          GYimg = GYimg / 255;

          cv::Mat_<float> GXimg1, GYimg1;          

          cv::resize(cv::Mat(GXimg, cv::Rect(rsize, rsize, gray.cols, gray.rows)), 
                 GXimg1, cv::Size(), 1.0 / dfactor, 1.0 / dfactor, 
                 cv::INTER_LANCZOS4);          

          cv::resize(cv::Mat(GYimg, cv::Rect(rsize, rsize, gray.cols, gray.rows)), 
                 GYimg1, cv::Size(), 1.0 / dfactor, 1.0 / dfactor, 
                 cv::INTER_LANCZOS4);

          // Covariance matrix computation
          cv::Mat_<float> M(2, 2);
          cv::Mat tmp = cv::Mat::zeros(2, 2, CV_32FC1);
          cv::Mat tmp1 = cv::Mat::zeros(2, 2, CV_32FC1);
          cv::Mat v1, v2;

          double alpha = 0.33;

          double len;
          if (wsize == 3)
               len = 20;
          else if (wsize == 5)
               len = 50;
          else
               len = 75.5398;

          double s1, para1, lambda1, lambda2, theta;

          cv::Mat_<double> GX_2(GXimg1.rows, GXimg1.cols);
          cv::Mat_<double> GY_2(GYimg1.rows, GYimg1.cols);
          cv::Mat_<double> GXY(GXimg1.rows, GXimg1.cols);
          cv::Mat_<double> GX_2integral(GXimg1.rows, GXimg1.cols);
          cv::Mat_<double> GY_2integral(GYimg1.rows, GYimg1.cols);
          cv::Mat_<double> GXY_integral(GXimg1.rows, GXimg1.cols);          
                    
          cv::pow(cv::Mat_<double>(GXimg), 2.0, GX_2);          
          cv::pow(cv::Mat_<double>(GYimg), 2.0, GY_2);
          cv::multiply(cv::Mat_<double>(GXimg), cv::Mat_<double>(GYimg), GXY);
         
          cv::integral(GX_2, GX_2integral);
          cv::integral(GY_2, GY_2integral);
          cv::integral(GXY, GXY_integral);          
          
          int row_count1 = 0;
          int col_count1 = 0;
          for (int i = 0; i < gray.rows; i = i + dfactor)
          {
               col_count1 = 0;
               for (int j = 0; j < gray.cols; j = j + dfactor)
               {
                    col_count1++;
               }
               row_count1++;
          }          

          sC11.create(row_count1, col_count1, CV_32FC1);
          sC12.create(row_count1, col_count1, CV_32FC1);
          sC22.create(row_count1, col_count1, CV_32FC1);          

          int row_count = 0;
          for (int i = 0; i < gray.rows; i = i + dfactor)
          {
               int col_count = 0;
               for (int j = 0; j < gray.cols; j = j + dfactor)
               {

                    M(0, 0) = GX_2integral(i + wsize - 1, j + wsize - 1)
                         + GX_2integral(i, j) - GX_2integral(i, j + wsize - 1)
                         - GX_2integral(i + wsize - 1, j);                    
                    M(0, 1) = GXY_integral(i + wsize - 1, j + wsize - 1)
                         + GXY_integral(i, j) - GXY_integral(i, j + wsize - 1)
                         - GXY_integral(i + wsize - 1, j);
                    M(1, 1) = GY_2integral(i + wsize - 1, j + wsize - 1)
                         + GY_2integral(i, j) - GY_2integral(i, j + wsize - 1)
                         - GY_2integral(i + wsize - 1, j);

                    double kk = sqrt(fabs(pow(M(0, 0) + M(1, 1), 2.0) - 4.0 * 
                                          (M(0,0) * M(1, 1) - M(0, 1) * 
                                           M(0, 1))));
                    
                    lambda1 = fabs((M(0, 0) + M(1, 1) + kk) / 2.0);
                    lambda2 = fabs((M(0, 0) + M(1, 1) - kk) / 2.0);
                    theta = atan(-(M(0, 0) + M(0, 1) - lambda1) / (M(1, 1) + 
                                                                   M(0, 1) - 
                                                                   lambda1));

                    if (cvIsNaN(theta))
                         theta = 0.0;
                    v1 = (cv::Mat_<float> (2, 1) << cos(theta), sin(theta));
                    v2 = (cv::Mat_<float> (2, 1) << -sin(theta), cos(theta));
                    s1 = (sqrt(lambda1) + 1.0) / (sqrt(lambda2) + 1.0);
                    tmp = s1 * v1 * v1.t();
                    tmp1 = (1.0 / s1) * v2 * v2.t();
                    tmp = tmp + tmp1;
                    para1 = (sqrt(lambda1) * sqrt(lambda2) + 0.0000001) / len;
                    para1 = pow(para1, alpha);
                    tmp = tmp * para1;
                    sC11.at<float> (row_count, col_count) = tmp.at<float> (0, 0);
                    sC12.at<float> (row_count, col_count) = tmp.at<float> (0, 1);
                    sC22.at<float> (row_count, col_count) = tmp.at<float> (1, 1);
                    if (cvIsNaN(sC11.at<float> (i / dfactor, j / dfactor))) {
                         std::cout << "i " << i << " j " << j << std::endl;
                         std::cout << "lambda1 " << lambda1 << " lambda2 " << lambda2
                                   << std::endl;
                         std::cout << "sqrt(lambda1) " << sqrt(lambda1)
                                   << " sqrt(lambda2) " << sqrt(lambda2) << std::endl;
                         std::cout << "theta " << theta << std::endl;
                         std::cout << "s1 " << s1 << " para1" << para1 << std::endl;
                         cv::waitKey(0);

                    }
                    col_count++;
               }
               row_count++;
          }          

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          //	cout << "Covariance: " << t << endl;
//	cout << "Covariance: " << t << endl;


          //} catch( cv::Exception& e ) {
          //     const char* err_msg = e.what();
          //     std::cout << "+==========================+" << endl;
          //     std::cout << "| Line Number: " << e.line << endl;
          //     std::cout << "+==========================+" << endl;
          //     std::cout << err_msg << endl;
          //     std::cout << "+==========================+" << std::endl;
          //}

     }

     void Larkcomputation::computeLARK(const int rows, const int cols, const 
                                       int wsize, cv::Mat& sC11, cv::Mat& sC12,
                                       cv::Mat& sC22, array_type1& temp)
     {          
          double t = (double) cv::getTickCount();
          int rsize = (wsize - 1) / 2;
          cv::Mat_<float> C11(rows, cols);
          cv::Mat_<float> C12(rows, cols);
          cv::Mat_<float> C22(rows, cols);
          
          // Extend border
          cv::resize(sC11, C11, C11.size(), 0, 0, cv::INTER_LANCZOS4);
          cv::resize(sC12, C12, C12.size(), 0, 0, cv::INTER_LANCZOS4);
          cv::resize(sC22, C22, C22.size(), 0, 0, cv::INTER_LANCZOS4);

          cv::Mat C11_1(rows + rsize * 2, cols + rsize * 2, CV_64F);
          cv::Mat C12_1(rows + rsize * 2, cols + rsize * 2, CV_64F);
          cv::Mat C22_1(rows + rsize * 2, cols + rsize * 2, CV_64F);
          
          cv::copyMakeBorder(C11, C11_1, rsize, rsize, rsize, rsize, cv::BORDER_REPLICATE);          
          cv::copyMakeBorder(C12, C12_1, rsize, rsize, rsize, rsize, cv::BORDER_REPLICATE);
          cv::copyMakeBorder(C22, C22_1, rsize, rsize, rsize, rsize, cv::BORDER_REPLICATE);

          // Spatial distance computation          
          cv::Mat XX = cv::Mat::zeros(1, wsize, CV_32FC1);
          for (int i = -rsize; i <= rsize; i++) {
               XX.at<float> (i + rsize) = i;
          }

          cv::Mat_<double> X2 = cv::repeat(XX, wsize, 1);
          cv::Mat_<double> X1 = X2.t();
          
          cv::Mat_<float> X12 = 2 * X1.mul(X2);
          cv::Mat_<float> X11 = X1.mul(X1);
          cv::Mat_<float> X22 = X2.mul(X2);

          cv::Mat_<float> X1X1(rows, cols);
          cv::Mat_<float> X2X2(rows, cols);
          cv::Mat_<float> X1X2(rows, cols);
                    
          cv::Mat norm_img = cv::Mat::zeros(rows, cols, CV_32F);          

          cv::Mat_<float> Comp1, Comp2, Comp3;
          cv::Mat temp1;
          
          // Geodesic distance computation
          for (int i = 0; i < wsize; i++) {
               for (int j = 0; j < wsize; j++) {
                    temp[i * wsize + j] = cv::Mat::zeros(rows, cols, CV_32F);
                    if (!(i == rsize && j == rsize)) {                         
                         X1X1 = X11(i, j);
                         X1X2 = X12(i, j);
                         X2X2 = X22(i, j);                         
                         Comp1 = cv::Mat(C11_1, cv::Rect(j, i, cols, rows));
                         Comp2 = cv::Mat(C12_1, cv::Rect(j, i, cols, rows));
                         Comp3 = cv::Mat(C22_1, cv::Rect(j, i, cols, rows));
                         temp[i * wsize + j] = Comp1.mul(X1X1) + Comp2.mul(X1X2)
                              + Comp3.mul(X2X2);
                         //temp[i * wsize + j] /= 1.6;

                         exp(-temp[i * wsize + j]/0.7, temp[i * wsize + j]);
                    }
                    //temp[i* wsize + j].convertTo(temp[i* wsize + j],norm_img.type());
                    cv::add(norm_img, temp[i * wsize + j], norm_img);

               }
          }          

          for (int i = 0; i < wsize; i++) {
               for (int j = 0; j < wsize; j++) {
                    temp[i * wsize + j] /= norm_img;
               }
          }

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
//		cout << "LARK : " << t << endl;

/*	cv::Mat LARK = cv::Mat::zeros(rows, cols, CV_32F);
	for (int i = 0; i < rows - wsize + 1; i = i + wsize)
        for (int j = 0; j < cols - wsize + 1; j = j + wsize)
        for (int m = 0; m < wsize; m++)
        for (int n = 0; n < wsize; n++)
        LARK.at<float> (i + m, j + n) = temp[m * wsize + n].at<
        float> (i, j);

	cv::Mat LARK1(rows * 5, cols * 5, CV_64F);
	resize(LARK, LARK1, LARK1.size(), 0, 0, INTER_LANCZOS4);

	namedWindow("C11",1);
	imshow("C11",C11*10);

	namedWindow("C12",1);
	imshow("C12",C12*10);


	namedWindow("LARK", CV_WINDOW_AUTOSIZE);
	imshow("LARK", LARK1*10);

	waitKey(0);
*/          
     }     

     void Saliency::ProtoObject(const cv::Mat &SaliencyMap, cv::Mat& thMap)
     {
          double minVal;
          double maxVal;
          cv::Point minLoc;
          cv::Point maxLoc;

          cv::minMaxLoc(SaliencyMap, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

          //cout << "min " << minVal << " max " << maxVal << endl;

          thMap = cv::Mat::zeros(SaliencyMap.rows, SaliencyMap.cols, CV_32F);

          for (int i = 0; i < SaliencyMap.rows; i++)
               for (int j = 0; j < SaliencyMap.cols; j++)
               {
                    if (SaliencyMap.at<double> (i, j) > 0.27)
                    {
                         //cout << SaliencyMap.at<double>(i,j) << endl;
                         thMap.at<float> (i, j) = 1;
                    }
               }

     }


     void Saliency::computeSaliency(const array_type1& LARK, const int wsize, 
                                    const int psize, cv::Mat& SaliencyMap)
     {

          double t = (double) cv::getTickCount();
          int rsize = (psize - 1) / 2;
          cv::Mat_<float> *temp1;
          cv::Mat_<float> *temp2;
          temp1 = new cv::Mat_<float> [psize * psize];
          temp2 = new cv::Mat_<float> [psize * psize];

          cv::Mat_<double> patch(psize, psize);

          for (int i = 0; i < wsize * wsize; i++)
          {
               cv::copyMakeBorder(LARK[i], temp1[i], rsize, rsize, rsize, rsize,
                                  IPL_BORDER_REPLICATE);
               cv::copyMakeBorder(temp1[i], temp2[i], rsize, rsize, rsize, rsize,
                                  IPL_BORDER_REPLICATE);
          }

          double Norm_center, Norm_surround;

          double sigma = 0.5;
          
          int skip = 3;
          for (int m = 0; m < LARK[0].rows; m = m + skip)
               for (int n = 0; n < LARK[0].cols; n = n + skip) {
                    cv::Mat_<double> center(wsize * wsize, (rsize + 1) * (rsize + 1));
                    for (int i = 0; i < wsize * wsize; i++)
                    {
                         patch = cv::Mat(temp1[i], cv::Rect(n, m, psize, psize));

                         for (int o = 0; o < psize; o = o + 2)
                              for (int p = 0; p < psize; p = p + 2)
                              {
                                   center(i, (o / 2) * (rsize + 1) + (p / 2)) = patch(
                                        o, p);
                              }
                    }
                    Norm_center = cv::norm(center, cv::NORM_L2);

                    cv::Mat_<double> surround(wsize * wsize, (rsize + 1) * (rsize + 1));
                    double s = 0.0;

                    for (int cnt1 = 0; cnt1 < rsize; cnt1++)
                         for (int cnt2 = 0; cnt2 < rsize; cnt2++)
                         {
                              double rho = 0;
                              for (int i = 0; i < wsize * wsize; i++)
                              {
                                   patch = cv::Mat(temp2[i], cv::Rect(n + cnt2, m + cnt1,
                                                                      psize, psize));

                                   for (int o = 0; o < psize; o = o + 2)
                                        for (int p = 0; p < psize; p = p + 2)
                                        {
                                             surround(i, (o / 2) * (rsize + 1) + p / 2)
                                                  = patch(o, p);
                                        }
                              }
                              Norm_surround = cv::norm(surround, cv::NORM_L2);

                              double kkk = 0;
                              for (int i = 0; i < center.rows; i++)
                                   for (int j = 0; j < center.cols; j++)
                                   {
                                        kkk = kkk + center(i, j) * surround(i, j);
                                   }

                              rho = kkk / (Norm_center * Norm_surround);
                              s = s + exp((-1.0 + rho) / pow(sigma, 2.0));
                         }

                    SaliencyMap.at<double> (m, n) = 1.0 / s;

                    for (int i = 0; i < skip; i++)
                         for (int j = 0; j < skip; j++)
                         {
                              if ((m + i < SaliencyMap.rows) && (n + j< SaliencyMap.cols))
                              {
                                   SaliencyMap.at<double> (m + i, n + j)
                                        = SaliencyMap.at<double> (m, n);
                              }
                         }

               }


          delete[] temp1;
          delete[] temp2;

          cv::Mat Saliency1(SaliencyMap.rows * 10, SaliencyMap.cols * 10, CV_64F);
          cv::resize(SaliencyMap, Saliency1, Saliency1.size(), 0, 0, CV_INTER_NN);

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          //cout << "Saliency: " << t << endl;

          //namedWindow("Saliency", CV_WINDOW_AUTOSIZE);
          //imshow("Saliency", Saliency1);
     }

     void LARKFeatureTemplates::computeFeatures(const cv::Mat &whole_gray, 
                                                const cv::Rect & roi, 
                                                const cv::Mat & mask, 
                                                int index, cv::PCA& pca1, 
                                                int cols1)
     {
          cv::Mat objImg(whole_gray, roi);

          objImg.copyTo(img_);
          mask.copyTo(mas_);

          int wsize = 5;
          float downfactor = 0.9;
          int dfactor = 4;
          int maxComponents = 3;

          cv::Mat img, img1, M1;

          cv::resize(objImg, img, cv::Size(), downfactor, downfactor, 
                     cv::INTER_LANCZOS4);
          cv::resize(mask, M_, cv::Size(), downfactor, downfactor, 
                     cv::INTER_LANCZOS4);          

          if (index != 0) {
               cout << "=====> Index not equal to 0" << endl;
               double factor = static_cast<double>( cols1)/ static_cast<double>(img.cols);
               img1.create(cv::Size(cols1, round(img.rows*factor) ), CV_8U);
               M1.create(cv::Size(cols1, round(img.rows*factor) ), CV_8U);
               resize(img, img1, img1.size(), 0,0, cv::INTER_LANCZOS4);
               resize(M_, M1, M_.size(), 0,0, cv::INTER_LANCZOS4);
               M_.release();
               M1.copyTo(M_);
               img.release();
               img1.copyTo(img);
          }

          Larkcomputation LARK;

          cv::Mat sC11, sC12, sC22;          
          
          LARK.computeCovariance(img, wsize, dfactor, sC11, sC12, sC22);
          array_type1 query_temp(boost::extents[wsize * wsize]);

          int rows = img.rows;
          cols_ = img.cols;
                    
          LARK.computeLARK(img.rows, img.cols, wsize, sC11, sC12, sC22, 
                           query_temp);          

          cv::Mat featureset(rows*cols_, wsize*wsize, CV_32F);
          for (int cnt = 0; cnt < wsize * wsize; cnt++) {
               for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols_; j++) {
                         featureset.at<float>(i*cols_+j,cnt) = 
                              query_temp[cnt].at<float>(i,j);
                    }
               }
          }
          
          cv::Mat queryfeature(rows*cols_, maxComponents,CV_32F);          

          if (index == 0) {
               cv::Mat featureset_mask;
               featureset_mask.create(rows*cols_, wsize * wsize, CV_32F);               

               int valid_points = 0;
               for (int cnt = 0; cnt < wsize * wsize; cnt++) {
                    valid_points = 0;

                    for (int i = 0; i < rows; i++) {
                         for (int j = 0; j < cols_; j++) {
                              if (M_.at<unsigned char> (i, j) > 0) {
                                   featureset_mask.at<float> (valid_points, cnt)
                                        = query_temp[cnt].at<float> (i, j);
                                   valid_points++;
                              }
                         }
                    }
               }
               cout << "Valid Points: " << valid_points << endl;

               pca_(featureset_mask(cv::Range(0, valid_points - 1), 
                                   cv::Range(0, wsize * wsize)), cv::Mat(), 
                   CV_PCA_DATA_AS_ROW, maxComponents);

               cv::Mat eigenvectors = pca_.eigenvectors;
               cv::Mat eigenvalues = pca_.eigenvalues;

               for (int i = 0; i < eigenvalues.rows; i++) {
                    std::cout << "Eigen Value #" << i << ": " << 
                         eigenvalues.at<float> (i, 0) << std::endl;
               }

               for (int i = 0; i < maxComponents; i++) {
                    cv::Mat eigenImage(wsize, wsize, CV_32F);
                    cv::Mat eigenImage1(wsize * 10, wsize * 10, CV_32F);
                    for (int m = 0; m < wsize; m++) {
                         for (int n = 0; n < wsize; n++) {
                              eigenImage.at<float> (m, n)
                                   = eigenvectors.at<float> (i, m * wsize + n);
                         }
                    }

                    // Resize and display each eigen image
                    resize(eigenImage, eigenImage1, cv::Size(), 10, 10,
                           cv::INTER_LANCZOS4);
                    std::stringstream kkk;
                    kkk << "eigen ";
                    kkk << i;
                    cv::namedWindow(kkk.str(), 1);
                    cv::imshow(kkk.str(), eigenImage1 * 10);

                    //cv::imshow("queryfeature",QF_[i]*10);

               }

               for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols_; j++) {
                         cv::Mat vec = featureset.row(i* cols_ + j);
                         cv::Mat coeffs = queryfeature.row(i*cols_ + j);
                         pca_.project(vec, coeffs);
                    }
               }
          } else {
               for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols_; j++) {
                         cv::Mat vec = featureset.row(i* cols_ + j);
                         cv::Mat coeffs = queryfeature.row(i*cols_ + j);
                         pca1.project(vec, coeffs);
                    }
               }
          }

          QF_.resize(boost::extents[maxComponents]);

          for (int cnt = 0; cnt < maxComponents; cnt++) {
               QF_[cnt].create(rows, cols_, CV_32F);
               for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols_; j++) {
                         QF_[cnt].at<float> (i, j) = 
                              queryfeature.at<float> (i* cols_ + j, cnt);
                    }
               }
          }
     }

     float LARKFeatureTemplates::MatrixCosineMeasure(array_type1& QF1, cv::Mat M1, array_type1& QF, cv::Mat& M)
     {
          int maxComponents = QF1.shape()[0];

          int Newrow = QF[0].rows < QF1[0].rows ? QF[0].rows : QF1[0].rows;

          bool indicator = QF[0].rows < QF1[0].rows ? false : true;
          float QF_norm = 0.0;
          float QF_norm1 = 0.0;
          float rho = 0.0;



          for (int i = 0; i < maxComponents; i++)
               for( int m = 0; m < QF[0].cols; m = m + 2)
                    for (int n = 0; n < Newrow; n = n + 2)
                    {
                         if (indicator)
                         {
                              if (M1.at<unsigned char> (n, m) > 10)
                              {
                                   rho += QF[i].at<float> (n,m)* QF1[i].at<float> (n,m);
                                   QF_norm	+= pow(QF[i].at<float> (n,m),2);
                                   QF_norm1	+= pow(QF1[i].at<float> (n,m),2);
                              }
                         }
                         else
                         {
                              if (M.at<unsigned char> (n, m)	> 10)
                              {
                                   rho += QF[i].at<float> (n,m)* QF1[i].at<float> (n,m);
                                   QF_norm	+= pow(QF[i].at<float> (n,m),2);
                                   QF_norm1	+= pow(QF1[i].at<float> (n,m),2);
                              }

                         }

                    }
          QF_norm = sqrt(QF_norm);
          QF_norm1 = sqrt(QF_norm1);

          rho = fabs(rho) / (QF_norm1 * QF_norm);

          return rho;

     }

     void LARKS::startTraining()
     {
          if (models.size() > 0) {
               models.erase(models.begin(),models.begin()+models.size());
          }
          index = 0;
     }


     void LARKS::trainInstance(const std::string& name, cv::Mat &img)
     {
          //ROS_INFO("LARKs: training instance: %s", name.c_str());
 
          cv::Mat img_gray;
          // compute the gradient summary image
          //cv::Mat img = data.image;
 
          if(img.channels() != 1) {
               cv::cvtColor(img, img_gray, CV_BGR2GRAY);
          } else {
               img_gray = img;
          }
 
          //    Mat drawImg(data.image,data.roi);
          //    bitwise_not(drawImg,drawImg,data.mask);
 
          cv::Rect roi(0,0,img.cols, img.rows);
          cv::Mat mask = cv::Mat::ones(img.rows, img.cols, CV_32F);
          //convert(data.roi,roi);
          //convert(data.mask,mask);
 
          LARKFeatureTemplates crt_tpl;
 
          if (index == 0) {
               crt_tpl.computeFeatures(img_gray, roi, mask, index, crt_tpl.pca_, 
                                       crt_tpl.cols_);
               models.push_back(crt_tpl);
               index++;
               printf("has ");
          } else {
               std::cout << "models.size()" << models.size() << std::endl;
               crt_tpl.computeFeatures(img_gray,roi,mask, index, models[0].pca_, 
                                       models[0].cols_);
               float max_score = 0.0;
               for (unsigned int i = 0; i < models.size(); i++) {
                    float score = crt_tpl.MatrixCosineMeasure(models[i].QF_,models[i].M_, crt_tpl.QF_, crt_tpl.M_);
                    cout << "score= " << score << endl;
                    if (score > max_score) {
                         max_score = score;
                    }
               }
  
               /*BOOST_FOREACH(LARKFeatureTemplates& tpl, models) {
                 float score = tpl.MatrixCosineMeasure(crt_tpl.QF, crt_tpl.M);
 
                 if (score > 0.4) {
                 found = true;
                 }
                 }*/
 
               if (max_score < 0.4) {
                    printf("has ");
                    models.push_back(crt_tpl);
                    index++;
               } else {
                    printf("still has ");
               }
          }
          printf("%d templates\n", index);
          cv::imshow("trainingImage", img_gray(roi));      
     }

     void LARKS::detect(cv::Mat &image)
     {
          //detections_.detections.clear();

          cv::Mat img4 = image;

          cv::Mat img2;

          cv::cvtColor(img4,img2,CV_BGR2RGB);
          Detection d;

          cv::Mat img1;
          cv::cvtColor(img2, img1, CV_RGB2GRAY);

          cv::Mat img3;
          img2.copyTo(img3);

          cv::Mat target;
          cv::resize(img1, target, cv::Size(), factor, factor, cv::INTER_LANCZOS4);          

          // Covariance Computation
          cv::Mat sC11, sC12, sC22;
          int wsize_s = 5;
          float f = 0.25;
          LARK.computeCovariance(target, wsize_s, dfactor, sC11, sC12, sC22);
          int block_width = img1.cols / x_block;
          int block_height = img1.rows / y_block;
          
          if (use_saliency) // Take advantage of Saliency detection
          {

               double t = (double) cv::getTickCount();
               // LARK computation for Saliency
               int rows = target.rows / dfactor;
               int cols = target.cols / dfactor;

               array_type1
                    Saliency_LARK(boost::extents[wsize_s * wsize_s]);
               LARK.computeLARK(rows, cols, wsize_s, sC11, sC12, sC22,
				Saliency_LARK);

               cv::Mat SaliencyMap;
               SaliencyMap.create(rows, cols, CV_64F);
               int psize = 5;
               SalientRegion.computeSaliency(Saliency_LARK, wsize_s, psize, SaliencyMap);

               cv::Mat thMap;
               SalientRegion.ProtoObject(SaliencyMap, thMap);

               cv::Mat Saliency_mask;
               cv::Mat selectedObj = cv::Mat::zeros(img1.size(), img1.type());

               cv::resize(thMap, Saliency_mask, img1.size(), 0, 0, CV_INTER_NN);

               cv::Mat Mask = (Saliency_mask > 0) / 255;
               img1.copyTo(selectedObj, Mask);

               start.resize(x_block * y_block);
               end.resize(x_block * y_block);

               std::vector<float> sum(x_block * y_block);
               int lineWidth = 3;

               block_table = cv::Mat::zeros(x_block, y_block, CV_8U);
               AtleastSalient = 0;
               for (int i = 0; i < x_block; i++)
                    for (int j = 0; j < y_block; j++) {
                         start[i * y_block + j].x = i * block_width;
                         start[i * y_block + j].y = j * block_height;
                         end[i * y_block + j].x = (i + 1) * block_width - 1;
                         end[i * y_block + j].y = (j + 1) * block_height - 1;

                         cv::Mat region = cv::Mat(Mask, cv::Rect(
                                               start[i * y_block + j].x, start[i * y_block
                                                                               + j].y, block_width, block_height));

                         sum[i * y_block + j] = 0;
                         for (int m = 0; m < region.rows; m++)
                              for (int n = 0; n < region.cols; n++) {
                                   sum[i * y_block + j] += region.at<
                                        unsigned char> (m, n);
                              }

                         sum[i * y_block + j]
                              /= (block_height * block_width);

                         if (sum[i * y_block + j] > 0.6) {
                              AtleastSalient++;
                              block_table.at<unsigned char> (i, j) = 1;

                              line(img3, cv::Point(start[i * y_block + j].x,
                                               start[i * y_block + j].y), cv::Point(
                                                    start[i * y_block + j].x, end[i
                                                                                  * y_block + j].y), white, lineWidth, CV_AA);
                              line(img3, cv::Point(start[i * y_block + j].x,
                                               end[i * y_block + j].y), cv::Point(end[i
                                                                                  * y_block + j].x,
                                                                              end[i * y_block + j].y),  white, lineWidth, CV_AA);
                              line(img3, cv::Point(end[i * y_block + j].x, end[i
                                                                           * y_block + j].y), cv::Point(end[i
                                                                                                        * y_block + j].x,
                                                                                                    start[i * y_block + j].y),  white, lineWidth, CV_AA);
                              line(img3, cv::Point(end[i * y_block + j].x,
                                               start[i * y_block + j].y), cv::Point(
                                                    start[i * y_block + j].x, start[i
                                                                                    * y_block + j].y),  white, lineWidth, CV_AA);

                              start[i * y_block + j].x = start[i * y_block
                                                               + j].x * factor;
                              start[i * y_block + j].y = start[i * y_block
                                                               + j].y * factor;
                              end[i * y_block + j].x = end[i * y_block + j].x
                                   * factor;
                              end[i * y_block + j].y = end[i * y_block + j].y
                                   * factor;
                         }

                    }

               t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
               std::cout << " Saliency Computation: " << t << std::endl;
               use_saliency = false;

          }          

          // LARK computation for target
          if (AtleastSalient)
          {

               double t = (double) cv::getTickCount();               
               int level = 3;

               std::vector<array_type2> TFs(num_models_);
               std::vector<array_type3> Py_TFs(num_models_);                              
               
               for (int i = 0; i < num_models_; i++)                    
               {
                    TFs[i].resize(boost::extents[numScale][maxComponents]);
                    
                    getTF(wsize, target, sC11, sC12, sC22, binaryfeaturemode, maxComponents, numScale, TFs[i], means[i], eigenvectors[i]);
                    return;
                    Py_TFs[i].resize(boost::extents[level][numScale][maxComponents]);
                    Py_QFs[i].resize(boost::extents[level][numTemplates[i]][numRotation][maxComponents]);
                    Py_query_masks[i].resize(boost::extents[level][numTemplates[i]][numRotation]);
                    Py_QF_norms[i].resize(boost::extents[level][numTemplates[i]][numRotation][numScale]);
                    pyramidFeatures(TFs[i], Py_TFs[i], QFs[i], Py_QFs[i], QF_norms[i], Py_QF_norms[i], query_masks[i], Py_query_masks[i], level, numTemplates[i]);

                    index_templates[i] = 0;
                    index_scales[i] = 0;
                    index_rotations[i] = 0;
               }
               
               t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
               std::cout << "PCA projection computation: time " << t << std::endl;

               ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
               // Find the location of objects

               std::vector<cv::Mat> RMs(num_models_);                             

               for (int i = 0; i < num_models_; i++)
               {
                    if (Prev_max[i] < threshold_[i])
                         Pre_Search(f, numTemplates[i], numRotation, numScale, Py_query_masks[i], Py_QFs[i], Py_QF_norms[i], target, binaryfeaturemode, Py_TFs[i], scales, block_table, region_index[i], Prev_max[i], maxComponents, maxVals[i], labels[i], 0, i, offset_x, offset_y);

                    else
                    {
                         for (int tem = 0; tem < numTemplates[i]; tem++)
                              for (int sca = 0; sca < numScale; sca++)
                                   for (int rot = 0; rot < numRotation; rot++)
                                   {
					region_index[i][tem][sca][rot] = region_index[i][index_templates[i]][index_scales[i]][index_rotations[i]];
                                   }

                    }


                    Search(factor, numTemplates[i], numRotation, numScale, Py_query_masks[i], Py_QFs[i], Py_QF_norms[i], target, binaryfeaturemode, Py_TFs[i], scales, block_table, region_index[i], Prev_max[i], maxComponents, img2, use_saliency, img1,  RMs[i], models_[i],  threshold_[i], maxVals[i], index_templates[i], index_scales[i], index_rotations[i], level, i, d);

               }

          }
          else
          {
               use_saliency = true;
               cv::waitKey(3);

          }
     }

     void LARKS::Pre_Search(const float factor, const int numTemplate, const int numRotation, const int numScale, array_type3& query_mask, array_type4& QF, double_type4& QF_norm, cv::Mat target, bool binaryfeaturemode, array_type3& TF, cv::Mat scales, cv::Mat block_table, Point_type3& region_index, double Prev_max, int maxComponents, double_type3& maxVal, cv::Mat& labels, int level, int obj, std::vector<int_type1> &offset_x, std::vector<int_type1> &offset_y)
     {

          double	t = (double) cv::getTickCount();
          array_type3 RM(boost::extents[numTemplate][numRotation][numScale]);

          int template0 = 0;
          int template1;
          for (int i = 1; i < numTemplate; i++)
          {
               if (labels.at<int>(template0,0) != labels.at<int>(i,0) )
               {
                    template1 = i;
                    break;
               }
               else
                    template1 = template0;
          }

          offset_x[obj][template0] = query_mask[level][template0][0].rows / 2;
          offset_y[obj][template0] = query_mask[level][template0][0].cols / 2;

          Multiscale_search(QF, QF_norm, query_mask,target,  RM, offset_x[obj][template0],
                            offset_y[obj][template0], template0, binaryfeaturemode, TF,  scales, block_table, region_index, maxComponents, factor, maxVal, level, obj);

          offset_x[obj][template1] = query_mask[level][template1][0].rows / 2;
          offset_y[obj][template1] = query_mask[level][template1][0].cols / 2;

          Multiscale_search(QF, QF_norm, query_mask,target,  RM, offset_x[obj][template1],
                            offset_y[obj][template1], template1, binaryfeaturemode, TF,  scales, block_table, region_index, maxComponents, factor, maxVal, level, obj);


          for (int i = 0; i < numTemplate; i++)
          {
               if ( labels.at<int>(i,0) == labels.at<int>(template0,0) )
               {
                    for (int sca = 0; sca < numScale; sca++)
                         for (int rot = 0; rot < numRotation; rot++)
                         {
                              region_index[i][sca][rot] = region_index[template0][index_scales[obj]][index_rotations[obj]];
                         }
               }
               if ( labels.at<int>(i,0) == labels.at<int>(template1,0) )
               {
                    for (int sca = 0; sca < numScale; sca++)
                         for (int rot = 0; rot < numRotation; rot++)
                         {
                              region_index[i][sca][rot] = region_index[template1][index_scales[obj]][index_rotations[obj]];
                         }
               }

          }
          /*

            std::cout << "numTemplate " << numTemplate << std::endl;

            for (int template0 = 0; template0 < numTemplate; template0++)
            {
            offset_x[obj][template0] = query_mask[level][template0][0].rows / 2;
            offset_y[obj][template0] = query_mask[level][template0][0].cols / 2;

            Multiscale_search(QF, QF_norm, query_mask,target,  RM, offset_x[obj][template0],
            offset_y[obj][template0], template0, binaryfeaturemode, TF,  scales,	block_table, region_index, maxComponents, factor, maxVal, level, obj);

            }
          */

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          cout << "[Pre detection search time " << t << " sec]" << endl;

     }

     void LARKS::pyramidFeatures(array_type2 TF, array_type3& TFs, array_type3 QF, array_type4& QFs,double_type3 QF_norm, double_type4& QF_norms, array_type2 query_mask, array_type3& query_masks, int level, const int numTemplate)
     {

//	int skip[] = { 5,5, 5,4};

          int skip[] = { 2,2, 2, 2 }; //5,5,5,4
          float f[] = {0.25, 0.5, 1.0};

          for (int le = 0; le < level; le++)
          {

               for (int scale = 0; scale < numScale; scale++)
                    for (int cnt = 0; cnt < maxComponents; cnt++)
                    {
                         if (le != level-1)
                              resize(TF[scale][cnt], TFs[le][scale][cnt], cv::Size(),f[le], f[le], cv::INTER_LANCZOS4);
                         else
                              TF[scale][cnt].copyTo(TFs[le][scale][cnt]);

                    }
               for (int tem = 0; tem < numTemplate; tem++)
                    for (int rot = 0; rot < numRotation; rot++)
                    {
                         if (le != level-1)
                              resize(query_mask[tem][rot], query_masks[le][tem][rot], cv::Size(), f[le], f[le], cv::INTER_LANCZOS4);
                         else
                              query_mask[tem][rot].copyTo(query_masks[le][tem][rot]);

                         for (int cnt = 0; cnt < maxComponents; cnt++)
                         {
                              if (le != level-1)
                                   resize(QF[tem][rot][cnt], QFs[le][tem][rot][cnt], cv::Size(), f[le], f[le], cv::INTER_LANCZOS4);
                              else
                                   QF[tem][rot][cnt].copyTo(QFs[le][tem][rot][cnt]);

                         }

                         for (int m = 0; m < numScale; m++)
                         {
                              if (le != level-1)
                              {
                                   QF_norms[le][tem][rot][m] = 0;

                                   for (int i = 0; i < query_masks[le][tem][rot].rows; i = i + skip[m])
					for (int j = 0; j < query_masks[le][tem][rot].cols; j = j + skip[m])
                                             for (int cnt = 0; cnt < maxComponents; cnt++)
                                             {

                                                  if (query_masks[le][tem][rot].at<unsigned char> (i, j) > 10)
                                                  {
                                                       QF_norms[le][tem][rot][m] += pow(QFs[le][tem][rot][cnt].at<float> (i, j), 2);
                                                  }
                                             }

                                   QF_norms[le][tem][rot][m] = sqrt(QF_norms[le][tem][rot][m]);
                              }
                              else
                                   QF_norms[le][tem][rot][m] = QF_norm[tem][rot][m];
                         }


                    }
          }

     }     

     void LARKS::Search(const float factor, const int numTemplate, const int numRotation, const int numScale, array_type3& query_mask, array_type4& QF, double_type4& QF_norm, cv::Mat target, bool binaryfeaturemode, array_type3& TF, cv::Mat scales, cv::Mat block_table, Point_type3& region_index, double Prev_max, int maxComponents, cv::Mat& img2, bool& use_saliency, cv::Mat img1,  cv::Mat& RM1, std::string modelname, const float threshold , double_type3& maxVals, int& index_template, int& index_scale, int& index_rotation, int level, int obj, Detection &d)
     {

          double t = (double) cv::getTickCount();
          array_type3 RM(boost::extents[numTemplate][numRotation][numScale]);

          int index_template1 = 0;
          //int index_scale1;
          //int index_rotation1 = 0;

          for (int lev = 1; lev < level; lev++)
               for (int i = 0; i < numTemplate; i++)
               {
                    offset_x[obj][i] = query_mask[lev][i][0].rows / 2;

                    offset_y[obj][i] = query_mask[lev][i][0].cols / 2;


                    Multiscale_search_withPrevious(QF, QF_norm, query_mask, target,  RM,
                                                   offset_x[obj][i], offset_y[obj][i], i, binaryfeaturemode, TF, scales, block_table, region_index, maxComponents, maxVals, threshold, lev, obj);


               }

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          cout << "[Multi-scale search time " << t << " sec]" << endl;


          t = (double) cv::getTickCount();
          cv::Mat rotationIndex = cv::Mat::zeros(RM[0][0][0].rows,
                                         RM[0][0][0].cols, CV_8U);

          cv::Mat scaleIndex = cv::Mat::zeros(RM[0][0][0].rows, RM[0][0][0].cols,
                                      CV_8U);
          cv::Mat templateIndex = cv::Mat::zeros(RM[0][0][0].rows,
                                         RM[0][0][0].cols, CV_8U);
          cv::Mat RM_final = cv::Mat::zeros(RM[0][0][0].rows, RM[0][0][0].cols,
                                    CV_32F);

          float maxval = 0.0;
          for (int rot = 0; rot < numRotation; rot++)
               for (int j = 0; j < numScale; j++)
                    for (int i = 0; i < numTemplate; i++)
                    {
                         double minVal, maxVal;
                         cv::Point minLoc, maxLoc;
                         minMaxLoc(RM[i][rot][j],&minVal,&maxVal,&minLoc, &maxLoc);
                         if (maxVal >= maxval)
                         {
                              maxval = maxVal;
                              index_template = i;
                              index_scale = j;
                              index_rotation = rot;
                              RM_final.at<float>(maxLoc.y, maxLoc.x) = maxval;

                         }
                    }

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          //cout << " Index decision time " << t << endl;

          t = (double) cv::getTickCount();
          minMaxLoc(RM_final, &minVal1, &maxVal1, &minLoc1, &maxLoc1,
                    cv::Mat());

          cv::Mat RM2;


          for (int i = 0; i < numTemplate;  i++) {
               offset_x[obj][i] = query_mask[level-1][i][0].rows / (2);
               offset_y[obj][i] = query_mask[level-1][i][0].cols / (2);
          }

          //cout << "min " << minVal1 << " max " << maxVal1 << endl;

          std::vector<cv::Point2f> detectionpt(4), detectionpt1(4);

          detectionpt[0].x = (maxLoc1.x - offset_y[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor > 0 ? (maxLoc1.x - offset_y[obj][index_template]
                                                                                   / scales.at<float> (0, index_scale)) / factor: 0;
          detectionpt[0].y = (maxLoc1.y - offset_x[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor > 0 ? (maxLoc1.y - offset_x[obj][index_template]
                                                                                   / scales.at<float> (0, index_scale)) / factor: 0 ;
          detectionpt[1].x = (maxLoc1.x - offset_y[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor > 0 ? (maxLoc1.x - offset_y[obj][index_template]
                                                                                   / scales.at<float> (0, index_scale)) / factor: 0;
          detectionpt[1].y = (maxLoc1.y + offset_x[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor < 480 ? (maxLoc1.y + offset_x[obj][index_template]
                                                                                     / scales.at<float> (0, index_scale)) / factor : 479;
          detectionpt[2].x = (maxLoc1.x + offset_y[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor < 640 ? (maxLoc1.x + offset_y[obj][index_template]
                                                                                     / scales.at<float> (0, index_scale)) / factor : 639;
          detectionpt[2].y = (maxLoc1.y - offset_x[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor> 0 ? (maxLoc1.y - offset_x[obj][index_template]
                                                                                  / scales.at<float> (0, index_scale)) / factor: 0 ;
          detectionpt[3].x = (maxLoc1.x + offset_y[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor < 640 ? (maxLoc1.x + offset_y[obj][index_template]
                                                                                     / scales.at<float> (0, index_scale)) / factor : 639;
          detectionpt[3].y = (maxLoc1.y + offset_x[obj][index_template]
                              / scales.at<float> (0, index_scale)) / factor < 480? (maxLoc1.y + offset_x[obj][index_template]
                                                                                    / scales.at<float> (0, index_scale)) / factor : 479;

          cv::Rect ROI;

          maxval = 0.0;

          cv::Mat RM_final1 = cv::Mat::zeros(RM[0][0][0].rows, RM[0][0][0].cols,	CV_32F);

          for (int rot = 0; rot < numRotation; rot++)
               for (int j = 0; j < numScale; j++)
                    for (int i = 0; i < numTemplate; i++)
                    {
                         double minVal, maxVal;
                         cv::Point minLoc, maxLoc;

                         for (int m = static_cast<int> (detectionpt[0].y * factor); m
                                   < static_cast<int> (detectionpt[3].y * factor); m++)
                              for (int n = static_cast<int> (detectionpt[0].x * factor); n
                                        < static_cast<int> (detectionpt[3].x * factor); n++) {
                                   RM[i][rot][j].at<float> (m, n) = 0;
                              }

                         minMaxLoc(RM[i][rot][j],&minVal,&maxVal,&minLoc, &maxLoc);
                         if (maxVal >= maxval)
                         {
                              maxval = maxVal;
                              index_template1 = i;
                              //index_scale1 = j;
                              //index_rotation1 = rot;
                              RM_final1.at<float>(maxLoc.y, maxLoc.x) = maxval;

                         }
                    }



          minMaxLoc(RM_final1, &minVal2, &maxVal2, &minLoc2, &maxLoc2,cv::Mat());

          add(RM_final,RM_final1,RM_final);

          resize(RM_final, RM1, cv::Size(), 1/ (factor), 1 / (factor),
                 CV_INTER_NN);



          detectionpt1[0].x = (maxLoc2.x - offset_y[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;
          detectionpt1[0].y = (maxLoc2.y - offset_x[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;
          detectionpt1[1].x = (maxLoc2.x - offset_y[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;
          detectionpt1[1].y = (maxLoc2.y + offset_x[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;
          detectionpt1[2].x = (maxLoc2.x + offset_y[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;
          detectionpt1[2].y = (maxLoc2.y - offset_x[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;
          detectionpt1[3].x = (maxLoc2.x + offset_y[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;
          detectionpt1[3].y = (maxLoc2.y + offset_x[obj][index_template1]
                               / scales.at<float> (0, index_scale)) / factor;

          Prev_Loc.x = maxLoc1.x;
          Prev_Loc.y = maxLoc1.y;
          Prev_max = maxVal1;

          maxLoc1.x /= factor;
          maxLoc1.y /= factor;
          maxLoc2.x /= factor;
          maxLoc2.y /= factor;



          if (maxVal1 < threshold )
          {
               use_saliency = true;
          }
          else
          {
               name = modelname;
               d.label = modelname;
               score = maxVal1;

               d.score = maxVal1;

               /// TODO, PUT IN DETECTION STRUCT / CLASS
               /// d.mask.roi.x = detectionpt[0].x;
               /// d.mask.roi.y = detectionpt[0].y;
               /// d.mask.roi.width = detectionpt[3].x - detectionpt[0].x + 1 ;
               /// d.mask.roi.height = detectionpt[3].y - detectionpt[0].y + 1;
               cout << "Detect X: " << detectionpt[0].x << endl;
               cout << "Detect Y: " << detectionpt[0].y << endl;
               

               /*roi.x = detectionpt[0].x;
                 roi.y = detectionpt[0].y;
                 roi.width = detectionpt[3].x - detectionpt[0].x + 1 ;
                 roi.height = detectionpt[3].y - detectionpt[0].y + 1;
                 mask.create(d.mask.roi.height, d.mask.roi.width, CV_8UC1);
                 mask = cv::Scalar::all(0);



                 for (int i = d.mask.roi.y; i < d.mask.roi.height; i++)
                 for (int j = d.mask.roi.x; j < d.mask.roi.width; j++){
                 mask.at<unsigned char>(i,j) = 255;
                 }
               */

               // TODO: ADD TO DETECTION VECTOR
               //detections_.detections.push_back(d);
               //d.mask.mask = mask;

          }
     }

     void LARKS::Multiscale_search(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const cv::Mat& target, array_type3& RM, const int offset_x, const int offset_y,
                                   const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const cv::Mat& scales,	const cv::Mat& block_table, Point_type3& region_index,  const int maxComponents, float f, double_type3& maxVals, int level, int obj)
     {

          //int skip[] = { 5,5, 5, 4 }; //5,5,5,4
          //int apart[] = { 4,4, 4, 4 };

          int skip[] = { 2,2, 2, 2 }; //5,5,5,4
          int apart[] = { 3,3, 3, 3 };



          double t = (double) cv::getTickCount();

          for (int rot = 0; rot < numRotation; rot++)
          {

               int width = query_mask[level][query_index][rot].cols;
               int height = query_mask[level][query_index][rot].rows;

               int half_width = query_mask[level][query_index][rot].cols / 2;
               int half_height = query_mask[level][query_index][rot].rows / 2;

               for (int scale = 0; scale < numScale; scale++)
               {

                    cv::Mat RM1 = cv::Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);
                    cv::Mat RM2 = cv::Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);


                    RM[query_index][rot][scale] = cv::Mat::zeros(TF[level][0][0].rows, TF[level][0][0].cols, CV_32F);

                    double maxvalue = 0;

                    for (int i = 0; i < x_block; i++)
                         for (int j = 0; j < y_block; j++)
                         {


                              if (block_table.at<unsigned char> (i, j) == 1)
                              {
                                   //cout << "i,j " << i << " , " << j << " : " << block_table.at<unsigned char> (i, j) << endl;

                                   int begin_y = start[i * y_block + j].y *f* scales.at<float> (0, scale) < half_height ? half_height
                                                                                                            : start[i * y_block + j].y*f* scales.at<float> (0,scale);
                                   int begin_x = start[i * y_block + j].x *f* scales.at<
                                        float> (0, scale) < half_width ? half_width
                                                            : start[i * y_block + j].x *f* scales.at<
                                                                 float> (0, scale);
                                   int end_y = end[i * y_block + j].y *f* scales.at<
                                        float> (0, scale) > TF[level][scale][0].rows
                                                                                                            - half_height + 1 ? TF[level][scale][0].rows
                                                                                                            - half_height + 1 : end[i * y_block + j].y
                                                                                                            *f* scales.at<float> (0, scale);
                                   int end_x = end[i * y_block + j].x *f* scales.at<
                                        float> (0, scale) > TF[level][scale][0].cols
                                        - half_width + 1 ? TF[level][scale][0].cols
                                        - half_width + 1 : end[i * y_block + j].x
                                        *f* scales.at<float> (0, scale);



                                   for (int m = begin_y; m < end_y; m = m	+ apart[scale])
					for (int n = begin_x; n < end_x; n = n	+ apart[scale])
					{

                                             double rho = 0;
                                             double TF_norm = 0;

                                             for (int cnt = 0; cnt < maxComponents; cnt++)
                                             {

                                                  for (int m1 = 0; m1 < height; m1 = m1 + skip[scale])
                                                       for (int n1 = 0; n1 < width; n1	= n1 + skip[scale])
                                                       {

                                                            if (query_mask[level][query_index][rot].at<unsigned char> (m1, n1)	> 10)
                                                            {
                                                                 rho += QF[level][query_index][rot][cnt].at<float> (m1,	n1)* TF[level][scale][cnt].at<	float> (
                                                                      m+ m1- half_height,n+ n1- half_width);
                                                                 TF_norm	+= pow(TF[level][scale][cnt].at<float> (m1+ m- half_height,n1+ n- half_width),2);
                                                            }

                                                       }

                                             }
                                             TF_norm = sqrt(TF_norm);

                                             rho = rho / (QF_norm[level][query_index][rot][scale] * TF_norm);

                                             RM1.at<float>(m,n) = pow(rho, 2) / (1- pow(rho, 2));

                                             if ( RM1.at<float>(m,n) > maxvalue)
                                             {

                                                  maxvalue = RM1.at<float>(m,n);

                                                  region_index[query_index][scale][rot].x = j;
                                                  region_index[query_index][scale][rot].y = i;

                                             }

					}


                              }


                         }

                    resize(RM1, RM[query_index][rot][scale],RM[query_index][rot][scale].size(), 0, 0, CV_INTER_NN);
                    double minVal;
                    cv::Point minLoc, maxLoc;
                    minMaxLoc(RM[query_index][rot][scale], &minVal, &maxVals[query_index][scale][rot], &minLoc, &maxLoc,cv::Mat());


               }

          }

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          //cout << "search" << t << endl;

     }

     void LARKS::Multiscale_search_withPrevious(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const cv::Mat& target,array_type3& RM, const int offset_x, const int offset_y,const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const cv::Mat& scales,	const cv::Mat& block_table,	Point_type3&  region_index, const int maxComponents,double_type3& maxVals, float threshold, int level, int obj)
     {

          int skip[] = { 5, 5, 5, 4 };
          int apart[] = { 4, 4, 4, 4 };

          //int skip[] = { 2,2, 2, 2 }; //5,5,5,4
          //int apart[] = { 3,3, 3, 3 };


          double t = (double) cv::getTickCount();

          for (int rot = 0; rot < numRotation; rot++)
          {

               int width = query_mask[level][query_index][rot].cols;
               int height = query_mask[level][query_index][rot].rows;

               int half_width = query_mask[level][query_index][rot].cols / 2;
               int half_height = query_mask[level][query_index][rot].rows / 2;

               for (int scale = 0; scale < numScale; scale++)
               {
                    float maxvalue = 0.0;
                    cv::Mat RM1 = cv::Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);
                    cv::Mat RM2 = cv::Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);

                    RM[query_index][rot][scale] = cv::Mat::zeros(TF[level][0][0].rows, TF[level][0][0].cols,	CV_32F);

                    if ( maxVals[query_index][scale][rot] > 0.1)
                    {

                         int i = region_index[query_index][scale][rot].y;
                         int j = region_index[query_index][scale][rot].x;

                         if (block_table.at<unsigned char> (i, j) == 1)
                         {

                              int begin_y = start[i * y_block + j].y
                                   * scales.at<float> (0, scale) < half_height ? half_height
                                                                   : start[i * y_block + j].y * scales.at<float> (0, scale);
                              int begin_x = start[i * y_block + j].x
                                                                   * scales.at<float> (0, scale) < half_width ? half_width
                                                                                                   : start[i * y_block + j].x * scales.at<float> (0, scale);
                              int end_y = end[i * y_block + j].y * scales.at<float> (0, scale)
                                                                                                   > TF[level][scale][0].rows - half_height + 1 ? TF[level][scale][0].rows
                                                                   - half_height + 1 : end[i * y_block + j].y * scales.at<
                                   float> (0, scale);
                              int end_x = end[i * y_block + j].x * scales.at<float> (0, scale)
                                                                   > TF[level][scale][0].cols - half_width + 1 ? TF[level][scale][0].cols
                                   - half_width + 1 : end[i * y_block + j].x
                                   * scales.at<float> (0, scale);

                              for (int m = begin_y; m < end_y; m = m + apart[scale])
                                   for (int n = begin_x; n < end_x; n = n + apart[scale])
                                   {

                                        double rho = 0;
                                        double TF_norm = 0;

                                        for (int cnt = 0; cnt < maxComponents; cnt++)
                                             for (int m1 = 0; m1 < height; m1 = m1 + skip[scale])
                                                  for (int n1 = 0; n1 < width; n1 = n1 + skip[scale])
                                                  {

                                                       if (query_mask[level][query_index][rot].at<unsigned char> (m1, n1) > 10)
                                                       {
                                                            rho += QF[level][query_index][rot][cnt].at<float> (m1,
                                                                                                               n1) * TF[level][scale][cnt].at<float> (m
                                                                                                                                                      + m1 - half_height, n + n1
                                                                                                                                                      - half_width);
                                                            TF_norm += pow(TF[level][scale][cnt].at<float> (m1
                                                                                                            + m - half_height, n1 + n
                                                                                                            - half_width), 2);
                                                       }


                                                  }
                                        TF_norm = sqrt(TF_norm);

                                        rho = rho / (QF_norm[level][query_index][rot][scale] * TF_norm);

                                        RM1.at<float>(m,n) = pow(rho, 2) / (1- pow(rho, 2));


                                        if ( RM1.at<float>(m,n) > maxvalue)
                                        {

                                             maxvalue = RM1.at<float>(m,n);

                                             region_index[query_index][scale][rot].x = j;
                                             region_index[query_index][scale][rot].y = i;

                                        }
                                   }
                         }
                    }

                    resize(RM1, RM[query_index][rot][scale], RM[query_index][rot][scale].size(),0, 0, CV_INTER_NN);

               }
          }

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
//	cout << "search" << t << endl;
     }



     void LARKS::getTF(const int wsize, cv::Mat target, cv::Mat sC11, 
                       cv::Mat sC12, cv::Mat sC22, bool binaryfeaturemode, 
                       const int maxComponents, const int numScale, 
                       array_type2& TF, cv::Mat& means, cv::Mat& eigenvectors)
     {
          array_type1 target_LARK(boost::extents[wsize * wsize]);
          LARK.computeLARK(target.rows, target.cols, wsize, sC11, sC12, sC22,
                           target_LARK);
                    
          cv::Mat target_featureset;
          target_featureset.create(target.cols * target.rows, wsize
                                   * wsize, CV_32F);
          
          for (int i = 0; i < target.rows; i++) {
               for (int j = 0; j < target.cols; j++) {
                    for (int cnt = 0; cnt < wsize * wsize; cnt++) {
                         target_featureset.at<float> (i * target.cols + j,
                                                      cnt) = target_LARK[cnt].at<float> (i, j);
                    }
               }
          }
          
          cv::Mat targetfeature;
          targetfeature.create(target_featureset.rows, maxComponents,
                               target_featureset.type());
          
          for (int j = 0; j < target_featureset.rows; j++) {
               cv::Mat vec = target_featureset.row(j);
               cv::Mat coeffs = targetfeature.row(j);
               //	pca.project(vec, coeffs);
               cout << "vec.type(): " << vec.type() << endl;
               cout << "means.type(): " << means.type() << endl;
               cout << "eigenvectors.t().type(): " << eigenvectors.t().type() << endl;
               coeffs = (vec - means)*eigenvectors.t();               
          }                    

          // make sure that the histogram has proper size and type

          for (int cnt = 0; cnt < targetfeature.cols; cnt++)
          {
               TF[0][cnt].create(target.rows, target.cols, CV_32F);

               for (int i = 0; i < target.rows; i++)
                    for (int j = 0; j < target.cols; j++) {
                         if (!binaryfeaturemode) {
                              TF[0][cnt].at<float> (i, j) = targetfeature.at<
                                   float> (i * target.cols + j, cnt);
                         } else {
                              TF[0][cnt].at<float> (i, j) = 1 / (1 + exp(
                                                                      -coeff * targetfeature.at<float> (i
                                                                                                        * target.cols + j, cnt))) - 0.5;
                         }
                    }

               for (int i = 1; i < numScale; i++)
               {
                    resize(TF[0][cnt], TF[i][cnt], cv::Size(),
                           scales.at<float> (0, i),
                           scales.at<float> (0, i), cv::INTER_LANCZOS4);
               }

          }

     }



     void LARKS::GradientImage(const cv::Mat& gray, cv::Mat& GradImage)
     {
          int rsize = 2;
          cv::Mat_<unsigned char> gray1(gray.rows + rsize * 2, gray.cols + rsize * 2);
          cv::copyMakeBorder(gray, gray1, rsize, rsize, rsize, rsize,
                             cv::BORDER_REPLICATE);

          //waitKey(0);
          // Gradient image
          float Y[] = { 0.0102, 0.0118, 0, -0.0118, -0.0102, 0.1060, 0.1225, 0,
                        -0.1225, -0.1060, 0.2316, 0.2675, 0, -0.2675, -0.2316, 0.1060,
                        0.1225, 0, -0.1225, -0.1060, 0.0102, 0.0118, 0, -0.0118,
                        -0.0102 };

          float X[] = { 0.0102, 0.1060, 0.2316, 0.1060, 0.0102, 0.0118, 0.1225,
                        0.2675, 0.1225, 0.0118, 0, 0, 0, 0, 0, -0.0118, -0.1225,
                        -0.2675, -0.1225, -0.0118, -0.0102, -0.1060, -0.2316, -0.1060,
                        -0.0102 };

          GradImage.create(gray1.rows, gray1.cols, CV_8U);

          cv::Mat_<unsigned char> GXimg(gray1.rows, gray1.cols);
          cv::Mat_<unsigned char> GYimg(gray1.rows, gray1.cols);

          cv::Mat_<float> GX(5, 5);
          cv::Mat_<float> GY(5, 5);

          for (int i = 0; i < 5; i++)
               for (int j = 0; j < 5; j++) {
                    GX(i, j) = X[i * 5 + j];
                    GY(i, j) = Y[i * 5 + j];
               }
     
          cv::filter2D(gray1, GXimg, GradImage.depth(), GX);
          cv::filter2D(gray1, GYimg, GradImage.depth(), GY);

          cv::add(GXimg, GYimg, GradImage);
          GradImage /= 2;
     }

     
     void TrainingTemplate::Training(std::vector<LARKFeatureTemplates> &models, 
                                     array_type2& query_mask, array_type3& QF, 
                                     double_type3& QF_norm, cv::Mat& labels)
     {
          numTemplate = models.size();
          numRotation = 1;
          numScale = 2;
          maxComponents = 4;
          int wsize = 5;
          int dfactor = 4;
          int coeff = 30;
          float factor = 0.25;
          bool binaryfeaturemode = false;

          array_type2 query(boost::extents[numTemplate][numRotation]);
          QF.resize(boost::extents[numTemplate][numRotation][maxComponents]);
          query_mask.resize(boost::extents[numTemplate][numRotation]);
          QF_norm.resize(boost::extents[numTemplate][numRotation][numScale]);
          labels.create(numTemplate,1,CV_8U);

          // Take each model, pull it out, resize based on factor
          for (unsigned int i = 0; i < models.size(); i++) {
               cv::Mat img, mask;
               models[i].img_.copyTo(img);
               models[i].mas_.copyTo(mask);
               cv::resize(img, query[i][0], cv::Size(), factor, factor, cv::INTER_LANCZOS4);
               cv::resize(mask, query_mask[i][0], cv::Size(), factor, factor, cv::INTER_LANCZOS4);
          }

          array_type2 query_featureset(boost::extents[numTemplate][numRotation]);
          array_type3 queries(boost::extents[numTemplate][numRotation][wsize*wsize]);

          int dim = 0;

          for (int n = 0; n < numTemplate; n++) {
               for (int m = 0; m < numRotation; m++) {

                    cv::Mat sC11, sC12, sC22;
                    LARK.computeCovariance(query[n][m], wsize, dfactor, sC11, sC12, sC22);
                    array_type1 query_temp(boost::extents[wsize * wsize]);

                    LARK.computeLARK(query[n][m].rows, query[n][m].cols, wsize, sC11, sC12, sC22,
                                     query_temp);

                    query_featureset[n][m].create(query[n][m].rows * query[n][m].cols, wsize
                                                  * wsize, CV_32F);
                    
                    for (int cnt = 0; cnt < wsize * wsize; cnt++) {
                         query_temp[cnt].copyTo(queries[n][m][cnt]);
                         for (int i = 0; i < query[n][m].rows; i++) {
                              for (int j = 0; j < query[n][m].cols; j++) {
                                   query_featureset[n][m].at<float> (i * query[n][m].cols + j,cnt) = queries[n][m][cnt].at<float> (i, j);
                              }
                         }
                    }
                    dim = dim + query[n][m].cols * query[n][m].rows;
               }
          }

          cv::Mat query_featureset_mask;
          query_featureset_mask.create(dim, wsize * wsize, CV_32F);

          int valid_points = 0;
          for (int cnt = 0; cnt < wsize * wsize; cnt++) {
               valid_points = 0;
               for (int rot = 0; rot < numRotation; rot++) {
                    for (int m = 0; m < numTemplate; m++) {
                         for (int i = 0; i < query_mask[m][rot].rows; i++) {
                              for (int j = 0; j < query_mask[m][rot].cols; j++) {
                                   if (query_mask[m][rot].at<unsigned char> (i, j) > 10) {
                                        query_featureset_mask.at<float> (valid_points, cnt)
                                             = queries[m][rot][cnt].at<float> (i, j);
                                        valid_points++;
                                   }
                              }
                         }
                    }
               }
          }

          cout << "TT.Training: valid_points: " << valid_points << endl;

          pca(query_featureset_mask(cv::Range(0, valid_points - 1), 
                                    cv::Range(0, wsize * wsize)), cv::Mat(), 
              CV_PCA_DATA_AS_ROW,
              maxComponents);

          array_type2 queryfeature(boost::extents[numTemplate][numRotation]);

          int skip[] = {5,5,5,4};
          
          array_type3 QF1(boost::extents[numTemplate][numRotation][maxComponents]);

          for (int tem = 0; tem < numTemplate; tem++) {
               for (int rot = 0; rot < numRotation; rot++) {
                    queryfeature[tem][rot].create(query[tem][rot].rows*query[tem][rot].cols, maxComponents,CV_32F);
                    for (int i = 0; i < query_mask[tem][rot].rows; i++) {
                         for (int j = 0; j < query_mask[tem][rot].cols; j++) {
                              cv::Mat vec = query_featureset[tem][rot].row(i* query[tem][rot].cols + j);
                              cv::Mat coeffs = queryfeature[tem][rot].row(i*query[tem][rot].cols + j);
                              pca.project(vec, coeffs);
                         }
                    }

                    for (int cnt = 0; cnt < maxComponents; cnt++) {
                         QF[tem][rot][cnt].create(query_mask[tem][rot].rows, query_mask[tem][rot].cols,
                                                  CV_32F);
                         double factor = static_cast<double>( query_mask[0][0].cols)/ static_cast<double>(query_mask[tem][rot].cols);

                         QF1[tem][rot][cnt].create(cv::Size(query_mask[0][0].cols, round(query_mask[tem][rot].rows*factor) ), CV_32F);

                         for (int i = 0; i < query_mask[tem][rot].rows; i++) {
                              for (int j = 0; j < query_mask[tem][rot].cols; j++) {
                                   if (!binaryfeaturemode) {
					QF[tem][rot][cnt].at<float> (i, j)
                                             = queryfeature[tem][rot].at<float> (i
                                                                                 * query_mask[tem][rot].cols + j, cnt);
                                   } else {
					QF[tem][rot][cnt].at<float> (i, j) = 1 / (1
                                                                                  + exp(-coeff * queryfeature[tem][rot].at<float> (i
                                                                                                                                   * query_mask[tem][rot].cols + j, cnt))) - 0.5;
                                   }
                              }
                         }
                         resize(QF[tem][rot][cnt], QF1[tem][rot][cnt], QF1[tem][rot][cnt].size(), 0,0, cv::INTER_LANCZOS4);
                    }

                    for (int m = 0; m < numScale; m++) {
                         QF_norm[tem][rot][m] = 0;

                         for (int i = 0; i < query_mask[tem][rot].rows; i = i + skip[m]) {
                              for (int j = 0; j < query_mask[tem][rot].cols; j = j + skip[m]) {
                                   for (int cnt = 0; cnt < queryfeature[tem][rot].cols; cnt++){
                                        if (query_mask[tem][rot].at<unsigned char> (i, j) > 10) {
                                             QF_norm[tem][rot][m] += pow(QF[tem][rot][cnt].at<float> (i, j), 2);
                                        }
                                   }
                              }
                         }                         
                         QF_norm[tem][rot][m] = sqrt(QF_norm[tem][rot][m]);
                    }
               }
          }

          int minRow = 640;
          for (int tem = 0; tem < numTemplate; tem++) {
               if ( minRow > QF1[tem][0][0].rows ) {
                    minRow = QF1[tem][0][0].rows;
               }
          }

          cv::Mat QF2(numTemplate, numRotation*maxComponents*minRow*query_mask[0][0].cols, CV_32F);

          for (int tem = 0; tem < numTemplate; tem++) {
               for (int rot = 0; rot < numRotation; rot++) {
                    for (int cnt = 0; cnt < maxComponents; cnt++) {
                         for (int i = 0; i < minRow; i++) {
                              for (int j = 0; j < query_mask[0][0].cols; j++) {
                                   QF2.at<float>(tem,rot*maxComponents*minRow*query_mask[0][0].cols + cnt*minRow*query_mask[0][0].cols + i*query_mask[0][0].cols ) = QF1[tem][rot][cnt].at<float>(i,j);
                              }
                         }
                    }
               }
          }

          cv::TermCriteria criteria;
          criteria.type = 5;

          //cv::Mat* centers = new cv::Mat[1];
          //cv::Mat centers;
          // TODO: Why is the clusterCount 2? I had to set it to 1.
          int clusterCount = 2; // K
          cv::Mat centers(clusterCount, 1, QF2.type());          

          cout << "QF2: " << QF2.size() << endl;

          //cv::kmeans(QF2, clusterCount, labels, criteria, 1, cv::KMEANS_PP_CENTERS, centers);

          bool isrow = (QF2.rows == 1 && QF2.channels() > 1);
          int N = !isrow ? QF2.rows : QF2.cols;
          cout << "Chan: " << QF2.channels() << endl;
          cout << "Rows: " << QF2.rows << endl;
          cout << "Cols: " << QF2.cols << endl;
          cout << "N: " << N << endl;
          cout << "K: " << clusterCount << endl;

          cv::kmeans(QF2, clusterCount, labels, 
                     cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 
                     1, cv::KMEANS_PP_CENTERS, centers);          

          array_type3 Cent(boost::extents[2][numRotation][maxComponents]);
          for (int i = 0; i < labels.rows ; i++)
               cout << labels.at<int>(i,0) << endl;


          cv::Mat eigenvectors = pca.eigenvectors;

          cv::Mat eigenvalues = pca.eigenvalues;

//	cout << "rows " << eigenvectors.rows << " cols " << eigenvectors.cols << endl;
//		cout << "rows " << eigenvalues.rows << " cols "	<< eigenvalues.cols << endl;
          for (int i = 0; i < eigenvalues.rows; i++) {
               std::cout << eigenvalues.at<float> (i, 0) << std::endl;
          }

          for (int i = 0; i < maxComponents; i++) {
               cv::Mat eigenImage(wsize, wsize, CV_32F);
               cv::Mat eigenImage1(wsize * 10, wsize * 10, CV_32F);
               for (int m = 0; m < wsize; m++)
                    for (int n = 0; n < wsize; n++) {
                         eigenImage.at<float> (m, n)
                              = eigenvectors.at<float> (i, m * wsize + n);
                    }
               cv::resize(eigenImage, eigenImage1, cv::Size(), 10, 10,
                      cv::INTER_LANCZOS4);
               std::stringstream kkk;
               kkk << "eigen ";
               kkk << i;
               cv::namedWindow(kkk.str(), 1);
               cv::imshow(kkk.str(), eigenImage1 * 10);


               cv::waitKey(3);
          }
     }


     void LARKS::endTraining(const std::string& name)
     {          
          TT.Training(models, query_mask, QF, QF_norm, label);
          cv::PCA pca = TT.get_pca();
          eigenvector = pca.eigenvectors;
          mean = pca.mean;
          numTemplate = TT.numTemplate;
          numRotation = TT.numRotation;
          numScale = TT.numScale;
          maxComponents = TT.maxComponents;        
          //model_storage_->save(name, getName(), *this);                   
          //this->save_model(name, "LARK", *this);
     }

     void LARKS::save_model(const std::string& name, const std::string& detector, const std::string& model_blob)
     {
          std::string directory_ = ".";
          if (!is_directory(path(directory_)/detector)) {
               create_directory(path(directory_)/detector);
          }
          ofstream ofile(path(directory_)/detector/name);
          std::vector<char> buffer(model_blob.begin(), model_blob.end());
          ofile.write(&buffer[0], buffer.size());
          ofile.close();
     }



     //void LARKS::loadModels(const std::vector<std::string>& model)
     void LARKS::loadModels()
     {
          std::cout << "size " << models.size() << std::endl;          
          num_models_ = models.size();
          
          eigenvectors.resize(num_models_);
          means.resize(num_models_);
          QFs.resize(num_models_);
          query_masks.resize(num_models_);
          QF_norms.resize(num_models_);
          labels.resize(num_models_);
          numTemplates.resize(num_models_);
          maxVals.resize(num_models_);
          offset_x.resize(num_models_);
          offset_y.resize(num_models_);
          Prev_max.resize(num_models_);
          index_templates.resize(num_models_);
          index_scales.resize(num_models_);
          index_rotations.resize(num_models_);
          region_index.resize(num_models_);
          
          for ( id = 0 ; id < num_models_; id++) {
               //std::cout << "model: " << model[id] << std::endl;
               //model_storage_->load(model[id],getName(),*this);

               maxVals[id].resize(boost::extents[numTemplates[id]][numScale][numRotation]);
               offset_x[id].resize(boost::extents[numTemplates[id]]);
               offset_y[id].resize(boost::extents[numTemplates[id]]);
               Prev_max[id] = 0;
               region_index[id].resize(boost::extents[numTemplates[id]][numScale][numRotation]);
          }

          Py_QFs.resize(num_models_);
          Py_query_masks.resize(num_models_);
          Py_QF_norms.resize(num_models_);

          const float scales1[] = { 1.0, 1.2, 1.4, 0.8 };
          scales.create(1, numScale, CV_32F);
          for (int i = 0; i < numScale; i++) {
               scales.at<float> (0, i) = scales1[i];
          }
     }
}
