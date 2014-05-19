#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <opencv_workbench/LARKS/LARKS.h>

namespace larks {

     using std::cout;
     using std::endl;

     void Larkcomputation::computeCovariance(const cv::Mat& gray, int wsize, 
                                             const int dfactor, cv::Mat& sC11, 
                                             cv::Mat& sC12, cv::Mat& sC22)
     {
          double t = (double) cv::getTickCount();

          int rsize = 2;
          cv::Mat_<float> gray1(gray.rows + rsize * 2, gray.cols + rsize * 2);
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

          cv::Mat_<float> GXimg(gray1.rows, gray1.cols);
          cv::Mat_<float> GYimg(gray1.rows, gray1.cols);
          cv::Mat_<float> GX(5, 5);
          cv::Mat_<float> GY(5, 5);

          for (int i = 0; i < 5; i++) {
               for (int j = 0; j < 5; j++) {
                    GX(i, j) = X[i * 5 + j];
                    GY(i, j) = Y[i * 5 + j];
               }
          } 

          cv::filter2D(gray1, GXimg, GXimg.depth(), GX);
          cv::filter2D(gray1, GYimg, GXimg.depth(), GY);

          t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
          //	cout << "filter2D: " << t << endl;

          t = (double) cv::getTickCount();

          GXimg = GXimg / 255;

          GYimg = GYimg / 255;

          cv::Mat_<float> GXimg1, GYimg1;

          resize(cv::Mat(GXimg, cv::Rect(rsize, rsize, gray.cols, gray.rows)), 
                 GXimg1, cv::Size(), 1.0 / dfactor, 1.0 / dfactor, 
                 cv::INTER_LANCZOS4);

          resize(cv::Mat(GYimg, cv::Rect(rsize, rsize, gray.cols, gray.rows)), 
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

          cv::Mat_<double> GX_2(GXimg.rows, GXimg.cols);
          cv::Mat_<double> GY_2(GXimg.rows, GXimg.cols);
          cv::Mat_<double> GXY(GXimg.rows, GXimg.cols);
          cv::Mat_<double> GX_2integral(GXimg.rows, GXimg.cols);
          cv::Mat_<double> GY_2integral(GXimg.rows, GXimg.cols);
          cv::Mat_<double> GXY_integral(GXimg.rows, GXimg.cols);

          pow(GXimg, 2.0, GX_2);
          pow(GYimg, 2.0, GY_2);
          multiply(GXimg, GYimg, GXY);

          integral(GX_2, GX_2integral);
          integral(GY_2, GY_2integral);
          integral(GXY, GXY_integral);

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


     }

     void Larkcomputation::computeLARK(const int rows, const int cols, const 
                                       int wsize, cv::Mat& sC11, cv::Mat& sC12,
                                       cv::Mat& sC22, array_type1& temp)
     {
          // Exend border
          double t = (double) cv::getTickCount();
          int rsize = (wsize - 1) / 2;
          cv::Mat_<float> C11(rows, cols);
          cv::Mat_<float> C12(rows, cols);
          cv::Mat_<float> C22(rows, cols);

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
          for (int i = -rsize; i <= rsize; i++)
               XX.at<float> (i + rsize) = i;

          cv::Mat X2 = repeat(XX, wsize, 1);

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
          for (int i = 0; i < wsize; i++)
               for (int j = 0; j < wsize; j++)
               {

                    temp[i * wsize + j] = cv::Mat::zeros(rows, cols, CV_32F);
                    if (!(i == rsize && j == rsize))
                    {

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

                    add(norm_img, temp[i * wsize + j], norm_img);

               }

          for (int i = 0; i < wsize; i++)
               for (int j = 0; j < wsize; j++)
               {
                    temp[i * wsize + j] /= norm_img;
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

     void LARKFeatureTemplates::computeFeatures(const cv::Mat &whole_gray, const cv::Rect & roi, const cv::Mat & mask, int index, cv::PCA& pca1, int cols1)
     {
          cv::Mat objImg(whole_gray, roi);

          objImg.copyTo(img);
          mask.copyTo(mas);

          int wsize = 5;
          float downfactor = 0.9;
          int dfactor = 4;
          int maxComponents = 3;

          cv::Mat img, img1, M1;

          resize(objImg, img, cv::Size(), downfactor, downfactor, cv::INTER_LANCZOS4);
          resize(mask, M, cv::Size(), downfactor, downfactor, cv::INTER_LANCZOS4);

          if (index != 0)
          {
               double factor = static_cast<double>( cols1)/ static_cast<double>(img.cols);
               img1.create(cv::Size(cols1, round(img.rows*factor) ), CV_8U);
               M1.create(cv::Size(cols1, round(img.rows*factor) ), CV_8U);
               resize(img, img1, img1.size(), 0,0, cv::INTER_LANCZOS4);
               resize(M, M1, M.size(), 0,0, cv::INTER_LANCZOS4);
               M.release();
               M1.copyTo(M);
               img.release();
               img1.copyTo(img);
          }


          Larkcomputation LARK;

          cv::Mat sC11, sC12, sC22;
          LARK.computeCovariance(img, wsize, dfactor, sC11, sC12, sC22);
          array_type1 query_temp(boost::extents[wsize * wsize]);

          int rows = img.rows;
          cols = img.cols;

          LARK.computeLARK(img.rows, img.cols, wsize, sC11, sC12, sC22, query_temp);

          cv::Mat featureset(rows*cols, wsize*wsize, CV_32F);
          for (int cnt = 0; cnt < wsize * wsize; cnt++)
          {
               for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                    {
                         featureset.at<float>(i*cols+j,cnt) = query_temp[cnt].at<float>(i,j);
                    }

          }

          cv::Mat queryfeature(rows*cols, maxComponents,CV_32F);

          if (index == 0)
          {
               cv::Mat featureset_mask;
               featureset_mask.create(rows*cols, wsize * wsize, CV_32F);


               int valid_points = 0;
               for (int cnt = 0; cnt < wsize * wsize; cnt++)
               {
                    valid_points = 0;

                    for (int i = 0; i < rows; i++)
                         for (int j = 0; j < cols; j++)
                         {

                              if (M.at<unsigned char> (i, j) > 0)
                              {
                                   featureset_mask.at<float> (valid_points, cnt)
                                        = query_temp[cnt].at<float> (i, j);
                                   valid_points++;
                              }


                         }

               }

               pca(featureset_mask(cv::Range(0, valid_points - 1), cv::Range(0,
                                                                     wsize * wsize)), cv::Mat(), CV_PCA_DATA_AS_ROW,
                   maxComponents);

               cv::Mat eigenvectors = pca.eigenvectors;

               cv::Mat eigenvalues = pca.eigenvalues;


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
                    resize(eigenImage, eigenImage1, cv::Size(), 10, 10,
                           cv::INTER_LANCZOS4);
                    std::stringstream kkk;
                    kkk << "eigen ";
                    kkk << i;
                    cv::namedWindow(kkk.str(), 1);
                    cv::imshow(kkk.str(), eigenImage1 * 10);

                    //namedWindow("queryfeature",1);
                    //		imshow("queryfeature",QF[i]*10);

               }


               for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                    {
                         cv::Mat vec = featureset.row(i* cols + j);
                         cv::Mat coeffs = queryfeature.row(i*cols + j);
                         pca.project(vec, coeffs);
                    }
          }
          else
          {

               for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                    {
                         cv::Mat vec = featureset.row(i* cols + j);
                         cv::Mat coeffs = queryfeature.row(i*cols + j);
                         pca1.project(vec, coeffs);
                    }

          }



          QF.resize(boost::extents[maxComponents]);

          for (int cnt = 0; cnt < maxComponents; cnt++)
          {
               QF[cnt].create(rows, cols, CV_32F);
               for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                    {

                         QF[cnt].at<float> (i, j) = queryfeature.at<float> (i* cols + j, cnt);
                    }

               //waitKey(30);


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

     //void LARKs::startTraining(const std::string& name)
     //{
     //     if (models.size() > 0)
     //          models.erase(models.begin(),models.begin()+ models.size());
     //     index = 0;
     //}

}
