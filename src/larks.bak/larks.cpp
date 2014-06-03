/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv_workbench/larks/larks.h>
#include "opencv2/highgui/highgui.hpp"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <math.h>

#include <boost/foreach.hpp>
//#include <opencv_workbench/recognition_pipeline/nodelets/type_conversions.h>
#include <boost/serialization/vector.hpp>


#include <fstream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

using namespace cv;

//REGISTER_TYPE_CONVERSION(recognition_pipeline::Rect, cv::Rect,
//                         (x,x)
//                         (y,y)
//                         (width,width)
//                         (height,height)
//     )
//
//REGISTER_TYPE_CONVERSION(recognition_pipeline::Detection, larks::LARKs,
//                         (label,name)
//                         (score,score)
//                         (mask.roi,roi)
//                         (mask.mask,mask)
//     )


namespace larks
{

     void Larkcomputation::computeCovariance(const Mat& gray, int wsize, const int dfactor, Mat& sC11, Mat& sC12, Mat& sC22)
     {
          double t = (double) getTickCount();

          int rsize = 2;
          Mat_<float> gray1(gray.rows + rsize * 2, gray.cols + rsize * 2);
          copyMakeBorder(gray, gray1, rsize, rsize, rsize, rsize,
                         BORDER_REPLICATE);

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

          Mat_<float> GXimg(gray1.rows, gray1.cols);
          Mat_<float> GYimg(gray1.rows, gray1.cols);
          Mat_<float> GX(5, 5);
          Mat_<float> GY(5, 5);

          for (int i = 0; i < 5; i++)
               for (int j = 0; j < 5; j++)
               {
                    GX(i, j) = X[i * 5 + j];
                    GY(i, j) = Y[i * 5 + j];
               }

          filter2D(gray1, GXimg, GXimg.depth(), GX);
          filter2D(gray1, GYimg, GXimg.depth(), GY);

          t = ((double) getTickCount() - t) / getTickFrequency();
          //	cout << "filter2D: " << t << endl;

          t = (double) getTickCount();

          GXimg = GXimg / 255;

          GYimg = GYimg / 255;

          Mat_<float> GXimg1, GYimg1;

          resize(Mat(GXimg, cv::Rect(rsize, rsize, gray.cols, gray.rows)), GXimg1,
                 Size(), 1.0 / dfactor, 1.0 / dfactor, INTER_LANCZOS4);
          resize(Mat(GYimg, cv::Rect(rsize, rsize, gray.cols, gray.rows)), GYimg1,
                 Size(), 1.0 / dfactor, 1.0 / dfactor, INTER_LANCZOS4);

          // Covariance matrix computation

          Mat_<float> M(2, 2);
          Mat tmp = Mat::zeros(2, 2, CV_32FC1);
          Mat tmp1 = Mat::zeros(2, 2, CV_32FC1);
          Mat v1, v2;

          double alpha = 0.33;

          double len;
          if (wsize == 3)
               len = 20;
          else if (wsize == 5)
               len = 50;
          else
               len = 75.5398;

          double s1, para1, lambda1, lambda2, theta;

          Mat_<double> GX_2(GXimg.rows, GXimg.cols);
          Mat_<double> GY_2(GXimg.rows, GXimg.cols);
          Mat_<double> GXY(GXimg.rows, GXimg.cols);
          Mat_<double> GX_2integral(GXimg.rows, GXimg.cols);
          Mat_<double> GY_2integral(GXimg.rows, GXimg.cols);
          Mat_<double> GXY_integral(GXimg.rows, GXimg.cols);

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

                    double kk = sqrt(fabs(pow(M(0, 0) + M(1, 1), 2.0) - 4.0 * (M(0,
                                                                                 0) * M(1, 1) - M(0, 1) * M(0, 1))));
                    lambda1 = fabs((M(0, 0) + M(1, 1) + kk) / 2.0);
                    lambda2 = fabs((M(0, 0) + M(1, 1) - kk) / 2.0);
                    theta = atan(-(M(0, 0) + M(0, 1) - lambda1) / (M(1, 1)
                                                                   + M(0, 1) - lambda1));


                    if (cvIsNaN(theta))
                         theta = 0.0;
                    v1 = (Mat_<float> (2, 1) << cos(theta), sin(theta));
                    v2 = (Mat_<float> (2, 1) << -sin(theta), cos(theta));
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
                         waitKey(0);

                    }
                    col_count++;
               }
               row_count++;
          }

          t = ((double) getTickCount() - t) / getTickFrequency();
          //	cout << "Covariance: " << t << endl;
//	cout << "Covariance: " << t << endl;


     }

     void Larkcomputation::computeLARK(const int rows, const int cols, const int wsize, Mat& sC11, Mat& sC12, Mat& sC22, array_type1& temp)
     {
          // Exend border
          double t = (double) getTickCount();
          int rsize = (wsize - 1) / 2;
          Mat_<float> C11(rows, cols);
          Mat_<float> C12(rows, cols);
          Mat_<float> C22(rows, cols);

          resize(sC11, C11, C11.size(), 0, 0, INTER_LANCZOS4);
          resize(sC12, C12, C12.size(), 0, 0, INTER_LANCZOS4);
          resize(sC22, C22, C22.size(), 0, 0, INTER_LANCZOS4);

          Mat C11_1(rows + rsize * 2, cols + rsize * 2, CV_64F);
          Mat C12_1(rows + rsize * 2, cols + rsize * 2, CV_64F);
          Mat C22_1(rows + rsize * 2, cols + rsize * 2, CV_64F);

          copyMakeBorder(C11, C11_1, rsize, rsize, rsize, rsize, BORDER_REPLICATE);
          copyMakeBorder(C12, C12_1, rsize, rsize, rsize, rsize, BORDER_REPLICATE);
          copyMakeBorder(C22, C22_1, rsize, rsize, rsize, rsize, BORDER_REPLICATE);

          // Spatial distance computation


          Mat XX = Mat::zeros(1, wsize, CV_32FC1);
          for (int i = -rsize; i <= rsize; i++)
               XX.at<float> (i + rsize) = i;

          Mat X2 = repeat(XX, wsize, 1);

          Mat_<double> X1 = X2.t();

          Mat_<float> X12 = 2 * X1.mul(X2);
          Mat_<float> X11 = X1.mul(X1);
          Mat_<float> X22 = X2.mul(X2);

          Mat_<float> X1X1(rows, cols);
          Mat_<float> X2X2(rows, cols);
          Mat_<float> X1X2(rows, cols);

          Mat norm_img = Mat::zeros(rows, cols, CV_32F);

          Mat_<float> Comp1, Comp2, Comp3;
          Mat temp1;

          // Geodesic distance computation
          for (int i = 0; i < wsize; i++)
               for (int j = 0; j < wsize; j++)
               {

                    temp[i * wsize + j] = Mat::zeros(rows, cols, CV_32F);
                    if (!(i == rsize && j == rsize))
                    {

                         X1X1 = X11(i, j);
                         X1X2 = X12(i, j);
                         X2X2 = X22(i, j);
                         Comp1 = Mat(C11_1, cv::Rect(j, i, cols, rows));
                         Comp2 = Mat(C12_1, cv::Rect(j, i, cols, rows));
                         Comp3 = Mat(C22_1, cv::Rect(j, i, cols, rows));
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

          t = ((double) getTickCount() - t) / getTickFrequency();
//		cout << "LARK : " << t << endl;

/*	Mat LARK = Mat::zeros(rows, cols, CV_32F);
	for (int i = 0; i < rows - wsize + 1; i = i + wsize)
        for (int j = 0; j < cols - wsize + 1; j = j + wsize)
        for (int m = 0; m < wsize; m++)
        for (int n = 0; n < wsize; n++)
        LARK.at<float> (i + m, j + n) = temp[m * wsize + n].at<
        float> (i, j);

	Mat LARK1(rows * 5, cols * 5, CV_64F);
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

     void Saliency::ProtoObject(const Mat &SaliencyMap, Mat& thMap)
     {
          double minVal;
          double maxVal;
          Point minLoc;
          Point maxLoc;

          minMaxLoc(SaliencyMap, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

          //cout << "min " << minVal << " max " << maxVal << endl;

          thMap = Mat::zeros(SaliencyMap.rows, SaliencyMap.cols, CV_32F);

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


     void Saliency::computeSaliency(const array_type1& LARK, const int wsize, const int psize, Mat& SaliencyMap)
     {

          double t = (double) getTickCount();
          int rsize = (psize - 1) / 2;
          Mat_<float> *temp1;
          Mat_<float> *temp2;
          temp1 = new Mat_<float> [psize * psize];
          temp2 = new Mat_<float> [psize * psize];

          Mat_<double> patch(psize, psize);

          for (int i = 0; i < wsize * wsize; i++)
          {
               copyMakeBorder(LARK[i], temp1[i], rsize, rsize, rsize, rsize,
                              IPL_BORDER_REPLICATE);
               copyMakeBorder(temp1[i], temp2[i], rsize, rsize, rsize, rsize,
                              IPL_BORDER_REPLICATE);
          }

          double Norm_center, Norm_surround;

          double sigma = 0.5;

          int skip = 3;
          for (int m = 0; m < LARK[0].rows; m = m + skip)
               for (int n = 0; n < LARK[0].cols; n = n + skip) {
                    Mat_<double> center(wsize * wsize, (rsize + 1) * (rsize + 1));
                    for (int i = 0; i < wsize * wsize; i++)
                    {
                         patch = Mat(temp1[i], cv::Rect(n, m, psize, psize));

                         for (int o = 0; o < psize; o = o + 2)
                              for (int p = 0; p < psize; p = p + 2)
                              {
                                   center(i, (o / 2) * (rsize + 1) + (p / 2)) = patch(
                                        o, p);
                              }
                    }
                    Norm_center = norm(center, NORM_L2);

                    Mat_<double> surround(wsize * wsize, (rsize + 1) * (rsize + 1));
                    double s = 0.0;

                    for (int cnt1 = 0; cnt1 < rsize; cnt1++)
                         for (int cnt2 = 0; cnt2 < rsize; cnt2++)
                         {
                              double rho = 0;
                              for (int i = 0; i < wsize * wsize; i++)
                              {
                                   patch = Mat(temp2[i], cv::Rect(n + cnt2, m + cnt1,
                                                                  psize, psize));

                                   for (int o = 0; o < psize; o = o + 2)
                                        for (int p = 0; p < psize; p = p + 2)
                                        {
                                             surround(i, (o / 2) * (rsize + 1) + p / 2)
                                                  = patch(o, p);
                                        }
                              }
                              Norm_surround = norm(surround, NORM_L2);

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

          Mat Saliency1(SaliencyMap.rows * 10, SaliencyMap.cols * 10, CV_64F);
          resize(SaliencyMap, Saliency1, Saliency1.size(), 0, 0, CV_INTER_NN);

          t = ((double) getTickCount() - t) / getTickFrequency();
          //cout << "Saliency: " << t << endl;

          //namedWindow("Saliency", CV_WINDOW_AUTOSIZE);
          //imshow("Saliency", Saliency1);


     }

     void TrainingTemplate::Training(std::vector<LARKFeatureTemplates> &models, array_type2& query_mask, array_type3& QF, double_type3& QF_norm, Mat& labels)
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

          for (unsigned int i = 0; i < models.size(); i++)
          {
               Mat img, mask;
               models[i].img.copyTo(img);
               models[i].mas.copyTo(mask);
               resize(img, query[i][0], Size(), factor, factor,INTER_LANCZOS4);
               resize(mask, query_mask[i][0], Size(), factor, factor, INTER_LANCZOS4);

          }


          array_type2 query_featureset(boost::extents[numTemplate][numRotation]);
          array_type3 queries(boost::extents[numTemplate][numRotation][wsize*wsize]);

          int dim = 0;

          for (int n = 0; n < numTemplate; n++)
               for (int m = 0; m < numRotation; m++)
               {

                    Mat sC11, sC12, sC22;
                    LARK.computeCovariance(query[n][m], wsize, dfactor, sC11, sC12, sC22);
                    array_type1 query_temp(boost::extents[wsize * wsize]);

                    LARK.computeLARK(query[n][m].rows, query[n][m].cols, wsize, sC11, sC12, sC22,
                                     query_temp);
                    query_featureset[n][m].create(query[n][m].rows * query[n][m].cols, wsize
                                                  * wsize, CV_32F);

                    for (int cnt = 0; cnt < wsize * wsize; cnt++)
                    {
                         query_temp[cnt].copyTo(queries[n][m][cnt]);
                         for (int i = 0; i < query[n][m].rows; i++)
                              for (int j = 0; j < query[n][m].cols; j++)
                              {
                                   query_featureset[n][m].at<float> (i * query[n][m].cols + j,cnt) = queries[n][m][cnt].at<float> (i, j);
                              }
                    }
                    dim = dim + query[n][m].cols * query[n][m].rows;

               }




          Mat query_featureset_mask;
          query_featureset_mask.create(dim, wsize * wsize, CV_32F);

          int valid_points = 0;
          for (int cnt = 0; cnt < wsize * wsize; cnt++)
          {
               valid_points = 0;
               for (int rot = 0; rot < numRotation; rot++)
                    for (int m = 0; m < numTemplate; m++)
                         for (int i = 0; i < query_mask[m][rot].rows; i++)
                              for (int j = 0; j < query_mask[m][rot].cols; j++)
                              {

                                   if (query_mask[m][rot].at<unsigned char> (i, j)	> 10)
                                   {
                                        query_featureset_mask.at<float> (valid_points, cnt)
                                             = queries[m][rot][cnt].at<float> (i, j);
                                        valid_points++;
                                   }

                              }

          }

//	cout << "valid_points " << valid_points << endl;

          pca(query_featureset_mask(Range(0, valid_points - 1), Range(0,
                                                                      wsize * wsize)), Mat(), CV_PCA_DATA_AS_ROW,
              maxComponents);
          array_type2 queryfeature(boost::extents[numTemplate][numRotation]);

          int skip[] = { 5,5, 5, 4 };


          array_type3 QF1(boost::extents[numTemplate][numRotation][maxComponents]);


          for (int tem = 0; tem < numTemplate; tem++)
               for (int rot = 0; rot < numRotation; rot++)
               {
                    queryfeature[tem][rot].create(query[tem][rot].rows*query[tem][rot].cols, maxComponents,CV_32F);
                    for (int i = 0; i < query_mask[tem][rot].rows; i++)
                         for (int j = 0; j < query_mask[tem][rot].cols; j++)
                         {
                              Mat vec = query_featureset[tem][rot].row(i* query[tem][rot].cols + j);
                              Mat coeffs = queryfeature[tem][rot].row(i*query[tem][rot].cols + j);
                              pca.project(vec, coeffs);
                         }

                    for (int cnt = 0; cnt < maxComponents; cnt++)
                    {
                         QF[tem][rot][cnt].create(query_mask[tem][rot].rows, query_mask[tem][rot].cols,
                                                  CV_32F);
                         double factor = static_cast<double>( query_mask[0][0].cols)/ static_cast<double>(query_mask[tem][rot].cols);

                         QF1[tem][rot][cnt].create(Size(query_mask[0][0].cols, round(query_mask[tem][rot].rows*factor) ), CV_32F);

                         for (int i = 0; i < query_mask[tem][rot].rows; i++)
                              for (int j = 0; j < query_mask[tem][rot].cols; j++)
                              {
                                   if (!binaryfeaturemode)
                                   {
					QF[tem][rot][cnt].at<float> (i, j)
                                             = queryfeature[tem][rot].at<float> (i
                                                                                 * query_mask[tem][rot].cols + j, cnt);
                                   }
                                   else
                                   {
					QF[tem][rot][cnt].at<float> (i, j) = 1 / (1
                                                                                  + exp(-coeff * queryfeature[tem][rot].at<float> (i
                                                                                                                                   * query_mask[tem][rot].cols + j, cnt))) - 0.5;
                                   }
                              }

                         resize(QF[tem][rot][cnt], QF1[tem][rot][cnt], QF1[tem][rot][cnt].size(), 0,0, INTER_LANCZOS4);

                    }

                    for (int m = 0; m < numScale; m++)
                    {
                         QF_norm[tem][rot][m] = 0;

                         for (int i = 0; i < query_mask[tem][rot].rows; i = i + skip[m])
                              for (int j = 0; j < query_mask[tem][rot].cols; j = j + skip[m])
                                   for (int cnt = 0; cnt < queryfeature[tem][rot].cols; cnt++)
                                   {

                                        if (query_mask[tem][rot].at<unsigned char> (i, j) > 10)
                                        {
                                             QF_norm[tem][rot][m] += pow(QF[tem][rot][cnt].at<float> (i, j), 2);
                                        }
                                   }

                         QF_norm[tem][rot][m] = sqrt(QF_norm[tem][rot][m]);
                    }


               }


          int minRow = 640;
          for (int tem = 0; tem < numTemplate; tem++)
          {
               if ( minRow > QF1[tem][0][0].rows )
                    minRow = QF1[tem][0][0].rows;
          }

          Mat QF2(numTemplate, numRotation*maxComponents*minRow*query_mask[0][0].cols, CV_32F);

          for (int tem = 0; tem < numTemplate; tem++)
               for (int rot = 0; rot < numRotation; rot++)
                    for (int cnt = 0; cnt < maxComponents; cnt++)
                    {

                         for (int i = 0; i < minRow; i++)
                              for (int j = 0; j < query_mask[0][0].cols; j++)
                              {
                                   QF2.at<float>(tem,rot*maxComponents*minRow*query_mask[0][0].cols + cnt*minRow*query_mask[0][0].cols + i*query_mask[0][0].cols )  = QF1[tem][rot][cnt].at<float>(i,j);

                              }
                    }



          cv::TermCriteria criteria;
          criteria.type = 5;

          Mat* centers = new Mat[1];

          kmeans(QF2, 2, labels, criteria, 1, KMEANS_PP_CENTERS, centers);

          array_type3 Cent(boost::extents[2][numRotation][maxComponents]);
          for (int i = 0; i < labels.rows ; i++)
               cout << labels.at<int>(i,0) << endl;


          Mat eigenvectors = pca.eigenvectors;

          Mat eigenvalues = pca.eigenvalues;

//	cout << "rows " << eigenvectors.rows << " cols " << eigenvectors.cols << endl;
//		cout << "rows " << eigenvalues.rows << " cols "	<< eigenvalues.cols << endl;
          for (int i = 0; i < eigenvalues.rows; i++) {
               std::cout << eigenvalues.at<float> (i, 0) << std::endl;
          }

          for (int i = 0; i < maxComponents; i++) {
               Mat eigenImage(wsize, wsize, CV_32F);
               Mat eigenImage1(wsize * 10, wsize * 10, CV_32F);
               for (int m = 0; m < wsize; m++)
                    for (int n = 0; n < wsize; n++) {
                         eigenImage.at<float> (m, n)
                              = eigenvectors.at<float> (i, m * wsize + n);
                    }
               resize(eigenImage, eigenImage1, Size(), 10, 10,
                      INTER_LANCZOS4);
               std::stringstream kkk;
               kkk << "eigen ";
               kkk << i;
               namedWindow(kkk.str(), 1);
               imshow(kkk.str(), eigenImage1 * 10);


               waitKey(3);
          }
     }


     void LARKFeatureTemplates::computeFeatures(const cv::Mat &whole_gray, const cv::Rect & roi, const cv::Mat & mask, int index, PCA& pca1, int cols1)
     {
          cv::Mat objImg(whole_gray, roi);

          objImg.copyTo(img);
          mask.copyTo(mas);

          int wsize = 5;
          float downfactor = 0.9;
          int dfactor = 4;
          int maxComponents = 3;

          Mat img, img1, M1;

          resize(objImg, img, Size(), downfactor, downfactor, INTER_LANCZOS4);
          resize(mask, M, Size(), downfactor, downfactor, INTER_LANCZOS4);

          if (index != 0)
          {
               double factor = static_cast<double>( cols1)/ static_cast<double>(img.cols);
               img1.create(Size(cols1, round(img.rows*factor) ), CV_8U);
               M1.create(Size(cols1, round(img.rows*factor) ), CV_8U);
               resize(img, img1, img1.size(), 0,0, INTER_LANCZOS4);
               resize(M, M1, M.size(), 0,0, INTER_LANCZOS4);
               M.release();
               M1.copyTo(M);
               img.release();
               img1.copyTo(img);
          }


          Larkcomputation LARK;

          Mat sC11, sC12, sC22;
          LARK.computeCovariance(img, wsize, dfactor, sC11, sC12, sC22);
          array_type1 query_temp(boost::extents[wsize * wsize]);

          int rows = img.rows;
          cols = img.cols;

          LARK.computeLARK(img.rows, img.cols, wsize, sC11, sC12, sC22, query_temp);

          Mat featureset(rows*cols, wsize*wsize, CV_32F);
          for (int cnt = 0; cnt < wsize * wsize; cnt++)
          {
               for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                    {
                         featureset.at<float>(i*cols+j,cnt) = query_temp[cnt].at<float>(i,j);
                    }

          }

          Mat queryfeature(rows*cols, maxComponents,CV_32F);

          if (index == 0)
          {
               Mat featureset_mask;
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

               pca(featureset_mask(Range(0, valid_points - 1), Range(0,
                                                                     wsize * wsize)), Mat(), CV_PCA_DATA_AS_ROW,
                   maxComponents);

               Mat eigenvectors = pca.eigenvectors;

               Mat eigenvalues = pca.eigenvalues;


               for (int i = 0; i < eigenvalues.rows; i++) {
                    std::cout << eigenvalues.at<float> (i, 0) << std::endl;
               }

               for (int i = 0; i < maxComponents; i++) {
                    Mat eigenImage(wsize, wsize, CV_32F);
                    Mat eigenImage1(wsize * 10, wsize * 10, CV_32F);
                    for (int m = 0; m < wsize; m++)
                         for (int n = 0; n < wsize; n++) {
                              eigenImage.at<float> (m, n)
                                   = eigenvectors.at<float> (i, m * wsize + n);
                         }
                    resize(eigenImage, eigenImage1, Size(), 10, 10,
                           INTER_LANCZOS4);
                    std::stringstream kkk;
                    kkk << "eigen ";
                    kkk << i;
                    namedWindow(kkk.str(), 1);
                    imshow(kkk.str(), eigenImage1 * 10);

                    //namedWindow("queryfeature",1);
                    //		imshow("queryfeature",QF[i]*10);

               }


               for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                    {
                         Mat vec = featureset.row(i* cols + j);
                         Mat coeffs = queryfeature.row(i*cols + j);
                         pca.project(vec, coeffs);
                    }
          }
          else
          {

               for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                    {
                         Mat vec = featureset.row(i* cols + j);
                         Mat coeffs = queryfeature.row(i*cols + j);
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

     float LARKFeatureTemplates::MatrixCosineMeasure(array_type1& QF1, Mat M1, array_type1& QF, Mat& M)
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

     void LARKs::startTraining(const std::string& name)
     {
          if (models.size() > 0)
               models.erase(models.begin(),models.begin()+ models.size());
          index = 0;
     }

/**
 * Trains the model on a new data instance.
 * @param name The name of the model
 * @param data Training data instance
 */

 
     void LARKs::trainInstance(const std::string& name, const recognition_pipeline::TrainingData& data)
     {
 
 
          ROS_INFO("LARKs: training instance: %s", name.c_str());
 
          cv::Mat img_gray;
          // compute the gradient summary image
          cv::Mat img = data.image;
 
          if(img.channels() != 1)
          {
               cv::cvtColor(img, img_gray, CV_BGR2GRAY);
          }
          else
          {
               img_gray = img;
          }
 
          //    Mat drawImg(data.image,data.roi);
          //    bitwise_not(drawImg,drawImg,data.mask);
 
          cv::Rect roi;
          cv::Mat mask;
          convert(data.roi,roi);
          convert(data.mask,mask);
 
 
          LARKFeatureTemplates crt_tpl;
 
          if (index == 0)
          {
               crt_tpl.computeFeatures(img_gray,roi,mask, index, crt_tpl.pca, crt_tpl.cols);
               models.push_back(crt_tpl);
               index++;
               printf("has ");
          }
          else
          {
               std::cout << "models.size()" << models.size() << std::endl;
               crt_tpl.computeFeatures(img_gray,roi,mask, index, models[0].pca, models[0].cols);
               float max_score = 0.0;
               for (unsigned int i = 0; i < models.size(); i++)
               {
                    float score = crt_tpl.MatrixCosineMeasure(models[i].QF,models[i].M, crt_tpl.QF, crt_tpl.M);
                    ROS_INFO("score= %f",score);
                    if (score > max_score)
                         max_score = score;
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
          printf("%d templates\n",index);
 
 
          cv::imshow("trainingImage", img_gray(roi));
          cv::waitKey(10);
 
 
 
     }

/**
 * Saves a trained model.
 * @param name model name
 */
     void LARKs::endTraining(const std::string& name)
     {

          TT.Training(models, query_mask, QF, QF_norm, label);
          PCA pca = TT.get_pca();
          eigenvector = pca.eigenvectors;
          mean = pca.mean;
          numTemplate = TT.numTemplate;
          numRotation = TT.numRotation;
          numScale = TT.numScale;
          maxComponents = TT.maxComponents;

          model_storage_->save(name, getName(), *this);
     }



     void LARKs::loadModels(const std::vector<std::string>& model)
     {
          std::cout << "size " << model.size() << std::endl;

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


          for ( id = 0 ; id < num_models_; id++)
          {
               std::cout << "model: " << model[id] << std::endl;
               model_storage_->load(model[id],getName(),*this);

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
          for (int i = 0; i < numScale; i++)
               scales.at<float> (0, i) = scales1[i];




     }


     void LARKs::detect()
     {
          detections_.detections.clear();

          Mat img4 = image_;

          Mat img2;

          cvtColor(img4,img2,CV_BGR2RGB);
          Detection d;


          Mat img1;
          cvtColor(img2, img1, CV_RGB2GRAY);

          Mat img3;
          img2.copyTo(img3);

          Mat target;
          resize(img1, target, Size(), factor, factor, INTER_LANCZOS4);

          // Covariance Computation

          Mat sC11, sC12, sC22;
          int wsize_s = 5;
          float f = 0.25;
          LARK.computeCovariance(target, wsize_s, dfactor, sC11, sC12, sC22);
          int block_width = img1.cols / x_block;
          int block_height = img1.rows / y_block;


          if (use_saliency) // Take advantage of Saliency detection
          {

               double t = (double) getTickCount();
               // LARK computation for Saliency
               int rows = target.rows / dfactor;
               int cols = target.cols / dfactor;

               array_type1
                    Saliency_LARK(boost::extents[wsize_s * wsize_s]);
               LARK.computeLARK(rows, cols, wsize_s, sC11, sC12, sC22,
				Saliency_LARK);

               Mat SaliencyMap;
               SaliencyMap.create(rows, cols, CV_64F);
               int psize = 5;
               SalientRegion.computeSaliency(Saliency_LARK, wsize_s, psize, SaliencyMap);

               Mat thMap;
               SalientRegion.ProtoObject(SaliencyMap, thMap);

               Mat Saliency_mask;
               Mat selectedObj = Mat::zeros(img1.size(), img1.type());

               resize(thMap, Saliency_mask, img1.size(), 0, 0, CV_INTER_NN);

               Mat Mask = (Saliency_mask > 0) / 255;
               img1.copyTo(selectedObj, Mask);



               start.resize(x_block * y_block);
               end.resize(x_block * y_block);

               vector<float> sum(x_block * y_block);
               int lineWidth = 3;


               block_table = Mat::zeros(x_block, y_block, CV_8U);
               AtleastSalient = 0;
               for (int i = 0; i < x_block; i++)
                    for (int j = 0; j < y_block; j++) {
                         start[i * y_block + j].x = i * block_width;
                         start[i * y_block + j].y = j * block_height;
                         end[i * y_block + j].x = (i + 1) * block_width - 1;
                         end[i * y_block + j].y = (j + 1) * block_height - 1;

                         Mat region = Mat(Mask, cv::Rect(
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

                              line(img3, Point(start[i * y_block + j].x,
                                               start[i * y_block + j].y), Point(
                                                    start[i * y_block + j].x, end[i
                                                                                  * y_block + j].y), white, lineWidth, CV_AA);
                              line(img3, Point(start[i * y_block + j].x,
                                               end[i * y_block + j].y), Point(end[i
                                                                                  * y_block + j].x,
                                                                              end[i * y_block + j].y),  white, lineWidth, CV_AA);
                              line(img3, Point(end[i * y_block + j].x, end[i
                                                                           * y_block + j].y), Point(end[i
                                                                                                        * y_block + j].x,
                                                                                                    start[i * y_block + j].y),  white, lineWidth, CV_AA);
                              line(img3, Point(end[i * y_block + j].x,
                                               start[i * y_block + j].y), Point(
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

               t = ((double) getTickCount() - t) / getTickFrequency();
               std::cout << " Saliency Computation: " << t << std::endl;
               use_saliency = false;

          }



          // LARK computation for target


          if (AtleastSalient)
          {

               double t = (double) getTickCount();




               int level = 3;
               vector<array_type2> TFs(num_models_);

               vector<array_type3> Py_TFs(num_models_);

               for (int i = 0; i < num_models_; i++)

               {
                    TFs[i].resize(boost::extents[numScale][maxComponents]);
                    getTF(wsize, target, sC11, sC12, sC22, binaryfeaturemode, maxComponents, numScale, TFs[i], means[i], eigenvectors[i]);


                    Py_TFs[i].resize(boost::extents[level][numScale][maxComponents]);
                    Py_QFs[i].resize(boost::extents[level][numTemplates[i]][numRotation][maxComponents]);
                    Py_query_masks[i].resize(boost::extents[level][numTemplates[i]][numRotation]);
                    Py_QF_norms[i].resize(boost::extents[level][numTemplates[i]][numRotation][numScale]);


                    pyramidFeatures(TFs[i], Py_TFs[i], QFs[i], Py_QFs[i], QF_norms[i], Py_QF_norms[i], query_masks[i], Py_query_masks[i], level, numTemplates[i]);


                    index_templates[i] = 0;
                    index_scales[i] = 0;
                    index_rotations[i] = 0;
               }


               t = ((double) getTickCount() - t) / getTickFrequency();
               std::cout << "PCA projection computation: time " << t << std::endl;

               ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
               // Find the location of objects

               vector<Mat> RMs(num_models_);

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
               waitKey(3);

          }
     }


     void LARKs::Pre_Search(const float factor, const int numTemplate, const int numRotation, const int numScale, array_type3& query_mask, array_type4& QF, double_type4& QF_norm, Mat target, bool binaryfeaturemode, array_type3& TF, Mat scales, Mat block_table, Point_type3& region_index, double Prev_max, int maxComponents, double_type3& maxVal, Mat& labels, int level, int obj, vector<int_type1> &offset_x, vector<int_type1> &offset_y)
     {

          double	t = (double) getTickCount();
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

          t = ((double) getTickCount() - t) / getTickFrequency();
          cout << "[Pre detection search time " << t << " sec]" << endl;

     }




     void LARKs::Search(const float factor, const int numTemplate, const int numRotation, const int numScale, array_type3& query_mask, array_type4& QF, double_type4& QF_norm, Mat target, bool binaryfeaturemode, array_type3& TF, Mat scales, Mat block_table, Point_type3& region_index, double Prev_max, int maxComponents, Mat& img2, bool& use_saliency, Mat img1,  Mat& RM1, std::string modelname, const float threshold , double_type3& maxVals, int& index_template, int& index_scale, int& index_rotation, int level, int obj, Detection &d)
     {

          double	t = (double) getTickCount();
          array_type3 RM(boost::extents[numTemplate][numRotation][numScale]);


          int index_template1 = 0, index_scale1 = 0, index_rotation1 = 0;

          for (int lev = 1; lev < level; lev++)
               for (int i = 0; i < numTemplate; i++)
               {
                    offset_x[obj][i] = query_mask[lev][i][0].rows / 2;

                    offset_y[obj][i] = query_mask[lev][i][0].cols / 2;


                    Multiscale_search_withPrevious(QF, QF_norm, query_mask, target,  RM,
                                                   offset_x[obj][i], offset_y[obj][i], i, binaryfeaturemode, TF, scales, block_table, region_index, maxComponents, maxVals, threshold, lev, obj);


               }

          t = ((double) getTickCount() - t) / getTickFrequency();
          cout << "[Multi-scale search time " << t << " sec]" << endl;


          t = (double) getTickCount();
          Mat rotationIndex = Mat::zeros(RM[0][0][0].rows,
                                         RM[0][0][0].cols, CV_8U);

          Mat scaleIndex = Mat::zeros(RM[0][0][0].rows, RM[0][0][0].cols,
                                      CV_8U);
          Mat templateIndex = Mat::zeros(RM[0][0][0].rows,
                                         RM[0][0][0].cols, CV_8U);
          Mat RM_final = Mat::zeros(RM[0][0][0].rows, RM[0][0][0].cols,
                                    CV_32F);

          float maxval = 0.0;
          for (int rot = 0; rot < numRotation; rot++)
               for (int j = 0; j < numScale; j++)
                    for (int i = 0; i < numTemplate; i++)
                    {
                         double minVal, maxVal;
                         Point minLoc, maxLoc;
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

          t = ((double) getTickCount() - t) / getTickFrequency();
          //cout << " Index decision time " << t << endl;

          t = (double) getTickCount();
          minMaxLoc(RM_final, &minVal1, &maxVal1, &minLoc1, &maxLoc1,
                    Mat());

          Mat RM2;


          for (int i = 0; i < numTemplate;  i++) {
               offset_x[obj][i] = query_mask[level-1][i][0].rows / (2);
               offset_y[obj][i] = query_mask[level-1][i][0].cols / (2);
          }

          //cout << "min " << minVal1 << " max " << maxVal1 << endl;

          vector<Point2f> detectionpt(4), detectionpt1(4);

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

          Mat RM_final1 = Mat::zeros(RM[0][0][0].rows, RM[0][0][0].cols,	CV_32F);

          for (int rot = 0; rot < numRotation; rot++)
               for (int j = 0; j < numScale; j++)
                    for (int i = 0; i < numTemplate; i++)
                    {
                         double minVal, maxVal;
                         Point minLoc, maxLoc;

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
                              index_scale1 = j;
                              index_rotation1 = rot;
                              RM_final1.at<float>(maxLoc.y, maxLoc.x) = maxval;

                         }
                    }



          minMaxLoc(RM_final1, &minVal2, &maxVal2, &minLoc2, &maxLoc2,Mat());

          add(RM_final,RM_final1,RM_final);

          resize(RM_final, RM1, Size(), 1/ (factor), 1 / (factor),
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

               d.mask.roi.x = detectionpt[0].x;
               d.mask.roi.y = detectionpt[0].y;
               d.mask.roi.width = detectionpt[3].x - detectionpt[0].x + 1 ;
               d.mask.roi.height = detectionpt[3].y - detectionpt[0].y + 1;


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
               detections_.detections.push_back(d);
               //d.mask.mask = mask;

          }
     }

     void LARKs::Multiscale_search(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const Mat& target, array_type3& RM, const int offset_x, const int offset_y,
                                   const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const Mat& scales,	const Mat& block_table, Point_type3& region_index,  const int maxComponents, float f, double_type3& maxVals, int level, int obj)
     {

          //int skip[] = { 5,5, 5, 4 }; //5,5,5,4
          //int apart[] = { 4,4, 4, 4 };

          int skip[] = { 2,2, 2, 2 }; //5,5,5,4
          int apart[] = { 3,3, 3, 3 };



          double t = (double) getTickCount();

          for (int rot = 0; rot < numRotation; rot++)
          {

               int width = query_mask[level][query_index][rot].cols;
               int height = query_mask[level][query_index][rot].rows;

               int half_width = query_mask[level][query_index][rot].cols / 2;
               int half_height = query_mask[level][query_index][rot].rows / 2;

               for (int scale = 0; scale < numScale; scale++)
               {

                    Mat RM1 = Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);
                    Mat RM2 = Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);


                    RM[query_index][rot][scale] = Mat::zeros(TF[level][0][0].rows, TF[level][0][0].cols, CV_32F);

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
                    Point minLoc, maxLoc;
                    minMaxLoc(RM[query_index][rot][scale], &minVal, &maxVals[query_index][scale][rot], &minLoc, &maxLoc,Mat());


               }

          }

          t = ((double) getTickCount() - t) / getTickFrequency();
          //cout << "search" << t << endl;

     }

     void LARKs::Multiscale_search_withPrevious(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const Mat& target,array_type3& RM, const int offset_x, const int offset_y,const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const Mat& scales,	const Mat& block_table,	Point_type3&  region_index, const int maxComponents,double_type3& maxVals, float threshold, int level, int obj)
     {

          int skip[] = { 5, 5, 5, 4 };
          int apart[] = { 4, 4, 4, 4 };

          //int skip[] = { 2,2, 2, 2 }; //5,5,5,4
          //int apart[] = { 3,3, 3, 3 };


          double t = (double) getTickCount();

          for (int rot = 0; rot < numRotation; rot++)
          {

               int width = query_mask[level][query_index][rot].cols;
               int height = query_mask[level][query_index][rot].rows;

               int half_width = query_mask[level][query_index][rot].cols / 2;
               int half_height = query_mask[level][query_index][rot].rows / 2;

               for (int scale = 0; scale < numScale; scale++)
               {
                    float maxvalue = 0.0;
                    Mat RM1 = Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);
                    Mat RM2 = Mat::zeros(TF[level][scale][0].rows, TF[level][scale][0].cols, CV_32F);

                    RM[query_index][rot][scale] = Mat::zeros(TF[level][0][0].rows, TF[level][0][0].cols,	CV_32F);

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

          t = ((double) getTickCount() - t) / getTickFrequency();
//	cout << "search" << t << endl;
     }



     void LARKs::getTF(const int wsize, Mat target, Mat sC11, Mat sC12, Mat sC22, bool binaryfeaturemode, const int maxComponents, const int numScale, array_type2& TF, Mat& means, Mat& eigenvectors)
     {
          array_type1 target_LARK(boost::extents[wsize * wsize]);
          LARK.computeLARK(target.rows, target.cols, wsize, sC11, sC12, sC22,
                           target_LARK);

          Mat target_featureset;
          target_featureset.create(target.cols * target.rows, wsize
                                   * wsize, CV_32F);
          for (int i = 0; i < target.rows; i++)
               for (int j = 0; j < target.cols; j++)
               {
                    for (int cnt = 0; cnt < wsize * wsize; cnt++)
                    {
                         target_featureset.at<float> (i * target.cols + j,
                                                      cnt) = target_LARK[cnt].at<float> (i, j);
                    }

               }
          Mat targetfeature;
          targetfeature.create(target_featureset.rows, maxComponents,
                               target_featureset.type());
          for (int j = 0; j < target_featureset.rows; j++)
          {
               Mat vec = target_featureset.row(j);
               Mat coeffs = targetfeature.row(j);
               //	pca.project(vec, coeffs);
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
                    resize(TF[0][cnt], TF[i][cnt], Size(),
                           scales.at<float> (0, i),
                           scales.at<float> (0, i), INTER_LANCZOS4);
               }

          }

     }

     void LARKs::GradientImage(const Mat& gray, Mat& GradImage)
     {
          int rsize = 2;
          Mat_<unsigned char> gray1(gray.rows + rsize * 2, gray.cols + rsize * 2);
          copyMakeBorder(gray, gray1, rsize, rsize, rsize, rsize,
                         BORDER_REPLICATE);

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

          Mat_<unsigned char> GXimg(gray1.rows, gray1.cols);
          Mat_<unsigned char> GYimg(gray1.rows, gray1.cols);

          Mat_<float> GX(5, 5);
          Mat_<float> GY(5, 5);

          for (int i = 0; i < 5; i++)
               for (int j = 0; j < 5; j++) {
                    GX(i, j) = X[i * 5 + j];
                    GY(i, j) = Y[i * 5 + j];
               }

          filter2D(gray1, GXimg, GradImage.depth(), GX);
          filter2D(gray1, GYimg, GradImage.depth(), GY);

          add(GXimg, GYimg, GradImage);
          GradImage /= 2;

     }

     void LARKs::pyramidFeatures(array_type2 TF, array_type3& TFs, array_type3 QF, array_type4& QFs,double_type3 QF_norm, double_type4& QF_norms, array_type2 query_mask, array_type3& query_masks, int level, const int numTemplate)
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
                              resize(TF[scale][cnt], TFs[le][scale][cnt], Size(),f[le], f[le], INTER_LANCZOS4);
                         else
                              TF[scale][cnt].copyTo(TFs[le][scale][cnt]);

                    }
               for (int tem = 0; tem < numTemplate; tem++)
                    for (int rot = 0; rot < numRotation; rot++)
                    {
                         if (le != level-1)
                              resize(query_mask[tem][rot], query_masks[le][tem][rot], Size(), f[le], f[le], INTER_LANCZOS4);
                         else
                              query_mask[tem][rot].copyTo(query_masks[le][tem][rot]);

                         for (int cnt = 0; cnt < maxComponents; cnt++)
                         {
                              if (le != level-1)
                                   resize(QF[tem][rot][cnt], QFs[le][tem][rot][cnt], Size(), f[le], f[le], INTER_LANCZOS4);
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

}
