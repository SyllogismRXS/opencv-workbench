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
#ifndef LARKS_H_
#define LARKS_H_

#include <iostream>
#include <boost/multi_array.hpp>
//#include <opencv_workbench/recognition_pipeline/algorithms/detector.h>
//#include <opencv_workbench/recognition_pipeline/algorithms/trainable.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp> 
//#include <opencv_workbench/recognition_pipeline/algorithms/serialization_support.h>

namespace larks {

//using namespace recognition_pipeline;
     using namespace cv;
     using namespace std;

     using boost::multi_array;
     typedef multi_array<Mat, 4> array_type4;
     typedef multi_array<Mat, 3> array_type3;
     typedef multi_array<Mat, 2> array_type2;
     typedef multi_array<Mat, 1> array_type1;
     typedef multi_array<double, 3> double_type3;
     typedef multi_array<double, 4> double_type4;
     typedef multi_array<Point, 3> Point_type3;
     typedef multi_array<Point, 4> Point_type4;
     typedef multi_array<int,1> int_type1;



     class Larkcomputation
     {

     public:
          Larkcomputation(){};
          void computeCovariance(const Mat& gray, int wsize, const int dfactor, Mat& sC11, Mat& sC12, Mat& sC22);
          void computeLARK(const int rows, const int cols, const int wsize, Mat& sC11, Mat& sC12, Mat& sC22, array_type1& temp);

     };
     
     class Saliency
     {
     public:
          Saliency(){};
          void ProtoObject(const Mat &SaliencyMap, Mat& thMap);
          void computeSaliency(const array_type1& LARK, const int wsize, const int psize, Mat& SaliencyMap);

     };
 

     class LARKFeatureTemplates
     {
     public:

          PCA pca;
          array_type1 QF;
          cv::Mat M;
          int cols;
          cv::Mat img;
          cv::Mat mas;



          LARKFeatureTemplates() { };

          void computeFeatures(const cv::Mat &whole_gray, const cv::Rect & roi, const cv::Mat & mask,  int index, PCA& pca1, int cols1);


          float MatrixCosineMeasure(array_type1& QF1, Mat M1, array_type1& QF, Mat& M);


     };

     class TrainingTemplate
     {
     public:
          TrainingTemplate(){};
          void Training(std::vector<LARKFeatureTemplates>& models, array_type2& query_mask, array_type3& QF, double_type3& QF_norm, Mat& labels);

          inline PCA get_pca() {return pca;}

          int numTemplate;
          int numScale;
          int numRotation;
          int maxComponents;

     protected:
          PCA pca;
          Larkcomputation LARK;
          std::vector<LARKFeatureTemplates> models;

     };



     class LARKs //: public Detector, public Trainable
     {


     public:

     LARKs(ModelStoragePtr model_storage) : Detector(model_storage)
          {
               std::cout << "test" << std::endl;

               factor = 0.25;
               dfactor = 4;
               blue = Scalar(255, 0,0);
               dark_blue = Scalar(125,0,0);
               red = Scalar(0,0,255);
               dark_red = Scalar(0,0,125);
               green = Scalar(0,255,0);
               dark_green = Scalar(0,125,0);
               purple = Scalar(255,0,255);
               dark_purple = Scalar(125,0,125);
               black = Scalar(0,0,0);
               light_black = Scalar(125,125,125);

               yellow = Scalar(0,255,255);
               white = Scalar(255,255,255);
               use_saliency = true;
               x_block = 10;
               y_block = 10;
               wsize = 5;
               AtleastSalient = 0;
               binaryfeaturemode = false;
               block_table = Mat::zeros(x_block, y_block, CV_8U);

               index = 0;

          };

          ~LARKs()
          {

          };

          /**
           * \brief Run the object detector. The detection results are stored in
           * class member detections_.
           */
          virtual void detect();

          /**
           * Each detector will have an unique name. This returns the detector name.
           * @return name of the detector
           */
          virtual std::string getName() { return "larks"; }

          /**
           * Loads pre-trained models for a list of objects.
           * @param models list of objects to load
           */
          virtual void loadModels(const std::vector<std::string>& models);

          /**
           * Starts training for a new object category model. It may allocate/initialize
           * data structures needed for training a new category.
           * @param name
           */
          virtual void startTraining(const std::string& name);

          /**
           * Trains the model on a new data instance.
           * @param name The name of the model
           * @param data Training data instance
           */
          virtual void trainInstance(const std::string& name, const recognition_pipeline::TrainingData& data);

          /**
           * Saves a trained model.
           * @param name model name
           */
          virtual void endTraining(const std::string& name);

          void Pre_Search(const float factor, const int numTemplate, const int numRotation, const int numScale, array_type3& query_mask, array_type4& QF, double_type4& QF_norm, Mat target, bool binaryfeaturemode, array_type3& TF, Mat scales, Mat block_table, Point_type3& region_index, double Prev_max, int maxComponents, double_type3& maxVals, Mat& labels, int level, int obj, vector<int_type1> & offset_x, vector<int_type1> & offset_y);

          void Search(const float factor, const int numTemplate, const int numRotation, const int numScale, array_type3& query_mask, array_type4& QF, double_type4& QF_norm, Mat target, bool binaryfeaturemode, array_type3& TF, Mat scales, Mat block_table, Point_type3& region_index, double Prev_max, int maxComponents, Mat& img2, bool& use_saliency, Mat img1, Mat& RM1, std::string modelname, const float threshold , double_type3& maxVals, int& index_template, int& index_scale, int& index_rotation, int level, int obj, Detection &d);

          void	getTF(const int wsize, Mat target, Mat sC11, Mat sC12, Mat sC22, bool binaryfeaturemode, const int maxComponents, const int numScale, array_type2& TF, Mat& means, Mat& eigenvectors);

          void Multiscale_search(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const Mat& target, array_type3& RM, const int offset_x, const int offset_y,
                                 const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const Mat& scales,	const Mat& block_table, Point_type3& region_index,  const int maxComponents, float f, double_type3& maxVal, int level, int obj);

          void Multiscale_search_withPrevious(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const Mat& target,	array_type3& RM, const int offset_x, const int offset_y,	const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const Mat& scales,	const Mat& block_table,Point_type3&  region_index, const int maxComponents,double_type3& maxVal1, float threshold, int level, int obj) ;

          void pyramidFeatures(array_type2 TF, array_type3& TFs, array_type3 QF, array_type4& QFs,double_type3 QF_norm, double_type4& QF_norms, array_type2 query_mask, array_type3& query_masks, int level, const int numTemplate);

          void GradientImage(const Mat& gray, Mat& GradImage);



          TrainingTemplate TT;

	  vector<string> model_list;
	  std::vector<LARKFeatureTemplates> models;
          //typedef map<string, vector<LARKFeatureTemplates> > ModelsMap;



	  template<class archive>
               void load(archive& ar, const unsigned int version)
	  {

               ar & numTemplates[id];
               ar & numScale;
               ar & numRotation;
               ar & maxComponents;
               ar & eigenvectors[id];
               ar & means[id];

               std::cout << "numTemplate " << numTemplates[id] << std::endl;

               QFs[id].resize(boost::extents[numTemplates[id]][numRotation][maxComponents]);
	       query_masks[id].resize(boost::extents[numTemplates[id]][numRotation]);
	       QF_norms[id].resize(boost::extents[numTemplates[id]][numRotation][numScale]);

               for ( int i = 0; i < numTemplates[id]; ++i)
                    for ( int j = 0; j < numRotation; ++j)
                         for ( int m = 0; m < maxComponents; ++m)
                         {
                              ar & QFs[id][i][j][m];

                         }
               for ( int i = 0; i < numTemplates[id]; ++i)
                    for ( int j = 0; j < numRotation; ++j)
                         for ( int m = 0; m < numScale; ++m)
                         {
                              ar & QF_norms[id][i][j][m];
                         }
               for ( int i = 0; i < numTemplates[id]; ++i)
                    for ( int j = 0; j < numRotation; ++j)
                    {
                         ar & query_masks[id][i][j];

                    }
               ar & labels[id];
          }

          template<class archive>
               void save(archive& ar, const unsigned int version) const
          {
	       ar & numTemplate;
	       ar & numScale;
	       ar & numRotation;
	       ar & maxComponents;
	       ar & eigenvector;
	       ar & mean;
	       for (unsigned int i = 0; i < QF.shape()[0]; ++i)
                    for (unsigned int j = 0; j < QF.shape()[1]; ++j)
                         for (unsigned int m = 0; m < QF.shape()[2]; ++m)
                         {
                              ar & QF[i][j][m];
                         }
               for (unsigned int i = 0; i < QF_norm.shape()[0]; ++i)
                    for (unsigned int j = 0; j < QF_norm.shape()[1]; ++j)
                         for (unsigned int m = 0; m < QF_norm.shape()[2]; ++m)
                         {
                              ar & QF_norm[i][j][m];


                         }
               for (unsigned int i = 0; i < query_mask.shape()[0]; ++i)
                    for (unsigned int j = 0; j < query_mask.shape()[1]; ++j)
                    {
                         ar & query_mask[i][j];
                    }

               ar & label;

          }

          // define serialize() using save() and load()
          BOOST_SERIALIZATION_SPLIT_MEMBER();

          inline void set_num_models(int num_models) { num_models_ = num_models;	}
          inline void set_models(vector<std::string> models, int num_models) { models_.resize(num_models); models_ = models;	}
          inline void set_threshold(vector<double> threshold, int num_models) { threshold_.resize(num_models); threshold_ = threshold;	}


          int id;

          Mat eigenvector;
          Mat mean;
          vector<cv::Mat> eigenvectors, means;
          int numTemplate, maxComponents, numScale, coeff, numRotation;
          vector<int> numTemplates;

          Mat scales;
          vector<cv::Mat> labels;
          Mat label;
          vector<int_type1> offset_x;
          vector<int_type1> offset_y;

          std::string objname;
          std::string name;
          float score;
          cv::Mat mask;
          cv::Rect roi;
          int num_models_;


          vector<float> Prev_max;

          vector<std::string> models_;
          vector<double> threshold_;



          Larkcomputation LARK;



          bool use_saliency, binaryfeaturemode;

          int wsize, query_counter;


          float factor;


          double minVal1, minVal2;
          double maxVal1, maxVal2;
          Point minLoc1, minLoc2;
          Point maxLoc1, maxLoc2;


          int x_block;
          int y_block;
          Mat block_table;

          vector<Point> start, end;
          Point Prev_Loc;


          int dfactor;
          int AtleastSalient;

          Saliency SalientRegion;

          Scalar blue;
          Scalar dark_blue;
          Scalar red;
          Scalar dark_red;
          Scalar green;
          Scalar dark_green;
          Scalar purple;
          Scalar dark_purple;
          Scalar yellow;
          Scalar white;
          Scalar black;
          Scalar light_black;

          vector<Point_type3> region_index;
          const char* modelname_;

          array_type3 QF, QF1;
          vector<array_type3> QFs;
          array_type2 query_mask, query_mask1;
          vector<array_type2> query_masks;
          double_type3 QF_norm, QF_norm1;
          vector<double_type3> QF_norms;

          vector<array_type4> Py_QFs;
          vector<array_type3> Py_query_masks;
          vector<double_type4> Py_QF_norms;

          cv::PCA pca;
          vector<cv::PCA> pcas;
          int index;

          double_type3 maxVal;
          vector<double_type3> maxVals;

          vector<int> index_templates, index_scales, index_rotations;




     };

#endif


}
