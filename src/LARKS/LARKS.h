#ifndef _LARKS_H_
#define _LARKS_H_

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <boost/multi_array.hpp>
#include <vector>
#include <string>
#include <map>

namespace larks 
{
     using boost::multi_array;
     typedef multi_array<cv::Mat, 4> array_type4;
     typedef multi_array<cv::Mat, 3> array_type3;
     typedef multi_array<cv::Mat, 2> array_type2;
     typedef multi_array<cv::Mat, 1> array_type1;
     typedef multi_array<double, 3> double_type3;
     typedef multi_array<double, 4> double_type4;
     typedef multi_array<cv::Point, 3> Point_type3;
     typedef multi_array<cv::Point, 4> Point_type4;
     typedef multi_array<int,1> int_type1;

     

     class LARKFeatureTemplates
     {
     public:

          cv::PCA pca;
          array_type1 QF;
          cv::Mat M;
          int cols;
          cv::Mat img;
          cv::Mat mas;

          LARKFeatureTemplates() { };
          
          void computeFeatures(const cv::Mat &whole_gray, const cv::Rect & roi, 
                               const cv::Mat & mask,  int index, cv::PCA& pca1, 
                               int cols1);

          float MatrixCosineMeasure(array_type1& QF1, cv::Mat M1, 
                                    array_type1& QF, cv::Mat& M);
     };

     class Larkcomputation
     {

     public:
          Larkcomputation(){};
          void computeCovariance(const cv::Mat& gray, int wsize, 
                                 const int dfactor, cv::Mat& sC11, 
                                 cv::Mat& sC12, cv::Mat& sC22);
          
          void computeLARK(const int rows, const int cols, const int wsize, 
                           cv::Mat& sC11, cv::Mat& sC12, cv::Mat& sC22, 
                           array_type1& temp);
     };

     class TrainingTemplate
     {
     public:
          TrainingTemplate(){};
          void Training(std::vector<LARKFeatureTemplates>& models, array_type2& query_mask, array_type3& QF, double_type3& QF_norm, cv::Mat& labels);
     
          inline cv::PCA get_pca() {return pca;}
     
          int numTemplate;
          int numScale;
          int numRotation;
          int maxComponents;
     
     protected:
          cv::PCA pca;
          Larkcomputation LARK;
          std::vector<LARKFeatureTemplates> models;     
     };


     class Saliency
     {
     public:
          Saliency(){};
          void ProtoObject(const cv::Mat &SaliencyMap, cv::Mat& thMap);
          void computeSaliency(const array_type1& LARK, const int wsize, 
                               const int psize, cv::Mat& SaliencyMap);
          
     };   
     
     struct Detection
     {
          //Object object;
          cv::Rect bounding_box;
          std::string detector;
          float score;
          std::string label;
          cv::Mat mask;
     };
     
     class LARKS// : public Detector, public Trainable
     {

     public:

          LARKS()
          {
               factor = 0.25;
               dfactor = 4;
               blue = cv::Scalar(255, 0,0);
               dark_blue = cv::Scalar(125,0,0);
               red = cv::Scalar(0,0,255);
               dark_red = cv::Scalar(0,0,125);
               green = cv::Scalar(0,255,0);
               dark_green = cv::Scalar(0,125,0);
               purple = cv::Scalar(255,0,255);
               dark_purple = cv::Scalar(125,0,125);
               black = cv::Scalar(0,0,0);
               light_black = cv::Scalar(125,125,125);
     
               yellow = cv::Scalar(0,255,255);
               white = cv::Scalar(255,255,255);
               use_saliency = true;
               x_block = 10;
               y_block = 10;
               wsize = 5;
               AtleastSalient = 0;
               binaryfeaturemode = false;
               block_table = cv::Mat::zeros(x_block, y_block, CV_8U);
     
               index = 0;     
          }

          void startTraining();          
          
          void trainInstance(const std::string& name, cv::Mat &img);
          void endTraining(const std::string& name);
          //void loadModels(const std::vector<std::string>& models);

          void save_model(const std::string& name, 
                                 const std::string& detector, 
                                 const std::string& model_blob);

          void loadModels();

          void detect(cv::Mat &image);

          void Pre_Search(const float factor, const int numTemplate, 
                          const int numRotation, const int numScale, 
                          array_type3& query_mask, array_type4& QF, 
                          double_type4& QF_norm, cv::Mat target, 
                          bool binaryfeaturemode, array_type3& TF, cv::Mat scales, 
                          cv::Mat block_table, Point_type3& region_index, 
                          double Prev_max, int maxComponents, 
                          double_type3& maxVals, cv::Mat& labels, int level, 
                          int obj, std::vector<int_type1> & offset_x, std::vector<int_type1> & offset_y);

          void Search(const float factor, const int numTemplate, const int numRotation, const int numScale, array_type3& query_mask, array_type4& QF, double_type4& QF_norm, cv::Mat target, bool binaryfeaturemode, array_type3& TF, cv::Mat scales, cv::Mat block_table, Point_type3& region_index, double Prev_max, int maxComponents, cv::Mat& img2, bool& use_saliency, cv::Mat img1, cv::Mat& RM1, std::string modelname, const float threshold , double_type3& maxVals, int& index_template, int& index_scale, int& index_rotation, int level, int obj, Detection &d);

          void	getTF(const int wsize, cv::Mat target, cv::Mat sC11, cv::Mat sC12, cv::Mat sC22, bool binaryfeaturemode, const int maxComponents, const int numScale, array_type2& TF, cv::Mat& means, cv::Mat& eigenvectors);

          void Multiscale_search(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const cv::Mat& target, array_type3& RM, const int offset_x, const int offset_y,
                                 const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const cv::Mat& scales,	const cv::Mat& block_table, Point_type3& region_index,  const int maxComponents, float f, double_type3& maxVal, int level, int obj);

          void Multiscale_search_withPrevious(array_type4& QF, double_type4& QF_norm,array_type3& query_mask, const cv::Mat& target,	array_type3& RM, const int offset_x, const int offset_y,	const int query_index, const bool binaryfeaturemode,const array_type3& TF,  const cv::Mat& scales,	const cv::Mat& block_table,Point_type3&  region_index, const int maxComponents,double_type3& maxVal1, float threshold, int level, int obj) ;

          void pyramidFeatures(array_type2 TF, array_type3& TFs, array_type3 QF, array_type4& QFs,double_type3 QF_norm, double_type4& QF_norms, array_type2 query_mask, array_type3& query_masks, int level, const int numTemplate);


          void GradientImage(const cv::Mat& gray, cv::Mat& GradImage);

          TrainingTemplate TT;

          std::vector<std::string> model_list;
	  std::vector<LARKFeatureTemplates> models;
          //typedef map<std::string, vector<LARKFeatureTemplates> > ModelsMap;

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
          inline void set_models(std::vector<std::string> models, int num_models) { models_.resize(num_models); models_ = models;	}
          inline void set_threshold(std::vector<double> threshold, int num_models) { threshold_.resize(num_models); threshold_ = threshold;	}


          int id;

          cv::Mat eigenvector;
          cv::Mat mean;
          std::vector<cv::Mat> eigenvectors, means;
          int numTemplate, maxComponents, numScale, coeff, numRotation;
          std::vector<int> numTemplates;

          cv::Mat scales;
          std::vector<cv::Mat> labels;
          cv::Mat label;
          std::vector<int_type1> offset_x;
          std::vector<int_type1> offset_y;

          std::string objname;
          std::string name;
          float score;
          cv::Mat mask;
          cv::Rect roi;
          int num_models_;


          std::vector<float> Prev_max;

          std::vector<std::string> models_;
          std::vector<double> threshold_;

          Larkcomputation LARK;

          bool use_saliency, binaryfeaturemode;

          int wsize, query_counter;

          float factor;

          double minVal1, minVal2;
          double maxVal1, maxVal2;
          cv::Point minLoc1, minLoc2;
          cv::Point maxLoc1, maxLoc2;

          int x_block;
          int y_block;
          cv::Mat block_table;

          std::vector<cv::Point> start, end;
          cv::Point Prev_Loc;


          int dfactor;
          int AtleastSalient;

          Saliency SalientRegion;

          cv::Scalar blue;
          cv::Scalar dark_blue;
          cv::Scalar red;
          cv::Scalar dark_red;
          cv::Scalar green;
          cv::Scalar dark_green;
          cv::Scalar purple;
          cv::Scalar dark_purple;
          cv::Scalar yellow;
          cv::Scalar white;
          cv::Scalar black;
          cv::Scalar light_black;

          std::vector<Point_type3> region_index;
          const char* modelname_;

          array_type3 QF, QF1;
          std::vector<array_type3> QFs;
          array_type2 query_mask, query_mask1;
          std::vector<array_type2> query_masks;
          double_type3 QF_norm, QF_norm1;
          std::vector<double_type3> QF_norms;

          std::vector<array_type4> Py_QFs;
          std::vector<array_type3> Py_query_masks;
          std::vector<double_type4> Py_QF_norms;

          cv::PCA pca;
          std::vector<cv::PCA> pcas;
          int index;

          double_type3 maxVal;
          std::vector<double_type3> maxVals;

          std::vector<int> index_templates, index_scales, index_rotations;
     };

}
#endif
