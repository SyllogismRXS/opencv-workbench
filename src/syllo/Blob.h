#ifndef BLOB_H_
#define BLOB_H_
/// ---------------------------------------------------------------------------
/// @file Blob.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-06-17 12:23:27 syllogismrxs>
///
/// @version 1.0
/// Created: 17 Jun 2015
///
/// ---------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ---------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The Blob class ...
/// 
/// ---------------------------------------------------------------------------
#include <iostream>
#include <sstream>

#include <cv.h>
#include <highgui.h>

#include <Eigen/Dense>

//#include "Sonar.h"
//#include "filters.h"
#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/track/ParticleFilter.h>
#include <opencv_workbench/track/MotionModels.h>

namespace syllo {

class Blob {
     protected:
	  Eigen::MatrixXf A;
	  Eigen::MatrixXf B;
	  Eigen::VectorXf C;
	  Eigen::MatrixXf R;
	  Eigen::MatrixXf Q;
	  Eigen::VectorXf mu;
	  Eigen::MatrixXf covar;
	  Eigen::VectorXf u;

	  Eigen::VectorXf estVel;
	  bool velDet;
	  int detectCount;

	  cv::Point lastDetCentroid;
	  int lastDetTime;
	  cv::Point lastEstCentroid;

     public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	  int id;
	  int size;
	  cv::Point centroid;
	  cv::Point lastCentroid;
	  int age;
	  int clusterID;

	  syllo::KalmanFilter kalmanTracker;

     Blob(int id) : id(id)
	  {
	       clusterID = -1;
	       resetAge();
	       size = 0;
	       
	       initModel();
	  }
	  
	  Blob() 
	  {
	       clusterID = -1;
	       id = -1;
	       size = -1;
	       centroid = cv::Point(-1,-1);
	       age = -1;

	       initModel();
	  }

	  int initModel()
	  {
	       A.resize(2,2);
	       B.resize(2,2);
	       C.resize(2);
	       R.resize(2,2);
	       Q.resize(2,2);
	       mu.resize(2);
	       covar.resize(2,2);
	       u.resize(2);

	       estVel.resize(2);

	       A << 0,0,0,0;
	       B << 1,0,0,1;
	       C << 1,1;
	       R << 0.1,0.1,0.1,0.1;
	       Q << 0.1,0.1,0.1,0.1;
	       
	       mu << 0,1;
	       covar << 0.1,0.1,0.1,0.1;
	       u << 1,1;

	       estVel << 0,0;
	       velDet = false;
	       detectCount = 0;

	       kalmanTracker.setModel(A,B,C,R,Q);

               return 0;
	  }

	  int step()
	  {
	       if (velDet) {
		    //kalmanTracker.step(mu, covar, estVel);
		    centroid = cv::Point((int)mu(0), (int)mu(1));
	       }

	       age--;
               return 0;
	  }

	  int velocityUpdate()
	  {
	       //estVel(0)
               return -1;
	  }

	  int update(Blob newBlob, int curTime)
	  {
	       //detectCount++;
	       //
	       //if (detectCount > 1) {
	       //	    //printf("Velocity: x:%f y:%f\n",estVel(0),estVel(1));
	       //	    estVel(0) = (newBlob.getCentroid().x - lastDetCentroid.x) / (curTime - lastDetTime);
	       //	    estVel(1) = (newBlob.getCentroid().y - lastDetCentroid.y) / (curTime - lastDetTime);
	       //	    velDet = true;
	       //}
	       lastDetTime = curTime;

	       size = newBlob.getSize();
	       centroid = newBlob.getCentroid();
	       //mu << centroid.x , centroid.y;
	       lastDetCentroid = centroid;
	       age++;
	       
	       return 0;
	  }

	  int getID()
	  {
	       return id;
	  }

	  void setID(int id)
	  {
	       this->id = id;
	  }

	  void setCentroid(cv::Point centroid)
	  {
	       this->lastCentroid = this->centroid;
	       this->centroid = centroid;
	  }

	  cv::Point getCentroid()
	  {
	       return centroid;
	  }

	  void incSize()
	  {
	       size++;
	  }
	  
	  void resetAge()
	  {
	       age = 3;
	  }

	  void incAge()
	  {
	       age++;
	  }

	  void decAge()
	  {
	       age--;
	  }

	  int getAge()
	  {
	       return age;
	  }

	  int getSize()
	  {
	       return size;
	  }

	  void setSize(int size)
	  {
	       this->size = size;               
	  }
          
     };
}

#endif
