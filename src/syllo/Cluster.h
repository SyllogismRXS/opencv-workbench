#ifndef CLUSTER_H_
#define CLUSTER_H_
/// ---------------------------------------------------------------------------
/// @file Cluster.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-07-02 17:17:32 syllogismrxs>
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
/// The Cluster class ...
/// 
/// ---------------------------------------------------------------------------

namespace syllo {
     typedef enum {
          clutter = 0,
          diver = 1
     }EntityType;

     class Cluster {
     protected:
	  std::map<int,syllo::Blob> blobs;
	  cv::Point centroid;
	  int age;
	  int maxAge;

          EntityType type_;

	  cv::Point firstCentroid;
	  int firstTime;

	  cv::Point2d velocity;
	  //cv::Vec2d variance;
	  double variance;
	  double velMag;
	  double varMag;
	  int count;
	  double sumX;
	  double sumY;
	  
	  double angle;
	  double angleVariance;

	  std::vector<cv::Point> prevCentroids_;

	  ParticleFilter pf_;

     public:

	  Cluster()
	  {
	       maxAge = 5;
	       resetAge();
	       centroid = cv::Point(-1,-1);
	       velocity = cv::Point2d(0,0);
	       //variance = cv::Vec2d(0,0);
	       variance = 1;
	       velMag = 0;
	       varMag = 0;
	       count = 0;
	       sumX = 0;
	       sumY = 0;
	       angle = 0;
	       angleVariance = 0;
               type_ = clutter;
	  }

          void set_type(EntityType type)
          {
               type_ = type;
          }

          EntityType type()
          {
               return type_;
          }

	  void particle_filter_init(int num_particles,
	      double x_min, double x_max, 
	      double y_min, double y_max,
	      double heading_min, double heading_max,
	      double velocity_min, double velocity_max)
	  {
	       pf_.init(num_particles,x_min,x_max,y_min,y_max,
		       heading_min,heading_max,velocity_min,velocity_max);
	  }

	  void pf_step()
	  {
	       pf_.kinematic_step();
	       pf_.importance_weight_update(centroid);
	       pf_.resample_wheel();
	  }

	  void getParticleFilter(ParticleFilter &pf)
	  {
	       pf = pf_;
	  }

	  std::vector<cv::Point> prevCentroids()
	  {
	       return prevCentroids_;
	  }

	  void addCentroid(cv::Point newCentroid)
	  {
	       prevCentroids_.push_back(newCentroid);
	  }

	  void declareFirstCentroid(int time)
	  {
	       this->firstCentroid = this->centroid;
	       this->firstTime = time;
	       
	       addCentroid(this->firstCentroid);

	  }

	  int getDistTravel()
	  {
	       return sqrt( pow(centroid.x-firstCentroid.x,2) + pow(centroid.y-firstCentroid.y,2) );
	  }

	  void setVelocity(cv::Point2d vel)
	  {
	       this->velocity = vel;
	       sumX = 0;
	       sumY = 0;
	       count = 0;
	       calcMags();
	  }

	  cv::Point2d getVelocity()
	  {
	       return velocity;
	  }

	  void calcMags()
	  {
	       velMag = sqrt( pow(velocity.x,2) + pow(velocity.y,2));
	       //varMag = sqrt( pow(variance(0),2) + pow(variance(1),2));
	  }

	  double CrossProduct(const cv::Vec2d &v1, const cv::Vec2d & v2)
	  {
	       return (v1(0)*v2(1)) - (v1(1)*v2(0));
	  }

	  void calcVelocity(cv::Point oldPoint, cv::Point newPoint)
	  {
	       velocity.x = newPoint.x - oldPoint.x;
	       velocity.y = newPoint.y - oldPoint.y;

	       //// BEFORE PARTICLE FILTER...
	       //cv::Vec2d newVel;
	       //newVel(0) = newPoint.x - oldPoint.x;
	       //newVel(1) = newPoint.y - oldPoint.y;
	       //
	       //double alpha = 0.01;
	       //velocity(0) = alpha*(newPoint.x - oldPoint.x) + (1-alpha)*velocity(0);
	       //velocity(1) = alpha*(newPoint.y - oldPoint.y) + (1-alpha)*velocity(1);
	       //
	       ////cv::Vec2d velDiff;
	       ////velDiff(0) = abs(newVel(0) - velocity(0));
	       ////velDiff(1) = abs(newVel(1) - velocity(1));
	       //double crossProd = CrossProduct(newVel, velocity);
	       //
	       //
	       //double alpha2 = 0.1;
	       //variance = alpha2*crossProd*crossProd + (1-alpha2)*variance;
	       ////variance(0) = alpha2*velDiff(0)*velDiff(0) + (1-alpha2)*variance(0);
	       ////variance(1) = alpha2*velDiff(1)*velDiff(1) + (1-alpha2)*variance(1);
	       //
	       //double alpha3 = 0.5;
	       //double newAngle;
	       //double angleDiff;
	       //newAngle = atan2(newVel(1),newVel(0));
	       //
	       //angle = alpha3*(newAngle) + (1-alpha3)*angle;
	       //angleDiff = abs(newAngle-angle);
	       //
	       //angleVariance = alpha3*angleDiff*angleDiff + (1-alpha3)*angleVariance;
	       //
	       //calcMags();	      
	  }

	  double getAngleVariance()
	  {
	       return angleVariance;
	  }

	  double getVelMag()
	  {
	       return velMag;
	  }

	  double getVarMag()
	  {
	       return varMag;
	  }

	  double getVelVar()
	  {
	       return variance;
	  }
	  
	  void incAge() 
	  {
	       //if (age < maxAge) {
	       age++;
		    //}
	  }

	  void resetAge()
	  {
	       age = 0;
	  }
	  
	  void decAge()
	  {
	       if (age > maxAge) {
		    age = maxAge;
	       }
	       age--;
	  }

	  int getAge()
	  {
	       return age;
	  }

	  void addBlob(int index, syllo::Blob blob)
	  {
	       blobs[index] = blob;
	  }

	  cv::Point getCentroid()
	  {
	       return centroid;
	  }

	  void setCentroid(cv::Point centroid)
	  {
	       this->centroid = centroid;
	       addCentroid(this->centroid);
	  }

	  void calcCentroid()
	  {
	       double xSum = 0;
	       double ySum = 0;
	       double xAvg = 0;
	       double yAvg = 0;

               int size = blobs.size();
	       
	       if (size > 0) {
		    std::map<int,syllo::Blob>::iterator it;
		    for (it = blobs.begin() ; it != blobs.end() ; it++) {
			 xSum += it->second.getCentroid().x;
			 ySum += it->second.getCentroid().y;
		    }
		    xAvg = (double)xSum / (double)size;
		    yAvg = (double)ySum / (double)size;
		    centroid = cv::Point(xAvg,yAvg);
	       } else {
		    centroid = cv::Point(-1,-1);
	       }

	       //if (prevTime != -1) {
	       //	    velocity(0) = 1;
	       //}
	  }
     };
}

#endif
