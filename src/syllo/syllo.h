#ifndef SYLLO_H_
#define SYLLO_H_

#include <cv.h>
#include <highgui.h>

#include <Eigen/Dense>

//#include "Sonar.h"
//#include "filters.h"
#include <opencv_workbench/track/KalmanFilter.h>
#include <opencv_workbench/track/ParticleFilter.h>
#include <opencv_workbench/track/MotionModels.h>

namespace syllo
{

     //typedef enum{
     //     NoneType = 0,
     //     CameraType = 1,
     //     AVIType = 2,
     //     SonarType = 3
     //}StreamType;

     //class Stream {
     //protected:
     //     cv::VideoCapture *vcap;
     //     sonar::Sonar *sonar;
     //     StreamType type;
     //public:
     //     Stream(cv::VideoCapture *vcap, sonar::Sonar *sonar)
     //     {
     //          this->vcap = vcap;
     //          this->sonar = sonar;
     //     }
     //     
     //     int open(int num)
     //     {
     //          vcap->open(num);
     //          vcap->set(CV_CAP_PROP_FRAME_WIDTH, 480);
     //          vcap->set(CV_CAP_PROP_FRAME_HEIGHT, 640);
     //          type = CameraType;
     //     }
     //     
     //     int open(std::string fn) 
     //     {
     //          if (fn.substr(fn.find_last_of(".") + 1) == "son") {
     //   	    sonar->setSonarFile(fn);
     //   	    sonar->setRange(0,45);
     //   	    sonar->init();
     //   	    type = SonarType;
     //          } else if (fn.substr(fn.find_last_of(".") + 1) == "avi") {
     //   	    vcap->open( fn );
     //   	    type = AVIType;
     //          } else {
     //   	    printf("Invalid file extension: %s\n", fn.c_str());
     //   	    return -1;
     //          }   
     //     }
     //
     //     bool isOpened()
     //     {
     //          if (type == SonarType) {
     //   	    return true;
     //          } else {
     //   	    return vcap->isOpened();
     //          }
     //     }
     //
     //     bool read(cv::Mat &frame) 
     //     {
     //          if(type == SonarType) {
     //   	    sonar->getNextSonarImage(frame);
     //          } else {
     //   	    return vcap->read(frame);
     //          }
     //     }
     //
     //     int GetFrameCount()
     //     {
     //          switch (type) {
     //          case AVIType:
     //   	    return vcap->get(CV_CAP_PROP_FRAME_COUNT);
     //   	    break;
     //          case CameraType:
     //   	    return 0;
     //   	    break;
     //          case SonarType:
     //               return sonar->getNumPings();
     //          default:
     //   	    return -1;
     //          }
     //     }
     //
     //     int GetCurrentFrameNum()
     //     {
     //          switch (type) {
     //          case AVIType:
     //               return vcap->get(CV_CAP_PROP_POS_FRAMES);
     //   	    break;
     //          case CameraType:
     //   	    return 0;
     //   	    break;
     //          case SonarType:
     //               return sonar->getCurrentPingNum();
     //          default:
     //   	    return -1;
     //          }
     //     }
     //
     //     void SetFrameNum(int num)
     //     {
     //          switch (type) {
     //          case AVIType:
     //               break;
     //          case CameraType:
     //               break;
     //          case SonarType:
     //               sonar->setFrameNum(num);
     //               break;
     //          default:
     //               break;
     //          }
     //     }
     //
     //
     //     int width()
     //     {
     //          switch (type) {
     //          case AVIType:
     //   	    return vcap->get(CV_CAP_PROP_FRAME_WIDTH);
     //   	    break;
     //          case CameraType:
     //   	    return vcap->get(CV_CAP_PROP_FRAME_WIDTH);
     //   	    break;
     //          case SonarType:
     //               sonar->width();
     //               break;
     //          default:
     //   	    return -1;
     //          }
     //     }
     //
     //     int height()
     //     {
     //          switch (type) {
     //          case AVIType:
     //   	    return vcap->get(CV_CAP_PROP_FRAME_HEIGHT);
     //   	    break;
     //          case CameraType:
     //   	    return vcap->get(CV_CAP_PROP_FRAME_HEIGHT);
     //   	    break;
     //          case SonarType:
     //               return sonar->height();
     //               break;
     //          default:
     //   	    return -1;
     //          }
     //     }
     //};
    
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
	  }

	  int step()
	  {
	       if (velDet) {
		    //kalmanTracker.step(mu, covar, estVel);
		    centroid = cv::Point((int)mu(0), (int)mu(1));
	       }

	       age--;
	  }

	  int velocityUpdate()
	  {
	       //estVel(0) 
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

	  int setSize(int size)
	  {
	       this->size = size;
	  }
     
     };

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
	       velMag = sqrt( pow(velocity.x,2) + (velocity.y,2));
	       //varMag = sqrt( pow(variance(0),2) + (variance(1),2));
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

	       int count = 0;
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

     cv::Point getTypeCentroid(EntityType type, std::map<int,syllo::Cluster> &clusters);

     int flipClusterEntity(cv::Point point, std::map<int,syllo::Cluster> &clusters);
     int tagClusterEntity(cv::Point point, EntityType target, EntityType others, 
                          std::map<int,syllo::Cluster> &clusters);

     int ClusterTemplateMatch(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Cluster> &clusters, cv::Mat &templateImg);

     int formClusters(std::map<int,syllo::Blob> blobs, std::map<int,syllo::Cluster> &clusters);
     int ClusterMatch(std::map<int,syllo::Cluster> &prevClusters, std::map<int,syllo::Cluster> &clusters, std::map<int,syllo::Cluster> &newClusters, std::map<int,syllo::Cluster> &allClusters, int curTime, std::map<int,bool> &IDs);
     int drawClusters(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Cluster> &clusters, int radius);

     int StepClusterPF(std::map<int,syllo::Cluster> &clusters);
     int drawClusterParticles(std::map<int,syllo::Cluster> &clusters, cv::Mat &input, cv::Mat &output);

     int getFastestCluster(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster);
     int getConstVelCluster(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster);
     int getFarthestTravel(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster);

     int templateDetect(const cv::Mat &inputImg, const cv::Mat &templImg, cv::Mat &outputImg);

     int DetectBlobs(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Blob> &blobs);

     int BlobTemplateMatch(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Blob> &blobs, cv::Mat &templateImage);

     int RunningGaussian(double alpha, cv::Mat &img1, cv::Mat &img2, cv::Mat &avgs, cv::Mat &varImg);

     cv::Point calcCentroid(cv::Mat &img, uchar value);
     int CentroidsOfBlobs(cv::Mat &img, std::map<int,syllo::Blob> &blobs);

     int drawCircleList(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Blob> &blobs, int radius, cv::Scalar color); 

     int drawBlobTargets(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Blob> &blobs, int radius);

     cv::Point GetLargestBlob(std::map<int,syllo::Blob> &blobs);

     int BlobMatch(std::map<int,syllo::Blob> &prevBlobs, std::map<int,syllo::Blob> &blobs, std::map<int,syllo::Blob> &newBlobs, int curTime);


     int drawParticles(ParticleFilter &pf, cv::Mat &src, cv::Mat &dst);

     // Tests...
     //int var = 5;
     void helloWorld();
}

#endif 
