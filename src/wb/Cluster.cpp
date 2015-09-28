#include <iostream>

#include "Cluster.h"

using std::cout;
using std::endl;

namespace wb {

     static inline cv::Point calcPoint(cv::Point2f center, double R, double angle)
     {
          return center + cv::Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
     }
     
     Cluster::Cluster()
     {          
          distance_ = 999999;
          matched_ = false;
          match_ = NULL;
          
          //A.resize(2,2);
          //B.resize(2,2);
          //C.resize(2);
          //R.resize(2,2);
          //Q.resize(2,2);
          //mu.resize(2);
          //covar.resize(2,2);
          //u.resize(2);
          //
          //estVel.resize(2);
          //
          //A << 0,0,0,0;
          //B << 1,0,0,1;
          //C << 1,1;
          //R << 0.1,0.1,0.1,0.1;
          //Q << 0.1,0.1,0.1,0.1;
	  //     
          //mu << 0,1;
          //covar << 0.1,0.1,0.1,0.1;
          //u << 1,1;
          //
          //estVel << 0,0;
          //
          //ekf_.setModel(A,B,C,R,Q,0.066666667);                    
     }          

     
}
