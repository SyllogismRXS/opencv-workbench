#include <iostream>

#include "Cluster.h"

using std::cout;
using std::endl;

namespace wb {

     Cluster::Cluster()
     {
     
     }

     cv::Point Cluster::centroid()
     {
          int sum_x = 0, sum_y = 0;
          std::vector<Point*>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               sum_x += (*it)->position().x;
               sum_y += (*it)->position().y;
          }
          
          double avg_x = (double)sum_x / points_.size();
          double avg_y = (double)sum_y / points_.size();
          return cv::Point(round(avg_x), round(avg_y));          
     }

}
