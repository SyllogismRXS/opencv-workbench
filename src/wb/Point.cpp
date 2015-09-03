#include <iostream>

#include "Point.h"

using std::cout;
using std::endl;

namespace wb {
     Point::Point()
     {
          parent_ = NULL;
          assigned_ = false;
     }

     float Point::distance(Point &other)
     {
          return sqrt( pow(this->point_.x-other.point_.x,2) + 
                       pow(this->point_.y-other.point_.y,2) );
     }

     void Point::set_parent(wb::Cluster *parent)
     {
          parent_ = parent;          
     }
}
