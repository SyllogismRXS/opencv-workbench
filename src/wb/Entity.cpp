#include <iostream>

#include "Entity.h"

using std::cout;
using std::endl;

namespace wb {

     Entity::Entity() : age_(0)
     {
     
     }

     void Entity::compute_metrics()
     {
          ////////////////////////////////////////////////////////
          // Compute Centroid and Bounding Box Rectangle
          ////////////////////////////////////////////////////////
          int sum_x = 0, sum_y = 0;
          int xmin = 999999999, ymin = 999999999, xmax = -999, ymax = -999;
          std::vector<Point>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               int x = it->position().x;
               int y = it->position().y;
               
               sum_x += x;
               sum_y += y;

               if (x < xmin) {
                    xmin = x;
               }
               if (x > xmax) {
                    xmax = x;
               }
               if (y < ymin) {
                    ymin = y;
               }
               if (y > ymax) {
                    ymax = y;
               }
          }
          
          double avg_x = (double)sum_x / points_.size();
          double avg_y = (double)sum_y / points_.size();

          centroid_ = cv::Point(round(avg_x), round(avg_y));
          
          // +1's to account for slight error in rectangle drawing?
          rectangle_ = cv::Rect(xmin, ymin, xmax-xmin+1, ymax-ymin+1);
     }

     cv::Point Entity::centroid()
     {
          return centroid_;
     }

     cv::Rect Entity::rectangle()
     {
          return rectangle_;
     }

     bool Entity::is_visible()
     {
          if (age_ > 10) {
               return true;
          } else {
               return false;
          }
     }

     bool Entity::is_dead()
     {
          if (age_ < -2) {
               return true;
          } else {
               return false;
          }
     }

     void Entity::inc_age()
     {
          age_++;
          if (age_ > 20) {
               age_ = 20;
          }
     }

     void Entity::dec_age()
     {
          age_--;
     }

     void Entity::remove_point(wb::Point &p)
     {          
          std::vector<Point>::iterator it = points_.begin();
          for (; it != points_.end(); it++) {               
               if (it->position() == p.position()) {
                    points_.erase(it);
                    break;
               }
          }
     }               

     
}
