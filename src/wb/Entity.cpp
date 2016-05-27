#include <iostream>
#include <limits.h>
#include <boost/tokenizer.hpp>

#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/wb/WB.h>

#include "Entity.h"

using std::cout;
using std::endl;

namespace wb {

     Entity::Entity() : id_(-1), name_("unknown:-1"), 
                        type_(Unknown), age_(0), occluded_age_(0), 
                        occluded_(false), is_confirmed_(false), visited_(false), 
                        cluster_id_(0), avg_pixel_value_(-1)
     {
          stream_ = NULL;          
          matched_ = false;    

          confirmed_age_ = 3;
          dead_occluded_age_ = 11;
     }     

     void Entity::set_estimated_pixel_centroid(cv::Point p)
     {
          pixel_tracker_.set_position(p);
     }

     void Entity::set_estimated_centroid(cv::Point2d p)
     {
          tracker_.set_position(p);
     }

     void Entity::init()
     {
          compute_metrics();

          start_pixel_centroid_ = pixel_centroid_;

          pixel_tracker_.set_measurement(pixel_centroid_);
          tracker_.set_measurement(centroid_);          
     }

     cv::Point Entity::trail_history(int past)
     {
          cv::Point result;
          int count = 0;
          for (std::list<cv::Point2f>::reverse_iterator rit = trail_.rbegin();
               rit != trail_.rend(); ++rit) {
               result = *rit;
               if (count >= past) {
                    return result;
               }
               count++;
          }
          return result;
     }

     cv::Point Entity::estimated_pixel_centroid()
     {
          // the Kalman filter's estimate centroid. Otherwise, use the entity's
          // current centroid if it isn't confirmed yet
          //if (this->is_confirmed()) {               
               return pixel_tracker_.position();
          //     //return est_pixel_centroid_;
          //} else {
          //     return pixel_centroid_;
          //}
     }

     cv::Point Entity::estimated_pixel_velocity()
     {
          return pixel_tracker_.velocity();
     }
     
     cv::Point2d Entity::estimated_centroid()
     {
          // the Kalman filter's estimate centroid. Otherwise, use the entity's
          // current centroid if it isn't confirmed yet
          //if (this->is_confirmed()) {               
               return tracker_.position();
          //} else {
          //     return centroid_;
          //}
          return centroid_;
     }

     int Entity::estimated_pixel_value()
     {
          return floor(pixel_value_tracker_.value());
     }
     
     int Entity::estimated_blob_size()
     {
          return floor(size_tracker_.value());
     }

     int Entity::lower_pixel_value(float num_stds)
     {          
          return pixel_value_tracker_.lower_end(num_stds);
     }

     int Entity::lower_blob_size(float num_stds)
     {
          return size_tracker_.lower_end(num_stds);
     }

     Ellipse Entity::error_ellipse(double confidence)
     {
          //return kf_.error_ellipse(confidence);
          return pixel_tracker_.error_ellipse(confidence);
     }

     void Entity::predict_tracker()
     {
          pixel_tracker_.predict();          
          tracker_.predict();
          
          pixel_value_tracker_.predict();
          size_tracker_.predict();

          //cout << "Update - Track: " << id_ << " , size=" << size_tracker_.value()
          //     << ", value=" << pixel_value_tracker_.value() << endl;
     }

     void Entity::correct_tracker()
     {          
          //// correct tracker update using the newly computed centroid.
          pixel_tracker_.set_measurement(pixel_centroid_);
          tracker_.set_measurement(centroid_);

          // Correct average pixel value tracker
          pixel_value_tracker_.set_value(avg_pixel_value_);

          // Correct blob size tracker
          size_tracker_.set_value((float)points_.size());
     }

     void Entity::update_trail()
     {
          trail_.push_back(estimated_pixel_centroid());
     }

     void Entity::compute_metrics()
     {
          ////////////////////////////////////////////////////////
          // Compute Centroid and Bounding Box Rectangle
          ////////////////////////////////////////////////////////
          int sum_x_pixel = 0, sum_y_pixel = 0, sum_value_pixel = 0;
          int xmin_pixel = INT_MAX, ymin_pixel = INT_MAX;
          int xmax_pixel = INT_MIN, ymax_pixel = INT_MIN;          
                    
          for (std::vector<Point>::iterator it = points_.begin(); 
               it != points_.end(); it++) {
               
               // Calculate centroid in "image" space
               int x_pixel = it->position().x;
               int y_pixel = it->position().y;

               int value_pixel = it->value();
               sum_value_pixel += value_pixel;

               sum_x_pixel += x_pixel * value_pixel;
               sum_y_pixel += y_pixel * value_pixel;

               if (x_pixel < xmin_pixel) {
                    xmin_pixel = x_pixel;
               }
               if (x_pixel > xmax_pixel) {
                    xmax_pixel = x_pixel;
               }
               if (y_pixel < ymin_pixel) {
                    ymin_pixel = y_pixel;
               }
               if (y_pixel > ymax_pixel) {
                    ymax_pixel = y_pixel;
               }                              
          }

          double avg_x_pixel = (double)sum_x_pixel / (double)sum_value_pixel;
          double avg_y_pixel = (double)sum_y_pixel / (double)sum_value_pixel;

          avg_pixel_value_ = (double)sum_value_pixel / (double)points_.size();
          
          pixel_value_tracker_.set_value(avg_pixel_value_);
          size_tracker_.set_value((float)points_.size());
                    
          pixel_centroid_ = cv::Point(cvRound(avg_x_pixel), cvRound(avg_y_pixel));
          
          // +1's to account for slight error in rectangle drawing?
          //bbox_ = cv::Rect rect(xmin, ymin, xmax-xmin+1, ymax-ymin+1);
          bbox_.set_box(xmin_pixel, xmax_pixel+1, ymin_pixel, ymax_pixel+1);
          
          // Get the centroid in X/Y Cartesian space
          if (stream_ != NULL) {
               centroid_ = wb::rowcol2cartesian(stream_, pixel_centroid_.y, pixel_centroid_.x);               
          } else {
               cout << "Missing stream object" << endl;
               centroid_ = cv::Point2d(-1,-1);               
          }
     }

     cv::Rect Entity::rectangle()
     {
          return bbox_.rectangle();
     }     

     bool Entity::is_confirmed()
     {
          if (is_confirmed_) return true;

          if (age_ >= confirmed_age_) {
               is_confirmed_ = true;
               return true;
          } else {
               is_confirmed_ = false;
               return false;
          }
     }

     bool Entity::is_dead()
     {
          if (occluded_age_ >= dead_occluded_age_) {
               return true;
          } else {
               return false;
          }          
     }

     void Entity::inc_age()
     {          
          age_++;          
     }

     void Entity::set_age(int age)
     {
          age_ = age;          
     }

     void Entity::dec_age()
     {
          age_--;
     }

     void Entity::missed_track()
     {                    
          this->set_occluded(true);
          this->inc_occluded_age();
     }

     void Entity::detected_track()
     {
          this->inc_age();
          this->set_occluded(false);

          // Update the Kalman Filter Tracker
          this->correct_tracker();

          //cout << "Detected - Track: " << id_ << " , size=" << points_.size()
          //     << ", value=" << avg_pixel_value_ << endl;                    
     }

     void Entity::new_track(int id)
     {
          this->set_id(id);
          this->set_age(1);
          this->set_occluded(false);
     }

     void Entity::set_occluded(bool occluded) 
     {
          occluded_ = occluded; 
          if (!occluded_) {
               this->set_occluded_age(0);
          }
     }

     void Entity::copy_track_info(Entity &other)
     {
          this->set_id(other.id());
          this->set_age(other.age());          
          this->set_estimated_pixel_centroid(other.estimated_pixel_centroid());
          this->set_estimated_centroid(other.estimated_centroid());
          this->set_start_centroid(other.start_centroid());
          this->set_start_pixel_centroid(other.start_pixel_centroid());
          this->set_tracker(other.tracker());
          this->set_pixel_tracker(other.pixel_tracker());
          this->trail_ = other.trail_;
     }

     void Entity::copy_meas_info(Entity &other)
     {
          this->centroid_ = other.centroid();
          this->pixel_centroid_ = other.pixel_centroid();
          this->bbox_ = other.bbox();
          this->points_ = other.points();
          
          this->avg_pixel_value_ = other.avg_pixel_value();
     }

     void Entity::matched_track(Entity &match)
     {
          // Update the current entity with the track info from "match"
          this->copy_track_info(match);

          // Update track variables
          this->detected_track();
     }

     void Entity::set_pixel_R(double r)
     {
          pixel_tracker_.set_R(r);
     }

     void Entity::set_pixel_P(double p)
     {
          pixel_tracker_.set_P(p);
     }

     void Entity::set_R(double r)
     {
          tracker_.set_R(r);
     }

     void Entity::set_R(double r0, double r1, double r2, double r3)
     {
          tracker_.set_R(r0, r1, r2, r3);
     }

     void Entity::set_P(double p)
     {
          tracker_.set_P(p);
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

     void Entity::add_points(std::vector<wb::Point> &points)
     {
          std::vector<wb::Point>::iterator it = points.begin();
          for (; it != points.end(); it++) {
               points_.push_back(*it);
          }
     }

     std::string Entity::name()
     {
          std::ostringstream convert;
          convert << id_;

          switch (type_) {
          case Unknown:
               name_ = "unknown:" + convert.str();
               break;
          case Diver:
               name_ = "diver:" + convert.str();
               break;
          case Fish:
               name_ = "fish:" + convert.str();
               break;
          case Clutter:
               name_ = "clutter:" + convert.str();
               break;
          case APoint:
               name_ = "point:" + convert.str();
               break;
          default:
               name_ = "unknown:" + convert.str();
          }

          return name_;
     }

     std::string Entity::type_str()
     {
          switch (type_) {
          case Unknown:
               return "unknown";
               break;
          case Diver:
               return "diver";
               break;
          case Fish:
               return "fish";
               break;
          case Clutter:
               return "clutter";
               break;
          case APoint:
               return "point";
               break;
          default:
               return "unknown";
          }
     }

     wb::Entity::EntityType_t Entity::str_2_type(std::string str)
     {
          if (str == "unknown") {
               return Unknown;
          } else if (str == "diver") {
               return Diver;
          } else if (str == "fish") {
               return Fish;
          } else if (str == "clutter") {
               return Clutter;
          } else if (str == "point") {
               return APoint;
          } else {
               return Unknown;
          }
     }

     void Entity::set_name(std::string name)
     {
          boost::char_separator<char> sep(":");
          boost::tokenizer<boost::char_separator<char> > tokens(name, sep);
          int iter = 0;
          std::string type_str = "unknown";
          id_ = -2;
          for (boost::tokenizer<boost::char_separator<char> >::iterator tok_iter = tokens.begin();
               tok_iter != tokens.end(); ++tok_iter) {

               if (iter == 0) {
                    type_str = *tok_iter;
               } else if (iter == 1) {
                    id_  = syllo::str2int(std::string(*tok_iter));
               }
               iter++;
          }          
          type_ = Entity::str_2_type(type_str);
     }
}
