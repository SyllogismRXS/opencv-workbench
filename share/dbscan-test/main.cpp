#include <iostream>
#include <stdio.h>

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/random.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <opencv_workbench/wb/WB.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

#include <opencv_workbench/wb/Blob.h>

// to store queries results
#include <list>

using namespace std;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, wb::Blob*> value;
typedef bgi::rtree< value, bgi::dynamic_rstar > rtree_t;

//bool is_within(value const& v, point, double dist) 
//{ 
//     //return v.is_within(); 
//     return false;
//}

void region_query(rtree_t &rtree, double eps, wb::Blob *b, std::list<wb::Blob*> &neighbors)
{
     std::list<value> returned_values;
     //point sought = point(b->centroid().x, b->centroid().y);          
     //rtree.query(bgi::nearest(sought, 20), std::back_inserter(returned_values));
      
     // find values intersecting some area defined by a box
     box bounds = rtree.bounds();
     rtree.query(bgi::intersects(bounds), std::back_inserter(returned_values));
     
     cout << "Blob ID: " << b->id() << " has neighbors... (" << returned_values.size() << ")" << endl;
     
     for (std::list<value>::iterator it = returned_values.begin(); it != returned_values.end(); it++) {
          double dist = wb::distance<cv::Point2f>(b->centroid(), it->second->centroid());
          if (dist > eps) {
               //break;
               continue;
          }
          cout << "\t" << it->second->id() << " : " << dist << endl;
          neighbors.push_back(it->second);
     }
}
     
int main()
{
     std::list<wb::Blob*> blobs_;
     
     wb::Blob *b1 = new wb::Blob();
     b1->set_id(1);
     b1->set_bbox(BoundingBox(0,2,0,2));
     b1->set_centroid(cv::Point(1,1));
     blobs_.push_back(b1);

     wb::Blob *b2 = new wb::Blob();
     b2->set_id(2);
     b2->set_bbox(BoundingBox(5,7,5,7));
     b2->set_centroid(cv::Point(6,6));
     blobs_.push_back(b2);

     wb::Blob *b3 = new wb::Blob();
     b3->set_id(3);
     b3->set_bbox(BoundingBox(8,9,5,6));
     b3->set_centroid(cv::Point(8,5));
     blobs_.push_back(b3);

     wb::Blob *b4 = new wb::Blob();
     b4->set_id(4);
     b4->set_bbox(BoundingBox(7,9,1,2));
     b4->set_centroid(cv::Point(8,1));
     blobs_.push_back(b4);       

     wb::Blob *b5 = new wb::Blob();
     b5->set_id(5);
     b5->set_bbox(BoundingBox(0,3,4,7));
     b5->set_centroid(cv::Point(1,5));
     blobs_.push_back(b5);       

     wb::Blob *b6 = new wb::Blob();
     b6->set_id(6);
     b6->set_bbox(BoundingBox(1,2,5,6));
     b6->set_centroid(cv::Point(1,5));
     blobs_.push_back(b6);       
     
     // Test distance between rectangles
     for (std::list<wb::Blob*>::iterator it1 = blobs_.begin(); 
          it1 != blobs_.end(); it1++ ) {
          for (std::list<wb::Blob*>::iterator it2 = blobs_.begin(); 
               it2 != blobs_.end(); it2++ ) {
               double dist = BoundingBox::distance((*it1)->bbox(), (*it2)->bbox());
               cout << "Blobs: " << (*it1)->id() << " and " << (*it2)->id() << " : " << dist << endl;
          }
     }     
     
     ////////////////////
     // Boost RTree
     ////////////////////    
     
     // Build the R* Tree
     rtree_t rtree(bgi::dynamic_rstar(blobs_.size()));     
     for (std::list<wb::Blob*>::iterator it = blobs_.begin(); it != blobs_.end();
          it++ ) {
          // create a box
          box b(point((*it)->bbox().xmin(), (*it)->bbox().ymin()), point((*it)->bbox().xmax(), (*it)->bbox().ymax()));
          // insert new value
          rtree.insert(std::make_pair(b, *it));
     }

     unsigned int MinPts = 3;
     double eps = 8;
     
     // 0 - unset, -1 is noise, 1+ are cluster ids.
     std::map<int, std::list<wb::Blob*> > clusters_;
     
     int UNSET_ID = 0;
     int NOISE_ID = -1;
     int cluster_id = 1;
     // for each point P in dataset D
     for (std::list<wb::Blob*>::iterator it1 = blobs_.begin(); it1 != blobs_.end();
          it1++ ) {          
                               
          if ((*it1)->visited()) {
               continue;
          }          
          (*it1)->set_visited(true);          

          std::list<wb::Blob*> neighbors;
          region_query(rtree,eps,*it1,neighbors);
          
          if (neighbors.size() < MinPts) {
               (*it1)->set_cluster_id(NOISE_ID);
               clusters_[NOISE_ID].push_back(*it1);
               cout << "NOISE - Blob ID: " << (*it1)->id() << endl;
          } else {
               //add P to cluster C
               (*it1)->set_cluster_id(cluster_id);
               clusters_[cluster_id].push_back(*it1);
               
               //for each point P' in NeighborPts
               std::list<wb::Blob*>::iterator it2 = neighbors.begin();
               while (it2 != neighbors.end()) {                    
                    //if P' is not visited                    
                    if (!(*it2)->visited()) {
                         (*it2)->set_visited(true); //mark P' as visited
                         
                         // Get neighbors2
                         std::list<wb::Blob*> neighbors2;
                         region_query(rtree,eps,*it2,neighbors2);
                         if (neighbors2.size() >= MinPts) {
                              //NeighborPts = NeighborPts joined with NeighborPts'
                              for (std::list<wb::Blob*>::iterator it_copy = neighbors2.begin();                                    
                                   it_copy != neighbors2.end(); it_copy++) {
                                   neighbors.push_back(*it_copy);                                   
                              }
                         }
                    }

                    //if P' is not yet member of any cluster
                    if ((*it2)->cluster_id() == UNSET_ID) {
                         (*it2)->set_cluster_id(cluster_id);
                         clusters_[cluster_id].push_back(*it2);  //add P' to cluster C
                    }                    
                    it2++;
               }
               cluster_id++; // Next cluster id
          }                              
     }
     
     // Print out cluster results:
     for (std::map<int, std::list<wb::Blob*> >::iterator it1 = clusters_.begin();
               it1 != clusters_.end(); it1++) {

          if (it1->first == NOISE_ID) {
               cout << "NOISE IDS: " << endl;
          } else if (it1->first == UNSET_ID) {
               cout << "WARNING: UNSET_ID FOUND: " << endl;
          } else {
               cout << "Cluster: " << it1->first << endl;
          }          
               
          for (std::list<wb::Blob*>::iterator it2 = it1->second.begin(); 
               it2 != it1->second.end(); it2++) {
               cout << "\tBlob: " << (*it2)->id() << endl;
          }
     }
     
     /// box bounds = rtree.bounds();
     /// 
     /// // find values intersecting some area defined by a box
     /// std::list<value> result_s;
     /// rtree.query(bgi::intersects(bounds), std::back_inserter(result_s));
     /// 
     /// for (std::list<value>::iterator it = result_s.begin(); it != result_s.end(); it++) {
     ///      cout << "ID: " << it->second->id() << endl;
     ///      it->second->set_id( it->second->id()+1 );
     /// }
     /// 
     /// result_s.clear();
     /// rtree.query(bgi::intersects(bounds), std::back_inserter(result_s));
     /// 
     /// for (std::list<value>::iterator it = result_s.begin(); it != result_s.end(); it++) {
     ///      cout << "ID: " << it->second->id() << endl;
     /// }

     
     return 0;
}
