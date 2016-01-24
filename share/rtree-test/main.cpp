#include <iostream>
#include <stdio.h>
#include <opencv_workbench/cluster/RTree.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

#include <boost/geometry/index/rtree.hpp>

// to store queries results
#include <vector>
#include <boost/foreach.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//
// Test.cpp
//
// This is a direct port of the C version of the RTree test program.
//

using namespace std;

// From: https://github.com/nushoin/RTree

typedef int ValueType;

struct Rect
{
     Rect()  {}

     Rect(int a_minX, int a_minY, int a_maxX, int a_maxY)
          {
               min[0] = a_minX;
               min[1] = a_minY;

               max[0] = a_maxX;
               max[1] = a_maxY;
          }


     int min[2];
     int max[2];
};

struct Rect rects[] =
{
     Rect(0, 0, 2, 2), // xmin, ymin, xmax, ymax (for 2 dimensional RTree)
     Rect(5, 5, 7, 7),
     Rect(8, 5, 9, 6),
     Rect(7, 1, 9, 2),
};

int nrects = sizeof(rects) / sizeof(rects[0]);

Rect search_rect(6, 4, 10, 6); // search will find above rects that this one overlaps


bool MySearchCallback(ValueType id, void* arg)
{
     cout << "Hit data rect " << id << "\n";
     return true; // keep going
}


int main()
{
     ////////////////
     // RTree.h Example
     ////////////////
     typedef RTree<ValueType, int, 2, float> MyTree;
     MyTree tree;

     int i, nhits;
     cout << "nrects = " << nrects << "\n";

     for(i=0; i<nrects; i++) {
          tree.Insert(rects[i].min, rects[i].max, i); // Note, all values including zero are fine in this version
     }

     nhits = tree.Search(search_rect.min, search_rect.max, MySearchCallback, NULL);

     cout << "Search resulted in " << nhits << " hits\n";

     // Iterator test
     int itIndex = 0;
     MyTree::Iterator it;
     for( tree.GetFirst(it);
          !tree.IsNull(it);
          tree.GetNext(it) ) {
          int value = tree.GetAt(it);

          int boundsMin[2] = {0,0};
          int boundsMax[2] = {0,0};
          it.GetBounds(boundsMin, boundsMax);
          cout << "it[" << itIndex++ << "] " << value << " = (" << boundsMin[0] << "," << boundsMin[1] << "," << boundsMax[0] << "," << boundsMax[1] << ")\n";
     }

     // Iterator test, alternate syntax
     itIndex = 0;
     tree.GetFirst(it);
     while( !it.IsNull() )
     {
          int value = *it;
          ++it;
          cout << "it[" << itIndex++ << "] " << value << "\n";
     }     

     // Output:
     //
     // nrects = 4
     // Hit data rect 1
     // Hit data rect 2
     // Search resulted in 2 hits
     // it[0] 0 = (0,0,2,2)
     // it[1] 1 = (5,5,7,7)
     // it[2] 2 = (8,5,9,6)
     // it[3] 3 = (7,1,9,2)
     // it[0] 0
     // it[1] 1
     // it[2] 2
     // it[3] 3

     cout << "===================================" << endl;
     
     ////////////////////
     // Boost RTree
     ////////////////////
     typedef bg::model::point<float, 2, bg::cs::cartesian> point;
     typedef bg::model::box<point> box;
     typedef std::pair<box, unsigned> value;

     // create the rtree using default constructor
     //bgi::rtree< value, bgi::quadratic<16> > rtree; // max elements is 16
     //bgi::rtree< value, bgi::quadratic<16> > rtree; // max elements is 16
     // rstar
     bgi::rtree<value, bgi::dynamic_rstar> rtree(bgi::dynamic_rstar(16));
     
     // create some values
     for ( unsigned i = 0 ; i < 10 ; ++i ) {
          // create a box
          box b(point(i + 0.0f, i + 0.0f), point(i + 0.5f, i + 0.5f));
          // insert new value
          rtree.insert(std::make_pair(b, i));
     }

     // find values intersecting some area defined by a box
     box query_box(point(0, 0), point(5, 5));
     std::vector<value> result_s;
     rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));

     // find 5 nearest values to a point
     std::vector<value> result_n;
     rtree.query(bgi::nearest(point(0, 0), 5), std::back_inserter(result_n));

     // display results
     std::cout << "spatial query box:" << std::endl;
     std::cout << bg::wkt<box>(query_box) << std::endl;
     std::cout << "spatial query result:" << std::endl;
     BOOST_FOREACH(value const& v, result_s)
          std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;

     std::cout << "knn query point:" << std::endl;
     std::cout << bg::wkt<point>(point(0, 0)) << std::endl;
     std::cout << "knn query result:" << std::endl;
     BOOST_FOREACH(value const& v, result_n)
          std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;

     return 0;
}
