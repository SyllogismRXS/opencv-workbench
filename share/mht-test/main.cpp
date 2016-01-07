#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <boost/graph/adjacency_list.hpp> 
#include <boost/graph/graphviz.hpp>

#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/wb/Blob.h>
#include <opencv_workbench/wb/BlobProcess.h>

using std::cout;
using std::endl;

using namespace boost; 

class CustomObject{
private:
     int currentId;
public:
     CustomObject(int id){
          currentId = id;
     }

};

//Define a class that has the data you want to associate to every vertex and edge
//struct Vertex{ int foo;};
class Vertex{
public:
     int foo;     
protected:
private:
};

namespace boost {
enum vertex_Vertex_t { vertex_Vertex };
BOOST_INSTALL_PROPERTY(vertex, Vertex);
}

int main() {
     cout << "MHT Test" << endl;


     std::vector<wb::Blob> measurements;
     std::vector<wb::Blob> tracks;
     std::vector<wb::Blob> fused_tracks;
     
     // Using two "established" tracks
     wb::Blob t1;
     t1.set_id(1);
     t1.set_undistorted_centroid(cv::Point2f(1,1));
     wb::Point p1;
     p1.set_position(cv::Point(1,1));
     p1.set_value(255);
     t1.add_point(p1);
     t1.set_frame(10);
     t1.init();
     t1.set_age(10);

     wb::Blob t2;
     t2.set_id(2);
     t2.set_undistorted_centroid(cv::Point2f(4,1));
     wb::Point p2;
     p2.set_position(cv::Point(4,1));
     p2.set_value(255);
     t2.add_point(p2);
     t2.set_frame(10);
     t2.init();
     t2.set_age(10);

     tracks.push_back(t1);
     tracks.push_back(t2);

     // Using three "measurements"
     wb::Blob m1;
     m1.set_id(1);
     m1.set_undistorted_centroid(cv::Point2f(2.5,1));
     wb::Point p3;
     p3.set_position(cv::Point(2.5,1));
     p3.set_value(255);
     m1.add_point(p3);
     m1.set_frame(10);
     m1.init();
     m1.set_age(1);

     wb::Blob m2;
     m2.set_id(1);
     m2.set_undistorted_centroid(cv::Point2f(4,0));
     wb::Point p4;
     p4.set_position(cv::Point(4,0));
     p4.set_value(255);
     m2.add_point(p4);
     m2.set_frame(10);
     m2.init();
     m2.set_age(1);

     wb::Blob m3;
     m3.set_id(1);
     m3.set_undistorted_centroid(cv::Point2f(5,0));
     wb::Point p5;
     p5.set_position(cv::Point(5,0));
     p5.set_value(255);
     m3.add_point(p5);
     m3.set_frame(10);
     m3.init();
     m3.set_age(1);
    
     measurements.push_back(m1);
     measurements.push_back(m2);
     measurements.push_back(m3);
     
     BlobProcess blob_process;
     blob_process.assign_mht(measurements, tracks, fused_tracks);

     /// // // Edges stored in std::list
     /// // // Nodes stored in std::vector
     /// // // Undirected graph
     /// // typedef boost::adjacency_list<listS, vecS, undirectedS> mygraph; 
     /// // 
     /// // mygraph g;
     /// // add_edge (0, 1, g);
     /// // add_edge (0, 3, g);
     /// // add_edge (1, 2, g);
     /// // add_edge (2, 3, g);
     /// // add_edge (1, 3, g);
     /// // 
     /// // mygraph::vertex_iterator vertexIt, vertexEnd;
     /// // mygraph::adjacency_iterator neighbourIt, neighbourEnd;
     /// // tie(vertexIt, vertexEnd) = vertices(g);
     /// // for (; vertexIt != vertexEnd; ++vertexIt) { 
     /// //      cout << *vertexIt << " is connected with "; 
     /// //      tie(neighbourIt, neighbourEnd) = adjacent_vertices(*vertexIt, g); 
     /// //      for (; neighbourIt != neighbourEnd; ++neighbourIt) 
     /// //           cout << *neighbourIt << " "; 
     /// //      cout << "\n"; 
     /// // }
     /// // 
     /// // cout << "\n----------" << endl;
     /// // boost::write_graphviz(cout, g);
     /// // cout << "\n-----------" << endl;
     /// 
     /// //////////////////////////////
     ///      
     /// // //Define the graph using those classes
     /// typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Vertex> Graph;
     /// //Some typedefs for simplicity
     /// typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
     /// typedef boost::graph_traits<Graph>::edge_descriptor edge_t;
     ///  
     /// //Instanciate a graph
     /// Graph g;
     /// 
     /// vertex_t root = boost::add_vertex(g);
     /// g[root].foo = 1;
     /// 
     /// vertex_t trav = root;
     /// for (int i = 0 ; i < 3; i++) {
     ///      vertex_t vert = boost::add_vertex(g);
     ///      g[vert].foo = (i+1)*5;
     ///      
     ///      edge_t e; bool b;
     ///      boost::tie(e,b) = boost::add_edge(trav,vert,g);
     ///      trav = vert;
     /// }                   
     /// 
     /// // write graph to console
     /// cout << "\n-- graphviz output START --" << endl;
     /// boost::write_graphviz(std::cout, g,
     ///                       boost::make_label_writer(boost::get(&Vertex::foo, g)));
     /// cout << "\n------------" << endl;     
     /// 
     /// 
     /// Graph::vertex_iterator vertexIt, vertexEnd;
     /// Graph::adjacency_iterator neighbourIt, neighbourEnd;
     /// tie(vertexIt, vertexEnd) = vertices(g);
     /// for (; vertexIt != vertexEnd; ++vertexIt) { 
     ///      cout << *vertexIt << "(" << syllo::int2str(g[*vertexIt].foo) << ") is connected with "; 
     ///      tie(neighbourIt, neighbourEnd) = adjacent_vertices(*vertexIt, g); 
     ///      for (; neighbourIt != neighbourEnd; ++neighbourIt)  {
     ///           cout << *neighbourIt << " "; 
     ///      }
     ///      cout << "\n"; 
     /// }
     /// 
     /// // get the property map for vertex indices
     /// //property_map<Graph, int Vertex::*>::type index = get(Vertex(), g);
     /// //property_map<Graph, vertex_index_t>::type index = get(vertex_index, g);
     /// //property_map<Graph, vertex_Vertex_t>::type index;// = get(vertex_Vertex, g);
     /// //property_map<Graph, Vertex>::type index = get(vertex_index, g);
     /// 
     /// // std::cout << "vertices(g) = ";
     /// // typedef graph_traits<Graph>::vertex_iterator vertex_iter;
     /// // std::pair<vertex_iter, vertex_iter> vp;
     /// // for (vp = vertices(g); vp.first != vp.second; ++vp.first) {
     /// //      std::cout << index[*vp.first].foo <<  " ";
     /// // }
     /// // std::cout << std::endl;
     /// 
     /// // If you want to write to dot file
     /// //std::ofstream dotfile ("/home/syllogismrxs/test.dot");
     /// //boost::write_graphviz(dotfile, g,
     /// //                      boost::make_label_writer(boost::get(&Vertex::foo, g)));

     cout << "Done" << endl;
     return 0;
}
