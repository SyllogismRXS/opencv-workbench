#ifndef BLOBPROCESS_H_
#define BLOBPROCESS_H_
/// ---------------------------------------------------------------------------
/// @file BlobProcess.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-03-08 12:38:14 syllogismrxs>
///
/// @version 1.0
/// Created: 10 Sep 2015
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
/// The BlobProcess class ...
/// 
/// ---------------------------------------------------------------------------
#include <map>
#include <vector>

#include <cv.h>
#include <highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/graph/adjacency_list.hpp> 
#include <boost/graph/graphviz.hpp>

#include "Blob.h"

typedef enum OverlayFlags {
     BLOBS          = 1 << 0,
     RECTS          = 1 << 1,
     TRACKS         = 1 << 2,
     IDS            = 1 << 3,
     ERR_ELLIPSE    = 1 << 4,
     CONFIRMED_ONLY = 1 << 5
}OverlayFlags_t;
     
inline OverlayFlags_t operator|(OverlayFlags_t a, OverlayFlags_t b)
{
     return static_cast<OverlayFlags_t>(static_cast<int>(a) | 
                                        static_cast<int>(b));
}

////////////////////////////////////////
// Definitions for Boost Graph Library
////////////////////////////////////////
namespace boost {
enum vertex_Blob_t { vertex_Blob };
BOOST_INSTALL_PROPERTY(vertex, Blob);
}

// //Define the graph using those classes
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, wb::Blob> Graph;
//Some typedefs for simplicity
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;


class TracksHypothesis {
public:
     TracksHypothesis() : probability(0)
     {          
     }
     std::vector<wb::Blob> tracks;
     vertex_t node;
     std::map<std::string,std::string> config; // "trackID:measurementID"
     double probability;
};

////////////////////////////
class BlobProcess {
public:          
     
     BlobProcess();
     
     int process_frame(cv::Mat &input, cv::Mat &original, int thresh);
     void find_blobs(cv::Mat &input, std::vector<wb::Blob> &blobs, 
                            int min_blob_size);

     void find_clusters(cv::Mat &input, std::vector<wb::Blob> &clusters, 
                            unsigned int min_cluster_size);

     void assign_hungarian(std::vector<wb::Blob> &meas, 
                           std::vector<wb::Blob> &tracks,
                           std::vector<wb::Blob> &fused);

     void assign_munkres(std::vector<wb::Blob> &meas, 
                         std::vector<wb::Blob> &tracks,
                         std::vector<wb::Blob> &fused);

     void assign_mht(std::vector<wb::Blob> &meas, 
                     std::vector<wb::Blob> &tracks,
                     std::vector<wb::Blob> &fused);

     void build_tree(vertex_t &vertex,     
                     std::vector<wb::Blob>::iterator it_meas,
                     std::vector<wb::Blob> &meas, 
                     std::vector<wb::Blob> tracks,
                     std::map<std::string,std::string> config,
                     int next_id);

     //void build_hyp_matrix(std::vector<wb::Blob>::iterator it_meas,
     //                      std::vector<wb::Blob> &meas, 
     //                      std::vector<wb::Blob> tracks,
     //                      int next_id);
     
     void overlay_blobs(cv::Mat &src, cv::Mat &dst, 
                        std::vector<wb::Blob> & blobs);
     void overlay_blobs(cv::Mat &src, cv::Mat &dst);
     void overlay_tracks(cv::Mat &src, cv::Mat &dst);  

     void overlay(cv::Mat &src, cv::Mat &dst, OverlayFlags_t flags);  
     void overlay_short_lived(cv::Mat &src, cv::Mat &dst);
     std::vector<wb::Blob> & blobs() { return blobs_; }
     void set_blobs(std::vector<wb::Blob> &blobs) { blobs_ = blobs; }

     bool consolidate_tracks(cv::Mat &in, cv::Mat &out);
     
     bool cluster_blobs(cv::Mat &in, cv::Mat &out, 
                        std::vector<wb::Blob> &new_blobs,
                        std::vector<wb::Blob> &clusters,
                        unsigned int MinPts, double eps);
     
     bool displace_detect(int history, double distance);

     void blobs_to_entities(std::vector<wb::Entity> &ents);
     
     static void blobs_to_entities(std::vector<wb::Blob> &blobs,
                                   std::vector<wb::Entity> &ents);

     void set_mask(cv::Mat &mask) { mask_ = mask; }

     void frame_ents(std::vector<wb::Entity> frame_ents) 
     { 
          BlobProcess::blobs_to_entities(frame_blobs_, frame_ents);
     }     
     
protected:
     
     static uchar valueAt(cv::Mat &img, int row, int col);
     static uchar findMin(uchar NE, uchar N, uchar NW, uchar W);
     static void labelNeighbors(cv::Mat &img, std::vector<uchar> &labelTable, 
                              uchar label, int i, int j);
     int next_available_id();

     void blob_maintenance();

     //std::map<int,wb::Blob> blobs_;
     //std::map<int,wb::Blob> prev_blobs_;
     std::vector<wb::Blob> blobs_;
     std::vector<wb::Blob> prev_blobs_;
     std::vector<wb::Blob> short_lived_;
     std::vector<wb::Blob> new_tracks_;

     std::vector<wb::Blob> clusters_;
     std::vector<wb::Blob> prev_clusters_;

     cv::Mat mask_;

     Graph graph_;

     std::vector<wb::Blob> frame_blobs_;

     double Pd_;
     double B_ft_;
     double B_nt_;
     int N_tgt_;

     std::list<TracksHypothesis> track_hyps_;
     
     std::list<vertex_t> hyps_;
     std::list<vertex_t> prev_hyps_;
     
private:
     int count_;
     int min_blob_size_;
     int next_id_;
     int next_mht_id_;
};

#endif
