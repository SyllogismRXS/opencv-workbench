#include <iostream>
#include <limits.h>
#include <list>

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/random.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <opencv_workbench/wb/Point.h>
#include <opencv_workbench/track/hungarian.h>
#include <opencv_workbench/utils/OpenCV_Helpers.h>
#include <opencv_workbench/utils/Ellipse.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include <opencv_workbench/wb/Entity.h>
#include <opencv_workbench/wb/WB.h>
#include "BlobProcess.h"

using std::cout;
using std::endl;

using namespace boost;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<box, wb::Blob*> value;
typedef bgi::rtree< value, bgi::dynamic_rstar > rtree_t;

BlobProcess::BlobProcess()
{
     min_blob_size_ = 30;//15, 20, 30;
     //min_blob_size_ = 5;//15, 20, 30;
     next_id_ = 0;
}

int BlobProcess::next_available_id()
{
     return next_id_++;
}

uchar BlobProcess::valueAt(cv::Mat &img, int row, int col)
{
     if (col >= 0 && col < img.cols && row >= 0 && row < img.rows) {
	       return img.at<uchar>(row,col);
     }
     // Return 0 if outside of the image bounds
     return 0;
}

uchar BlobProcess::findMin(uchar NE, uchar N, uchar NW, uchar W)
{
     uchar minChamp = UCHAR_MAX;

     // If there are no neighbors, return 0.
     if (NE == 0 && N == 0 && NW == 0 && W == 0) {
	  return 0;
     }

     if (NE != 0 && NE < minChamp) {
	  minChamp = NE;
     }

     if (N != 0 && N < minChamp) {
	  minChamp = N;
     }

     if (NW != 0 && NW < minChamp) {
	  minChamp = NW;
     }

     if (W != 0 && W < minChamp) {
	  minChamp = W;
     }

     if (minChamp == UCHAR_MAX) {
          cout << "Warning: Couldn't find minimum neighbor value" << endl;
          minChamp = 0;
     }

     return minChamp;
}


void draw_on_label(cv::Mat &img, int r, int c)
{
     cv::Mat temp = img.clone();
     cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
     cv::applyColorMap(temp,temp,cv::COLORMAP_JET);
     //cv::circle(temp, cv::Point(c,r), 3, cv::Scalar(0,255,0), 1, 8, 0);
     temp.at<cv::Vec3b>(r,c) = cv::Vec3b(100,100,100);
     cv::resize(temp, temp, cv::Size(0,0),5,5,cv::INTER_NEAREST);
     cv::imshow("label", temp);
     cv::waitKey(0);
}

void new_label(cv::Mat &img, int r, int c)
{
     cv::Mat temp = img.clone();
     cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
     cv::applyColorMap(temp,temp,cv::COLORMAP_JET);
     cv::circle(temp, cv::Point(c,r), 7, cv::Scalar(0,255,0), 1, 8, 0);
     temp.at<cv::Vec3b>(r,c) = cv::Vec3b(0,255,0);
     cv::resize(temp, temp, cv::Size(0,0),5,5,cv::INTER_NEAREST);
     cv::imshow("new label!", temp);
     cv::waitKey(0);
}

void BlobProcess::labelNeighbors(cv::Mat &img, std::vector<uchar> &labelTable,
                                 uchar label, int i, int j)
{
     uchar value;

     // For each valid neighbor, update it's previous labelTable label with the
     // current label

     // North East
     value = valueAt(img,i-1,j+1);
     if (value != 0 && labelTable[label] < labelTable[value]) {
	  labelTable[value] = labelTable[label];
          img.at<uchar>(i-1,j+1) = labelTable[label];
     }

     // North
     value = valueAt(img,i-1,j);
     if (value != 0 && labelTable[label] < labelTable[value]) {
	  labelTable[value] = labelTable[label];
          img.at<uchar>(i-1,j) = labelTable[label];
     }

     // North West
     value = valueAt(img,i-1,j-1);
     if (value != 0 && labelTable[label] < labelTable[value]) {
	  labelTable[value] = labelTable[label];
          img.at<uchar>(i-1,j-1) = labelTable[label];
     }

     // West
     value = valueAt(img,i,j-1);
     if (value != 0 && labelTable[label] < labelTable[value]) {
	  labelTable[value] = labelTable[label];
          img.at<uchar>(i,j-1) = labelTable[label];
     }
}

void BlobProcess::find_blobs(cv::Mat &input,
                                    std::vector<wb::Blob> &blobs,
                                    int min_blob_size)
{
     blobs.clear();

     std::vector<uchar> labelTable;
     labelTable.push_back(0);

     cv::Mat img;
     input.copyTo(img);

     uchar label = 1;

     uchar NE, N, NW, W;

     for(int i = 0; i < img.rows; i++) {
          for(int j = 0; j < img.cols; j++) {
               if (img.at<uchar>(i,j) != 0) {
                    // The current pixel is not background

                    // Find the values of its neighbors
                    NE = valueAt(img,i-1,j+1);
                    N  = valueAt(img,i-1,j);
                    NW = valueAt(img,i-1,j-1);
                    W  = valueAt(img,i,j-1);

                    // Find the minimum value of its neighbors
                    uchar value = findMin(NE,N,NW,W);

                    if (value == 0) {
                         // There are no neighbors, uniquely label the current
                         // element
                         //cout << "New Label: " << (int)label << endl;
                         //new_label(input,i,j);
                         img.at<uchar>(i,j) = label;
                         labelTable.push_back(label);
                         label++;
                    } else {
                         // The pixel has neighbors with assigned
                         // values. Assign this pixel with the smallest label
                         // of its neighbors and label the neighbors
                         img.at<uchar>(i,j) = value;
                         labelNeighbors(img, labelTable, value, i, j);
                    }
               }
          }
     }

     // Second pass to fix connected components that have different labels
     std::map<int,wb::Blob> blobs_temp;
     for (int i = 0; i < img.rows; i++) {
          for (int j = 0; j < img.cols; j++) {
               if (img.at<uchar>(i,j) != 0) {
                    // Find the smallest equivalent label to the label of the
                    // current pixel
                    int id = labelTable[img.at<uchar>(i,j)];
                    //while (labelTable[id] != id) {
                    //     id = labelTable[id];
                    //}

                    img.at<uchar>(i,j) = id;

                    // If the ID is new, add it to the blob map
                    if (blobs_temp.count(id) == 0) {
                         wb::Blob blob(id);
                         blobs_temp[id] = blob;
                    }
                    wb::Point p;
                    p.set_position(cv::Point(j,i));
                    p.set_value(input.at<uchar>(i,j));
                    blobs_temp[id].add_point(p);
               }
          }
     }

     //// Print out labelTable
     //cout << "--------" << endl;
     //int loop = 0;
     //for (std::vector<uchar>::iterator it = labelTable.begin();
     //     it != labelTable.end(); it++) {
     //     cout << loop++ << ": " << (int)*it << endl;
     //}

     //cv::Mat temp;//(img, cv::Rect(0, img.rows/4, img.cols, img.rows - img.rows/4));
     //img.copyTo(temp);
     //cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
     //cv::applyColorMap(temp,temp,cv::COLORMAP_JET);
     //cv::resize(temp, temp, cv::Size(0,0),5,5,cv::INTER_NEAREST);
     //cv::imshow("color-blobs", temp);

     //////////////////////////////////////////////////////////////////////////
     // Remove small blobs / convert map to vector
     //////////////////////////////////////////////////////////////////////////
     std::map<int,wb::Blob>::iterator it_temp = blobs_temp.begin();
     int id = 0;
     for(; it_temp != blobs_temp.end(); it_temp++) {
          // Only copy over blobs that are of a minimum size
          if (it_temp->second.size() >= min_blob_size) {
               it_temp->second.set_id(id++);
               it_temp->second.init();
               blobs.push_back(it_temp->second);
          }
     }

     cv::Mat temp_img = input.clone();
     this->overlay_blobs(temp_img, temp_img, blobs);
     cv::imshow("Frame Blobs", temp_img);

}

int** array_to_matrix(int* m, int rows, int cols) {
     int i,j;
     int** r;
     r = (int**)calloc(rows,sizeof(int*));
     for(i=0;i<rows;i++)
     {
          r[i] = (int*)calloc(cols,sizeof(int));
          for(j=0;j<cols;j++)
               r[i][j] = m[i*cols+j];
     }
     return r;
}

void delete_matrix(int **array, int rows, int cols)
{
     for (int r = 0; r < rows; r++) {
          free (array[r]);
     }
     free (array);
}

void BlobProcess::build_tree(vertex_t &vertex,
                             std::vector<wb::Blob>::iterator it_meas,
                             std::vector<wb::Blob> &meas,
                             std::vector<wb::Blob> tracks,
                             int next_id)
{
     if (it_meas == meas.end()) {
          return;
     }

     // It could be a false positive
     vertex_t fp = boost::add_vertex(graph_);
     graph_[fp].mht_type = wb::Entity::fp;
     graph_[fp].set_id(0);
     edge_t fp_e; bool fp_b;
     boost::tie(fp_e,fp_b) = boost::add_edge(vertex,fp,graph_);

     build_tree(fp, it_meas+1, meas, tracks, next_id+1);

     // It could be a new track
     vertex_t nt = boost::add_vertex(graph_);
     graph_[nt].mht_type = wb::Entity::nt;
     graph_[nt].set_id(next_id);
     edge_t nt_e; bool nt_b;
     boost::tie(nt_e,nt_b) = boost::add_edge(vertex,nt,graph_);

     build_tree(nt, it_meas+1, meas, tracks, next_id+1);

     // Does the measurement fall within a track's gate?
     std::vector<wb::Blob>::iterator it = tracks.begin();
     while (it != tracks.end()) {
          if (cv::norm(it_meas->undistorted_centroid() - it->undistorted_centroid()) <= 2.0) {
               // The measurement falls within a track's gate.
               // Add it to the hyp tree, remove the track from the track's
               // vector and recurse.
               vertex_t dt = boost::add_vertex(graph_);
               graph_[dt] = *it;
               graph_[dt].mht_type = wb::Entity::dt;

               edge_t dt_e; bool dt_b;
               boost::tie(dt_e,dt_b) = boost::add_edge(vertex,dt,graph_);

               // Remove the current track, since it was used
               it = tracks.erase(it);
               // recurse
               build_tree(dt, it_meas+1, meas, tracks, next_id+1);
          } else {
               it++;
          }
     }
     return;
}

void BlobProcess::assign_mht(std::vector<wb::Blob> &meas,
                             std::vector<wb::Blob> &tracks,
                             std::vector<wb::Blob> &fused)
{
     //vertex_t node = boost::add_vertex(graph_);
     //graph_[node].set_id(4);
     //
     //wb::Blob b;
     //graph_[node] = b;

     // Each measurement can be:
     // 1. A New Track
     // 2. A Detected Track
     // 3. A False Positive Track

     vertex_t root = boost::add_vertex(graph_);
     graph_[root].set_id(-2);
     int next_id = 3;

     std::vector<wb::Blob>::iterator it_meas = meas.begin();
     std::vector<wb::Blob>::iterator it_tracks = tracks.begin();
     build_tree(root, it_meas, meas, tracks, next_id);

     // write graph to console
     cout << "\n-- graphviz output START --" << endl;
     boost::write_graphviz(std::cout, graph_,
                           boost::make_label_writer(boost::get(&wb::Blob::id_, graph_)));
     cout << "\n------------" << endl;

     std::ofstream dotfile ("/home/syllogismrxs/test.dot");
     boost::write_graphviz(dotfile, graph_,
                           boost::make_label_writer(boost::get(&wb::Blob::id_, graph_)));

}

void BlobProcess::assign_hungarian(std::vector<wb::Blob> &meas,
                                   std::vector<wb::Blob> &tracks,
                                   std::vector<wb::Blob> &fused)
{
     //////////////////////////////////////////////////////////////////////////
     // Use the Hungarian Method to match new blob measurements with previous
     // blob tracks
     //////////////////////////////////////////////////////////////////////////

     // Create cost matrix using the euclidean distance between previous and
     // current blob centroids
     int meas_count = meas.size();
     int tracks_count = tracks.size();

     // Determine max of meas_count and tracks
     int rows = -1, cols = -1;

     int r_start = 0, r_end = 0;
     int c_start = 0, c_end = 0;

     if (meas_count == tracks_count) {
          // Equal number of tracks and measurements
          rows = cols = meas_count;
     } else if (meas_count > tracks_count) {
          // More measurements than tracks
          rows = cols = meas_count;

          r_start = 0;
          r_end = rows;
          c_start = tracks_count;
          c_end = meas_count;
     } else {
          // More tracks than measurements
          rows = cols = tracks_count;

          r_start = meas_count;
          r_end = tracks_count;
          c_start = 0;
          c_end = cols;
     }

     int * cost = new int[rows*cols];

     // New blob measurements are along the Y-axis (left hand side)
     // Old Blob tracks are along x-axis (top-side)
     std::vector<wb::Blob>::iterator it = meas.begin();
     int r = 0;
     int max_cost = -1e3;
     for(; it != meas.end(); it++) {
          std::vector<wb::Blob>::iterator it_prev = tracks.begin();
          int c = 0;
          for (; it_prev != tracks.end(); it_prev++) {
               cv::Point p1 = it->estimated_centroid(); //it->centroid();
               cv::Point p2 = it_prev->estimated_centroid(); //it_prev->centroid();
               int dist = round(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));

               //int p1_size = it->size();
               //int p2_size = it_prev->size();
               //int size_diff = pow(p1_size - p2_size,2);

               //if (dist < 0) {
               //     dist = INT_MAX;
               //}
               //printf("%d\n",dist);
               //int curr_cost = 0.6 * dist + 0.4 * size_diff;

               int curr_cost = dist;
               if (curr_cost > 1000) {
                    curr_cost = INT_MAX;
               }

               cost[r*cols + c] = curr_cost;

               if (curr_cost > max_cost) {
                    max_cost = curr_cost;
               }

               c++;
          }
          r++;
     }

     // If necessary, pad cost matrix with appropriate number of dummy rows or
     // columns
     for (int r = r_start; r < r_end; r++) {
          for (int c = c_start; c < c_end; c++) {
               cost[r*cols + c] = max_cost;
          }
     }

     int** m = array_to_matrix(cost,rows, cols);
     delete[] cost;

     hungarian_problem_t p;
     //int matrix_size = hungarian_init(&p, m, rows, cols, HUNGARIAN_MODE_MINIMIZE_COST);
     hungarian_init(&p, m, rows, cols, HUNGARIAN_MODE_MINIMIZE_COST);

     //fprintf(stderr, "cost-matrix:");
     //hungarian_print_costmatrix(&p);

     hungarian_solve(&p);

     // Get assignment matrix;
     int * assignment = new int[rows*cols];
     hungarian_get_assignment(&p, assignment);

     new_tracks_.clear();
     // Use the assignment to update the old tracks with new blob measurement
     r = 0;
     it = meas.begin();
     for(int r = 0; r < rows; r++) {
          std::vector<wb::Blob>::iterator it_prev = tracks.begin();
          for (int c = 0; c < cols; c++) {
               if (assignment[r*cols + c] == 1) {
                    if (r < meas_count && c < tracks_count) {

                         cv::Point p1 = it->estimated_centroid();//it->centroid();
                         cv::Point p2 = it_prev->estimated_centroid(); //it->it_prev->centroid();
                         double dist = round(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));

                         if (dist > 100) { // needs to be based off of covariance matrix
                              // TOO MUCH OF A JUMP IN POSITION
                              // Probably a missed track
                              it_prev->missed_track();
                              fused.push_back(*it_prev);

                              // And a new track
                              it->new_track(next_available_id());
                              fused.push_back(*it);

                              new_tracks_.push_back(*it);

                         } else {
                              // Found an assignment. Update the new measurement
                              // with the track ID and age of older track. Add
                              // to fused vector
                              it->matched_track(*it_prev);
                              fused.push_back(*it);
                         }
                    } else if (r >= meas_count) {
                         it_prev->missed_track();
                         fused.push_back(*it_prev);
                    } else if (c >= tracks_count) {
                         // Possible new track
                         it->new_track(next_available_id());
                         fused.push_back(*it);

                         new_tracks_.push_back(*it);
                    }
                    break; // There is only one assignment per row
               }
               if (c < tracks_count-1) {
                    it_prev++;
               }
          }
          if (r < meas_count-1) {
               it++;
          }
     }

     hungarian_free(&p);
     free(m);
     delete[] assignment;
}

int BlobProcess::process_frame(cv::Mat &input, cv::Mat &original, int thresh)
{    
     std::vector<wb::Blob> new_blobs;
     this->find_blobs(input, new_blobs, min_blob_size_);

     //cv::Mat out;
     //double eps = 10;
     //std::vector<wb::Blob> blob_clusters;
     //this->cluster_blobs(input, out, new_blobs, blob_clusters, 3, eps);
     
     //blobs_ = blob_clusters;

     //cv::Mat cluster_img;
     //this->overlay_blobs(input,cluster_img);
     //overlay(input, cluster_img, RECTS | IDS);     
     //cv::imshow("Clustered",cluster_img);
     
     // First match blobs based on overlapping bounding boxes
     //////////////////////////////////////////////////////////////////////////
     // Run Kalman filter update on blobs from previous iteration
     //////////////////////////////////////////////////////////////////////////
     std::vector<wb::Blob>::iterator it = prev_blobs_.begin();
     for(; it != prev_blobs_.end(); it++) {
          if (it->is_tracked()) {
               it->predict_tracker();
          }
     }

     blobs_.clear();

     // This has some bug that crashes with too many blobs
     this->assign_hungarian(new_blobs, prev_blobs_, blobs_);
     
     blob_maintenance();

     prev_blobs_.clear();
     prev_blobs_ = blobs_;

     return blobs_.size();
}

void BlobProcess::blob_maintenance()
{
     short_lived_.clear();

     // cull dead
     // Also, keep track of tracks that are short lived
     std::vector<wb::Blob>::iterator it = blobs_.begin();
     while(it != blobs_.end()) {
          it->set_matched(false);
          it->update_trail();
          
          if (it->is_dead()) {
               if (it->age() < 5) {
                    short_lived_.push_back(*it);
               }
               it = blobs_.erase(it);
          } else {               
               it++;
          }          
     }
}

bool BlobProcess::consolidate_tracks(cv::Mat &in, cv::Mat &out)
{
     out = in.clone();

     // Consolidate blob tracks that have overlapping rectangles
     // The track ID from the oldest track is used.

     bool found_overlap = false;

     std::map<int, std::list<wb::Blob*> > matches;
     int next_id = 1;

     std::vector<wb::Blob>::iterator it1 = blobs_.begin();
     for(; it1 != blobs_.end(); it1++) {
          std::vector<wb::Blob>::iterator it2 = blobs_.begin();
          for (; it2 != blobs_.end(); it2++) {

               if (it1->id() == it2->id()) {
                    // If it's the same ID, move on to next track
                    continue;
               }

               if (BoundingBox::overlap(it1->bbox(), it2->bbox())) {
                    found_overlap = true;
                    //cout << "Overlap" << endl;
                    if (it1->matched() && it2->matched()) {
                         if (it1->matched_id() < it2->matched()) {
                              // Move items from it2's list to it1's list
                              for(std::list<wb::Blob*>::iterator it = matches[it2->matched_id()].begin();
                                  it != matches[it2->matched_id()].end(); it++) {
                                   cout << "Moving " << (*it)->id() << " to " << it1->id() << "'s list. "<< endl;
                                   matches[it1->matched_id()].push_back(*it);
                              }
                              matches[it2->matched_id()].clear();

                         } else if (it2->matched_id() < it1->matched()) {
                              // Move items from it1's list to it2's list
                              for(std::list<wb::Blob*>::iterator it = matches[it1->matched_id()].begin();
                                  it != matches[it1->matched_id()].end(); it++) {
                                   matches[it2->matched_id()].push_back(*it);
                                   cout << "Moving " << (*it)->id() << " to " << it2->id() << "'s list. "<< endl;
                              }
                              matches[it1->matched_id()].clear();
                         }
                    } else if (it1->matched()) {
                         it2->set_matched(true);
                         it2->set_matched_id(it1->matched_id());
                         matches[it1->matched_id()].push_back(&(*it2));
                    } else if(it2->matched()) {
                         it1->set_matched(true);
                         it1->set_matched_id(it2->matched_id());
                         matches[it2->matched_id()].push_back(&(*it1));
                         //} else if (it1->id() < it2->id()) {
                    } else if (it1->age() > it2->age()) {
                         it1->set_matched(true);
                         it1->set_matched_id(next_id);
                         it2->set_matched(true);
                         it2->set_matched_id(next_id);

                         matches[next_id].push_back(&(*it1));
                         matches[next_id].push_back(&(*it2));
                         next_id++;

                    } else {
                         it1->set_matched(true);
                         it1->set_matched_id(next_id);
                         it2->set_matched(true);
                         it2->set_matched_id(next_id);

                         matches[next_id].push_back(&(*it1));
                         matches[next_id].push_back(&(*it2));
                         next_id++;
                    }
               }
          }
     }

     std::vector<wb::Blob> temp_blobs;
     for (std::map<int, std::list<wb::Blob*> >::iterator it = matches.begin();
          it != matches.end(); it++) {
          int max_age = INT_MIN;
          wb::Blob *champ_blob = NULL;
          wb::Blob blob;
          for(std::list<wb::Blob*>::iterator it_blob = it->second.begin();
              it_blob != it->second.end(); it_blob++) {

               if ((*it_blob)->age() > max_age) {
                    max_age = (*it_blob)->age();
                    champ_blob = *it_blob;
               }

               blob.add_points((*it_blob)->points());
          }

          if (champ_blob != NULL) {
               blob.copy_track_info(*champ_blob);
               blob.compute_metrics();
               blob.detected_track();

               temp_blobs.push_back(blob);
          } else {
               cout << "Error: Couldn't find champ blob" << endl;
          }
     }

     // Add the blobs that had no overlap
     for(std::vector<wb::Blob>::iterator it = blobs_.begin();
         it != blobs_.end(); it++) {
          if (!it->matched()) {
               temp_blobs.push_back(*it);
          }
     }

     blobs_.clear();
     blobs_ = temp_blobs;

     prev_blobs_.clear();
     prev_blobs_ = temp_blobs;

     this->overlay_blobs(out, out);
     this->overlay_tracks(out, out);

     return found_overlap;
}

void region_query(rtree_t &rtree, double eps, wb::Blob *b,
                  std::list<wb::Blob*> &neighbors)
{
     std::list<value> returned_values;
#if 0
     // Find nearest # of rectangles
     point sought = point(b->centroid().x, b->centroid().y);
     rtree.query(bgi::nearest(sought, 20), std::back_inserter(returned_values));
#else
     // Find all elements within the rtree bounds
     box bounds = rtree.bounds();
     rtree.query(bgi::intersects(bounds), std::back_inserter(returned_values));
#endif
     //cout << "Blob ID: " << b->id() << " has neighbors... (" << returned_values.size() << ")" << endl;
     for (std::list<value>::iterator it = returned_values.begin(); it != returned_values.end(); it++) {
          double dist = BoundingBox::distance(b->bbox(), it->second->bbox());
          //cout << "\t" << it->second->id() << " : " << dist;
          if (dist <= eps) {
               //cout << " - neighbor" << endl;
               neighbors.push_back(it->second);
          } else {
               //cout << " - too far" << endl;
               continue;
          }
     }
     //cout << "Near Neighbors: " << neighbors.size() << endl;
}

bool BlobProcess::cluster_blobs(cv::Mat &in, cv::Mat &out,
                                std::vector<wb::Blob> &data_blobs,
                                std::vector<wb::Blob> &clusters,
                                unsigned int MinPts,
                                double eps)
{
     out = in.clone();

     if (data_blobs.size() <= 0) {
          return false;
     }

     // Cluster blobs that are within a specified distance

     // Build the R* Tree and blobs* list
     std::list<wb::Blob*> blobs;
     rtree_t rtree(bgi::dynamic_rstar(data_blobs.size()));
     for (std::vector<wb::Blob>::iterator it = data_blobs.begin(); it != data_blobs.end();
          it++ ) {
          // create a box
          box b(point(it->bbox().xmin(), it->bbox().ymin()), point(it->bbox().xmax(), it->bbox().ymax()));
          // insert new value
          rtree.insert(std::make_pair(b, &(*it)));
          blobs.push_back(&(*it));
     }

     // 0 - unset, -1 is noise, 1+ are cluster ids.
     std::map<int, std::list<wb::Blob*> > clusters_;

     int UNSET_ID = 0;
     int NOISE_ID = -1;
     int cluster_id = 1;
     // for each point P in dataset D
     for (std::list<wb::Blob*>::iterator it1 = blobs.begin(); it1 != blobs.end();
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
               //cout << "NOISE - Blob ID: " << (*it1)->id() << endl;
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

     clusters.clear();
     
     // Fill in the clusters output
     for (std::map<int, std::list<wb::Blob*> >::iterator it1 = clusters_.begin();
          it1 != clusters_.end(); it1++) {

          if (it1->first == NOISE_ID) {
               //cout << "NOISE IDS: " << endl; 
               
               //If the cluster defined as "Noise" is of a certain size, allow
               // it to be its own cluster, use the next cluster_id               
               //if (it1->second.size() > 20) {
               //it1->second->set_id(cluster_id++);
               //clusters.push_back((it1->second));
               //}
               
               // Loop through each Blob in the NOISE_ID "cluster", determine
               // if the Blob has a certain number of points, if it does, add it
               // as its own cluster
               for (std::list<wb::Blob*>::iterator it2 = it1->second.begin();
                    it2 != it1->second.end(); it2++) {
                    if ((*it2)->size() > 20) {
                         (*it2)->set_id(cluster_id++);
                         clusters.push_back(*(*it2));
                    }
               }
                    
          } else if (it1->first == UNSET_ID) {
               cout << "WARNING: UNSET_ID FOUND: " << endl;
          } else {
               //cout << "Cluster: " << it1->first << endl;
               // Reduce the multiple blobs to a single blob
               // Compute bounding box and centroid    
               wb::Blob b;
               b.set_id(it1->first);
               //int sum_x = 0, sum_y = 0, sum_value = 0;          
               int xmin = INT_MAX, ymin = INT_MAX, xmax = INT_MIN, ymax = INT_MIN;
               for (std::list<wb::Blob*>::iterator it2 = it1->second.begin();
                    it2 != it1->second.end(); it2++) {
                    //cout << "\tBlob: " << (*it2)->id() << endl;
                    if ((*it2)->bbox().xmin() < xmin) {
                         xmin = (*it2)->bbox().xmin();
                    }
                    if ((*it2)->bbox().ymin() < ymin) {
                         ymin = (*it2)->bbox().ymin();
                    }
                    if ((*it2)->bbox().xmax() > xmax) {
                         xmax = (*it2)->bbox().xmax();
                    }
                    if ((*it2)->bbox().ymax() > ymax) {
                         ymax = (*it2)->bbox().ymax();
                    }               
               }
               b.set_centroid(cv::Point(cvRound((xmax-xmin)/2.0),cvRound((ymax-ymin)/2.0)));
               b.set_bbox(BoundingBox(xmin,xmax,ymin,ymax));
               clusters.push_back(b);
          }
     }
     return true;
}

void BlobProcess::overlay_blobs(cv::Mat &src, cv::Mat &dst,
                                std::vector<wb::Blob> & blobs)
{
     cv::Mat color;
     cv::cvtColor(src, color, CV_GRAY2BGR);
     dst = color;

     std::vector<wb::Blob>::iterator it = blobs.begin();
     for(; it != blobs.end(); it++) {

          cv::Vec3b point_color = cv::Vec3b(20,255,57);
          if (it->occluded()) {
               point_color = cv::Vec3b(0,0,0);
          }

          // Draw all blob points in the image
          std::vector<wb::Point> points = it->points();
          std::vector<wb::Point>::iterator it_points = points.begin();
          for(; it_points != points.end(); it_points++) {
               dst.at<cv::Vec3b>(it_points->y(), it_points->x()) = point_color;
          }

          cv::Point centroid_point = it->centroid();
          cv::Rect rect = it->bbox().rectangle();

          std::ostringstream convert;
          convert << it->id();
          const std::string& text = convert.str();

          cv::circle(dst, centroid_point, 1, cv::Scalar(255,255,255), -1, 8, 0);
          cv::rectangle(dst, rect, cv::Scalar(255,255,255), 1, 8, 0);
          cv::putText(dst, text, cv::Point(rect.x-3,rect.y-3), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 1, 8, false);
     }
}

void BlobProcess::overlay_blobs(cv::Mat &src, cv::Mat &dst)
{
     this->overlay_blobs(src, dst, blobs_);
}

void BlobProcess::overlay_short_lived(cv::Mat &src, cv::Mat &dst)
{
     cv::Mat color;
     if (src.channels() == 1) {
          cv::cvtColor(src, color, CV_GRAY2BGR);
     } else {
          color = src;
     }
     dst = color;

     for(std::vector<wb::Blob>::iterator it = short_lived_.begin();
         it != short_lived_.end(); it++) {

          cv::Rect rect = it->bbox().rectangle();
          cv::rectangle(dst, rect, cv::Scalar(0,255,0), -1, 8, 0);
     }

     for(std::vector<wb::Blob>::iterator it = new_tracks_.begin();
         it != new_tracks_.end(); it++) {

          cv::Rect rect = it->bbox().rectangle();
          cv::rectangle(dst, rect, cv::Scalar(255,0,0), -1, 8, 0);
     }
}

void BlobProcess::overlay(cv::Mat &src, cv::Mat &dst, OverlayFlags_t flags)
{
     cv::Mat color;
     if (src.channels() == 1) {
          cv::cvtColor(src, color, CV_GRAY2BGR);
     } else {
          color = src;
     }
     dst = color;

     std::vector<wb::Blob>::iterator it = blobs_.begin();
     for(; it != blobs_.end(); it++) {

          if (flags & BLOBS) {
               cv::Vec3b point_color = cv::Vec3b(20,255,57);
               if (it->occluded()) {
                    point_color = cv::Vec3b(0,0,0);
               }

               // Draw all blob points in the image
               std::vector<wb::Point> points = it->points();
               std::vector<wb::Point>::iterator it_points = points.begin();
               for(; it_points != points.end(); it_points++) {
                    dst.at<cv::Vec3b>(it_points->y(), it_points->x()) = point_color;
               }
          }

          //cv::Point centroid_point = it->centroid();
          //cv::circle(dst, centroid_point, 1, cv::Scalar(255,255,255), -1, 8, 0);

          cv::Rect rect = it->bbox().rectangle();

          if (flags & RECTS) {               
               cv::rectangle(dst, rect, cv::Scalar(255,255,255), 1, 8, 0);
          }

          if (flags & IDS) {
               std::ostringstream convert;
               convert << it->id();
               const std::string& text = convert.str();
               cv::putText(dst, text, cv::Point(rect.x-3,rect.y-3), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 2, 8, false);
          }

          if (flags & TRACKS) {
               cv::Point est_centroid = it->estimated_centroid();
               wb::drawCross(dst, est_centroid, cv::Scalar(255,255,255), 5);
          }

          if (flags & ERR_ELLIPSE) {
               Ellipse ell = it->error_ellipse(0,2,0.95);
               cv::Point center = ell.center();
               cv::Size axes(ell.axes()(0), ell.axes()(1));
               cv::ellipse(dst, center, axes, ell.angle(), 0, 360, cv::Scalar(0,80,80), 1, 8, 0);
          }
     }
}

void BlobProcess::overlay_tracks(cv::Mat &src, cv::Mat &dst)
{
     cv::Mat color;
     //cv::cvtColor(src, color, CV_GRAY2BGR);

     dst = src.clone();
     std::vector<wb::Blob>::iterator it = blobs_.begin();
     for (; it != blobs_.end(); it++) {
          if (it->is_tracked()) {
               cv::Point est_centroid = it->estimated_centroid();
               //cv::Rect rect = (*it)->rectangle();

               std::ostringstream convert;
               convert << it->id();
               //const std::string& text = convert.str();

               wb::drawCross(dst, est_centroid, cv::Scalar(255,255,255), 5);
          }
     }
}

bool BlobProcess::displace_detect(int history, double distance)
{
     // Find "confirmed" track that has moved more than (X) distance
     // Label them as "divers"     
     
     for (std::vector<wb::Blob>::iterator it = blobs_.begin(); 
          it != blobs_.end(); it++) {
          if (it->is_tracked()) {               
               if (wb::distance(it->centroid(),it->trail_history(history)) > distance) { //30, 15
                    it->set_type(wb::Entity::Diver);                    
               } else {
                    it->set_type(wb::Entity::Unknown);
               }
          }
     }          
     return true;
}

void BlobProcess::blobs_to_entities(std::vector<wb::Entity> &ents)
{     
     for (std::vector<wb::Blob>::iterator it = blobs_.begin(); 
          it != blobs_.end(); it++) {
          wb::Entity track;
          track.set_bbox(it->bbox());
          track.copy_track_info(*it);          
          track.set_type(it->type());
          track.set_centroid(it->centroid());
          track.set_estimated_centroid(it->estimated_centroid());

          ents.push_back(track);
     }
}
