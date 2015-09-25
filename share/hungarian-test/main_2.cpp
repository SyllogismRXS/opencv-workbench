#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv_workbench/track/hungarian.h>
#include <opencv_workbench/wb/Blob.h>
#include <opencv_workbench/wb/Entity.h>


using std::cout;
using std::endl;

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


int main() {

     std::map<int,wb::Blob> blobs_;
     std::map<int,wb::Blob> prev_blobs_;

     wb::Blob b;     
     int id;

     //////////////////////
     // Old Blob Tracks
     id = 1;
     b.set_id(id);
     b.set_centroid(cv::Point(1,1));     
     prev_blobs_[id] = b;

     id = 2;
     b.set_id(id);
     b.set_centroid(cv::Point(6,3));     
     prev_blobs_[id] = b;

     id = 3;
     b.set_id(id);
     b.set_centroid(cv::Point(2,4));     
     prev_blobs_[id] = b;

     ///////////////////////
     // New Blob Measurements
     // Old Blob Tracks
     id = 1;
     b.set_id(id);
     b.set_centroid(cv::Point(1,2));     
     blobs_[id] = b;

     id = 2;
     b.set_id(id);
     b.set_centroid(cv::Point(7,4));     
     blobs_[id] = b;

     id = 3;
     b.set_id(id);
     b.set_centroid(cv::Point(1,3));     
     blobs_[id] = b;
     
     ///////////////////////////////////////
     std::map<int,wb::Blob>::iterator it;

     //////////////////////////////////////////////////////////////////////////
     // Use the Hungarian Method to match new blob measurements with previous
     // blob tracks
     //////////////////////////////////////////////////////////////////////////

     // Create cost matrix using the euclidean distance between previous and 
     // current blob centroids
     int blob_count = blobs_.size();
     int prev_blob_count = prev_blobs_.size();

     // Determine max of blob_count and prev_blob_count
     int rows = -1, cols = -1;
          
     int r_start = 0, r_end = 0;
     int c_start = 0, c_end = 0;
     
     if (blob_count == prev_blob_count) {
          rows = cols = blob_count;          
     } else if (blob_count > prev_blob_count) {
          rows = cols = blob_count;
                    
          r_start = 0;
          r_end = rows;
          c_start = prev_blob_count;
          c_end = blob_count;
          
     } else {
          rows = cols = prev_blob_count;
          
          r_start = blob_count;
          r_end = prev_blob_count;
          c_start = 0;
          c_end = cols;
     }

     //cout << "r_start: " << r_start << endl;
     //cout << "r_end: " << r_end << endl;
     //cout << "c_start: " << c_start << endl;
     //cout << "c_end: " << c_end << endl;

     int * cost = new int[rows*cols];
          
     // New blob measurements are along the Y-axis (left hand side)
     // Old Blob tracks are along x-axis (top-side)
     it = blobs_.begin();
     int r = 0;
     int max_cost = -1e3;
     for(; it != blobs_.end(); it++) {          
          std::map<int,wb::Blob>::iterator it_prev = prev_blobs_.begin();
          int c = 0;
          for (; it_prev != prev_blobs_.end(); it_prev++) {
               cv::Point p1 = it->second.centroid();
               cv::Point p2 = it_prev->second.centroid();
               int dist = round(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
               
               cost[r*cols + c] = dist;

               if (dist > max_cost) {
                    max_cost = dist;
               }
               
               c++;
          }
          r++;
     }

     cout << "Max cost: " << max_cost << endl;

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
     int matrix_size = hungarian_init(&p, m, rows, cols, HUNGARIAN_MODE_MINIMIZE_COST);

     fprintf(stderr, "assignment matrix now has a size %d rows and %d columns.\n\n",  matrix_size, matrix_size);

     // some output
     fprintf(stderr, "cost-matrix:");
     hungarian_print_costmatrix(&p);
     
     hungarian_solve(&p);

     // some output 
     fprintf(stderr, "assignment:");
     hungarian_print_assignment(&p);

     // Get assignment matrix;
     int * assignment = new int[rows*cols];
     hungarian_get_assignment(&p, assignment);

     for (int r = 0; r < rows; r++) {
          for (int c = 0; c < cols; c++) {
               printf("%d ", assignment[r*cols + c]);
          }
          printf("\n");
     }
     
     hungarian_free(&p);
     free(m);          

     delete[] assignment;

     return 0;
}
