#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <opencv_workbench/track/hungarian.h>

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

     hungarian_problem_t p;     

     //int rows = 4;
     //int cols = 3;
     //int r[4*3] =  {  100, 1, 1, 
     //                       100, 2, 2, 
     //                       1, 0, 0, 
     //0, 2, 0 };
     
#define ROWS 3
#define COLS 3
     int r[ROWS*COLS] =  {  250, 400, 350, 
                            400, 600, 350, 
                            200, 400, 250
     };

//#define ROWS 4
//#define COLS 4
//     int r[ROWS*COLS] =  {  250, 400, 350, 600, 
//                            400, 600, 350, 600,
//                            200, 400, 250, 600,
//                            200, 500, 300, 600
//     };

     //std::vector<Point
     
     
     
     //int rows = 4;
     //int cols = 4;
     //int r[4*4] =  {  90,75,75,80,
     //                 35,85,55,65, 
     //                 125,95,90,105,
     //                 45,110,95,115
     //};

     //int rows = 2;
     //int cols = 3;
     //int r[2*3] =  {  10,2,3,
     //                 4,5,6
     //};

     
     int** m = array_to_matrix(r,ROWS,COLS);
     
     // initialize the gungarian_problem using the cost matrix
     int matrix_size = hungarian_init(&p, m, ROWS, COLS, HUNGARIAN_MODE_MINIMIZE_COST) ;

     fprintf(stderr, "assignment matrix now has a size %d rows and %d columns.\n\n",  matrix_size, matrix_size);

     // some output
     fprintf(stderr, "cost-matrix:");
     hungarian_print_costmatrix(&p);
     
     // solve the assignement problem
     hungarian_solve(&p);

     // some output 
     fprintf(stderr, "assignment:");
     hungarian_print_assignment(&p);

     // free used memory
     hungarian_free(&p);
     free(m);

     return 0;
}
