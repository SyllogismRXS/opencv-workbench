#include <iostream>

#include "MatrixTracker.h"

using std::cout;
using std::endl;

MatrixTracker::MatrixTracker()
{
     this->init(2,2);
}

void MatrixTracker::init(int rows, int cols)
{
     rows_ = rows;
     cols_ = cols;
     
     elems_.clear();
     for (int i = 0; i < rows_*cols_; i++) {
          elems_.push_back(ScalarTracker());
     }
}

void MatrixTracker::set_values(Eigen::MatrixXd &vals)
{
     std::vector<double> v;
     for (int r = 0; r < vals.rows(); r++) {
          for (int c = 0; c < vals.cols(); c++) {
               v.push_back(vals(r,c));
          }
     }
     this->set_values(v);
}

void MatrixTracker::set_values(std::vector<double> &vals)
{
     if (vals.size() != elems_.size()) {
          cout << "ERROR: Mismatched matrix sizes." << endl;
          return;
     }

     for (int i = 0; i < rows_*cols_; i++) {
          elems_[i].set_value(vals[i]);
     }
}

void MatrixTracker::predict()
{
     for (int i = 0; i < rows_*cols_; i++) {
          elems_[i].predict();
     }
}

Eigen::MatrixXd MatrixTracker::values()
{
     Eigen::MatrixXd m(rows_,cols_);
     int i = 0;
     for (int r = 0; r < rows_; r++) {
          for (int c = 0; c < cols_; c++) {
               m(r,c) = elems_[i++].value();              
          }
     }
     return m;
}
