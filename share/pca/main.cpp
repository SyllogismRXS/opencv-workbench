#include <iostream>
#include <stdio.h>
#include <opencv_workbench/LARKS/LARKS.h>

using std::cout;
using std::endl;

cv::PCA compressPCA(cv::InputArray pcaset, int maxComponents,
                    const cv::Mat& testset, cv::OutputArray compressed)
{
     cv::PCA pca(pcaset, // pass the data
             cv::Mat(), // there is no pre-computed mean vector,
             // so let the PCA engine to compute it
             CV_PCA_DATA_AS_ROW, // indicate that the vectors
             // are stored as matrix rows
             // (use CV_PCA_DATA_AS_COL if the vectors are
             // the matrix columns)
             maxComponents // specify how many principal components to retain
          );
     // if there is no test data, just return the computed basis, ready-to-use
     if( !testset.data )
          return pca;
     CV_Assert( testset.cols == pcaset.getMat().cols );
     
     compressed.create(testset.rows, maxComponents, testset.type());
     
     cv::Mat reconstructed;
     for( int i = 0; i < testset.rows; i++ )
     {
          cv::Mat vec = testset.row(i), coeffs = compressed.getMat().row(i);
          // compress the vector, the result will be stored
          // in the i-th row of the output matrix
          pca.project(vec, coeffs);
          // and then reconstruct it
          pca.backProject(coeffs, reconstructed);
          // and measure the error
          printf("%d. diff = %g\n", i, norm(vec, reconstructed, cv::NORM_L2));
     }
     return pca;
}

int main(int argc, char *argv[])
{
     cout << "PCA Test" << endl;

     

     return 0;
}
