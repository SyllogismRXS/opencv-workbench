#include <iostream>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/track/filters.h>
#include <opencv_workbench/track/ParticleFilter.h>

using namespace std;

#define ENABLE_PF 0

uchar valueAt(cv::Mat &img, int row, int col)
{
     if (col >= 0 && col < img.cols) {
	  if (row >= 0 && row < img.rows) {
	       return img.at<uchar>(row,col);
	  }
     } 
     return 0;
}
     
uchar findMin(uchar NE, uchar N, uchar NW, uchar W)
{
     int minChamp = 9999;

     if (NE == 0 && N == 0 && NW == 0 && W == 0) {
	  return 0;
     }

     if(NE != 0 && NE < minChamp) {
	  minChamp = NE;
     }

     if(N != 0 && N < minChamp) {
	  minChamp = N;
     }

     if(NW != 0 && NW < minChamp) {
	  minChamp = NW;
     }

     if(W != 0 && W < minChamp) {
	  minChamp = W;
     }

     return minChamp;
}

void labelNeighbors(cv::Mat &img, std::vector<uchar> &labelTable, uchar label, int i, int j)
{
     uchar value;

     value = valueAt(img,i-1,j+1);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i-1,j+1) = label;
     }

     value = valueAt(img,i-1,j);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i-1,j) = label;
     }

     value = valueAt(img,i-1,j-1);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i-1,j-1) = label;
     }

     value = valueAt(img,i,j-1);
     if (value != 0) {
	  labelTable[value] = label;
	  img.at<uchar>(i,j-1) = label;
     }		    
}
     

namespace syllo {

     void fill_line(std::string ch) 
     {
          if (ch.length() > 1) {
               ch = "=";
          }
          
          for (int i = 0; i < 80; i++) {
               cout << ch;
          }
          cout << endl;
     }
     
     int str2int(std::string str)
     {
          int num;
          if ( ! (std::istringstream(str) >> num) ) num = 0;
          return num;
     }

     std::string int2str(int x)
     {
          std::ostringstream convert;   // stream used for the conversion
          convert << x;      // insert the textual representation of 'Number' in the characters in the stream
          return convert.str();          
     }

     std::string double2str(double x)
     {
          std::ostringstream convert;   // stream used for the conversion
          convert << x;      // insert the textual representation of 'Number' in the characters in the stream
          return convert.str();          
     }
     
     int templateDetect(const cv::Mat &inputImg, const cv::Mat &templImg, cv::Mat &outputImg)
     {
	  // Copy original input to output image
	  inputImg.copyTo(outputImg);

	  cv::Mat matchImg;
	  int match_cols = inputImg.cols - templImg.cols + 1; 
	  int match_rows = inputImg.rows - templImg.rows + 1;
	  matchImg.create( match_cols, match_rows, CV_32FC1 );
    
	  // Do the Matching and Normalize
	  int matchMethod = CV_TM_CCOEFF;
	  cv::matchTemplate( inputImg, templImg, matchImg, matchMethod );
	  cv::normalize( matchImg, matchImg, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
     
	  // Localizing the best match with minMaxLoc
	  double minVal; 
	  double maxVal; 
	  cv::Point minLoc; 
	  cv::Point maxLoc; 
	  cv::Point matchLoc;
     
	  cv::minMaxLoc( matchImg, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

	  // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
	  if( matchMethod == CV_TM_SQDIFF || matchMethod == CV_TM_SQDIFF_NORMED ) { 
	       matchLoc = minLoc; 
	  } else { 
	       matchLoc = maxLoc; 
	  }

	  cv::Rect roi = cv::Rect(matchLoc.x-10, matchLoc.y-10, templImg.cols+20, templImg.rows+20);
	  cv::Mat subImg(outputImg, roi);
     
	  cv::cvtColor(subImg, subImg, CV_BGRA2BGR);
     
	  //imshow("match,",subImg);
	  //record << subImg;
     
	  // Show me what you got
	  rectangle( outputImg, matchLoc, cv::Point( matchLoc.x + templImg.cols , matchLoc.y + templImg.rows ), cv::Scalar::all(0), 2, 8 ); 
	  //rectangle( result, matchLoc, Point( matchLoc.x + templImg.cols , matchLoc.y + templImg.rows ), Scalar::all(0), 2, 8, 0 );
          return 0;
     }

     /*
       Grayscale:
       White - 255
       Black - 0
     */
     int DetectBlobs(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Blob> &blobs)
     {
	  std::vector<uchar> labelTable;
	  labelTable.push_back(0);

	  cv::Mat img;
	  input.copyTo(img);
     
	  uchar label = 1;

	  uchar NE, N, NW, W;

	  for( int i = 0; i < img.rows; ++i) {
	       for( int j = 0; j < img.cols; ++j ) {
		    if (img.at<uchar>(i,j) != 0) {
			 NE = valueAt(img,i-1,j+1);
			 N  = valueAt(img,i-1,j);
			 NW = valueAt(img,i-1,j-1);
			 W  = valueAt(img,i,j-1);
	       	    
			 uchar value = findMin(NE,N,NW,W);

			 if (value == 0) {
			      img.at<uchar>(i,j) = label;
			      labelTable.push_back(label);
			      label++;
			 } else {
			      img.at<uchar>(i,j) = value;
			      labelNeighbors(img, labelTable, value, i, j);
			 }
		    }
	       }
	  }
          
	  // Second pass to fix connected components that have different labels
	  for( int i = 0; i < img.rows; ++i) {
	       for( int j = 0; j < img.cols; ++j ) {
		    if (img.at<uchar>(i,j) != 0) {
			 int id = labelTable[img.at<uchar>(i,j)];
			 img.at<uchar>(i,j) = id;

			 // If the ID is new, add it to the blob map
			 if (blobs.count(id) == 0) {
			      syllo::Blob blob(id);
			      blobs[id] = blob;
			 }
			 blobs[id].incSize();
		    }
	       }
	  }

	  //std::map<int,syllo::Blob>::iterator it;
	  //for(it=blobs.begin();it!=blobs.end();it++)
	  //{
	  //     printf("id: %d \t size:%d\n",it->first,it->second.getSize());
	  //}

	  output = img;
	  return blobs.size();
     }


     cv::Point calcCentroid(cv::Mat &img, uchar value)
     {
	  double xSum = 0;
	  double ySum = 0;

	  double xAvg = 0;
	  double yAvg = 0;

	  int channels = img.channels();
	  int nRows = img.rows;
	  int nCols = img.cols * channels;
	  if (img.isContinuous()) {
	       nCols *= nRows;
	       nRows = 1; 
	  }
     
	  int i,j;
	  int count = 0;
	  uchar *p1;
	  for( i = 0; i < nRows; ++i) {
	       p1 = img.ptr<uchar>(i);
	       for ( j = 0; j < nCols; ++j) {
		    if ( p1[j] == value) {
			 xSum += ( j % img.cols);
			 ySum += floor(j/img.cols);
			 count++;
		    }
	       }
	  }

	  cv::Point centroid;
	  if (count > 0) {
	       xAvg = (double)xSum / (double)count;
	       yAvg = (double)ySum / (double)count;
	       centroid = cv::Point(xAvg, yAvg);
	  } else {
	       centroid = cv::Point(-1,-1);
	  }

	  return centroid;
     }

     int CentroidsOfBlobs(cv::Mat &img, std::map<int, syllo::Blob> &blobs)
     {
	  std::map<int, syllo::Blob>::iterator it;
	  for (it = blobs.begin(); it != blobs.end(); it++) {
	       cv::Point centroid = calcCentroid(img, it->first);
	       it->second.setCentroid(centroid);
	  }
	  return 0;
     }

     cv::Point GetLargestBlob(std::map<int, syllo::Blob> &blobs) 
     {
	  int champ = 0;
	  int champIndex = 0;
	  std::map<int,syllo::Blob>::iterator it;
	  int count = 0;
	  for (it = blobs.begin() ; it != blobs.end() ; it++) {
	       if (it->second.getSize() > champ) {
		    champ = it->second.getSize();
		    champIndex = it->first;
	       }
	       count++;
	  }
	  return blobs[champIndex].getCentroid();
     }

     // 15 times faster than the classical float sqrt. 
     // Reasonably accurate up to root(32500)
     // Source: http://supp.iar.com/FilesPublic/SUPPORT/000419/AN-G-002.pdf
     unsigned int root(unsigned int x){
	  unsigned int a,b;
	  b     = x;
	  a = x = 0x3f;
	  x     = b/x;
	  a = x = (x+a)>>1;
	  x     = b/x;
	  a = x = (x+a)>>1;
	  x     = b/x;
	  x     = (x+a)>>1;
	  return(x);  
     }

     unsigned int dist(const cv::Point &pt0, const cv::Point &pt1)
     {
	  return root( pow(pt0.x-pt1.x,2) + pow(pt0.y-pt1.y,2) );
     }

     
     int BlobMatch(std::map<int,syllo::Blob> &prevBlobs, std::map<int,syllo::Blob> &blobs, std::map<int,syllo::Blob> &newBlobs, int curTime) 
     {
          // If there aren't any previous blobs, just copy the current blobs
          // over to the new blobs.
	  if (prevBlobs.size() == 0) {
	       std::map<int,syllo::Blob>::iterator it;
	       for (it = blobs.begin() ; it != blobs.end(); it++) {
		    newBlobs[it->first] = blobs[it->first];
	       }
	       return 0;
	  }

          // Loop through the previous blobs and match the current blobs with
          // the previous blobs based on absolute distance.
	  std::map<int,syllo::Blob>::iterator itPrev;
	  for (itPrev = prevBlobs.begin() ; itPrev != prevBlobs.end(); itPrev++) {
	       bool matchFound = false;
	       std::map<int,syllo::Blob>::iterator it;
	       for (it = blobs.begin() ; it != blobs.end(); it++) {
		    
                    // Is the distance within a specified threshold?
		    if ( abs(dist(itPrev->second.getCentroid(), it->second.getCentroid())) < 30) {
                         // Match is found, set new blob's ID to the previous blob's ID
			 // Add blob to new blobs map   
			 itPrev->second.setSize(it->second.getSize());
			 itPrev->second.setCentroid(it->second.getCentroid());
			 //itPrev->second.incAge();
			 itPrev->second.resetAge();
			 //itPrev->second.updateVelocity();
			 newBlobs[itPrev->second.getID()] = itPrev->second;
			 blobs.erase(it->first);

			 matchFound = true;
			 break;
		    }
	       }
               
	       if (!matchFound) {
                    // If a match wasn't found, decrement the age.
		    itPrev->second.decAge();
		    if (itPrev->second.getAge() > 0) {
                         // As long as the age is greater than zero, copy the
                         // previous blob into the new blobs
			 newBlobs[itPrev->second.getID()] = itPrev->second;
		    }
		    
                    //// Could not find blob's match
		    //// Find the next available slot in the map
		    //int newID = 1;
		    //while (prevBlobs.count(newID) == 1 || newBlobs.count(newID) == 1) {
		    //	 newID++;
		    //}
		    //it->second.setID(newID);
		    //newBlobs[newID] = it->second;
	       }
	  }
	  
          // Previously, we deleted all the current blobs that matched previous
          // blobs. Loop through all of the current blobs that weren't matched
          // and find the next available ID for the new blobs. Copy the current
          // newly detected blob into the newBlobs map.
	  std::map<int,syllo::Blob>::iterator it;
	  for (it = blobs.begin() ; it != blobs.end(); it++) {
	       int newID = 1;
	       //while (prevBlobs.count(newID) == 1 || newBlobs.count(newID) == 1) {
	       while (newBlobs.count(newID) == 1) {
                    newID++;
	       }
	       it->second.setID(newID);
	       newBlobs[newID] = it->second;
	  }
          return 0;
     }

     int formClusters(std::map<int,syllo::Blob> blobs, std::map<int,syllo::Cluster> &clusters)
     {
          // Compare each blob to each other blob and cluster together blobs
          // that are within a specified threshold of each other.
	  std::map<int,syllo::Blob>::iterator it;
	  for (it = blobs.begin() ; it != blobs.end() ; it++) {
	       std::map<int,syllo::Blob>::iterator it2;
	       for (it2 = blobs.begin() ; it2 != blobs.end() ; it2++) {
		    if (abs(dist(it->second.getCentroid(),it2->second.getCentroid())) < 30) {
			 if (it->second.clusterID == -1 && it2->second.clusterID == -1) {
                              // If neither blob has been added to a cluster
			      // yet...  Add to new cluster
			      int newID = 1;
                              
                              // Find the next available cluster ID
			      while (clusters.count(newID) == 1) {
				   newID++;
			      }

			      // Set the cluster ID in each blob
			      it->second.clusterID = newID;
			      it2->second.clusterID = newID;

                              // Add each blob to the new cluster
			      syllo::Cluster cluster;
			      cluster.addBlob(it->first,it->second);
			      cluster.addBlob(it2->first,it2->second);

			      // Save the cluster to a map
			      clusters[newID] = cluster;
			      
			 } else if (it->second.clusterID != -1 && it2->second.clusterID == -1) {
                              // If one blob has been added to a cluster and
                              // one hasn't yet... Add the blob that hasn't
                              // been added to the already created cluster.
			      it2->second.clusterID = it->second.clusterID;
			      clusters[it->second.clusterID].addBlob(it2->first,it2->second);
			 } else if (it->second.clusterID == -1 && it2->second.clusterID != -1) {
                              // Similar to previous case, but the first blob
                              // hasn't been added to a cluster
                              // yet. (vice-versa)
			      it->second.clusterID = it2->second.clusterID;
			      clusters[it2->second.clusterID].addBlob(it->first,it->second);
			 }
		    }
	       }
	  }

	  // Calculate the centroids of the clusters
	  std::map<int,syllo::Cluster>::iterator clustIt;
	  for (clustIt = clusters.begin() ; clustIt != clusters.end() ; clustIt++) {
	       clustIt->second.calcCentroid();
	  }

          return 0;
     }

     int ClusterMatch(std::map<int,syllo::Cluster> &prevClusters, std::map<int,syllo::Cluster> &clusters, std::map<int,syllo::Cluster> &newClusters, std::map<int,syllo::Cluster> &allClusters, int curTime, std::map<int,bool> &IDs) 
     {
	  /*
	  if (prevClusters.size() == 0) {
	       IDs[-2] = true;
	       IDs[-1] = true;
	       IDs[0] = true;
	       std::map<int,syllo::Cluster>::iterator it;
	       for (it = clusters.begin() ; it != clusters.end(); it++) {
	       newClusters[it->first] = clusters[it->first];
		    IDs[it->first] = true;
	       }
	       return 0;
	  }
	  */

	  if (curTime == 0) {
               // Initialize some variables on first frame.
	       IDs[-2] = true;
	       IDs[-1] = true;
	       IDs[0] = true;
	       std::map<int,syllo::Cluster>::iterator it;

               // If using particle filter, initialize particle filter.
               // Also, loop through each cluster and copy to new clusters
	       for (it = clusters.begin() ; it != clusters.end(); it++) {

#if ENABLE_PF == 1
		    clusters[it->first].particle_filter_init(1000,0,600,0,800,0,360,-10,10);//TODO: height/width
#endif

		    newClusters[it->first] = clusters[it->first];
		    allClusters[it->first] = clusters[it->first];
		    IDs[it->first] = true;
	       }
	       return 0;
	  }

          // Loop through all previous clusters and match with current clusters
          // based on absolute distance.
	  std::map<int,syllo::Cluster>::iterator itPrev;
	  for (itPrev = prevClusters.begin() ; itPrev != prevClusters.end(); itPrev++) {
	       bool matchFound = false;
	       std::map<int,syllo::Cluster>::iterator it;
	       for (it = clusters.begin() ; it != clusters.end(); it++) {
                    if ( abs(dist(itPrev->second.getCentroid(), it->second.getCentroid())) < 50) {
                         // Found a match between a current cluster and a
                         // previous cluster. Copy over the current cluster's
                         // information to the previous cluster.
			 itPrev->second.calcVelocity(itPrev->second.getCentroid(), it->second.getCentroid());
			 itPrev->second.setCentroid(it->second.getCentroid());
			 itPrev->second.incAge();

#if ENABLE_PF == 1
			 itPrev->second.pf_step();
#endif

                         // Copy over previous cluster into new clusters map
			 newClusters[itPrev->first] = itPrev->second;
			 allClusters[itPrev->first] = itPrev->second;

                         // Erase the current cluster from the map
			 clusters.erase(it->first);

			 matchFound = true;
			 break;
		    }
	       }

	       if (!matchFound) {
                    // If a match wasn't found, decrement the age.

		    itPrev->second.decAge();
#if ENABLE_PF == 1
                    itPrev->second.pf_step();
#endif 		    
		    if (itPrev->second.getAge() > 0) {
                         // Only copy over the previous cluster if it's age is
                         // greater than zero.
			 newClusters[itPrev->first] = itPrev->second;
		    }
                    // Keep track of all clusters, just for logging purposes
		    allClusters[itPrev->first] = itPrev->second;
	       }
	  }
	  
	  /*
	    // Find lowest available cluster ID.
	  std::map<int,syllo::Cluster>::iterator it;
	  for (it = clusters.begin() ; it != clusters.end(); it++) {
	       int newID = 1;
	       while (newClusters.count(newID) == 1) {
	       	 newID++;
	       }
	       it->second.declareFirstCentroid(curTime);
	       newClusters[newID] = it->second;
	  }
	  */

	  // Find next largest cluster ID, don't reuse cluster IDs
	  int champID = -2;
	  std::map<int,bool>::iterator iter;
	  for (iter = IDs.begin() ; iter != IDs.end(); iter++) {
	       if (iter->first > champID) {
		    champID = iter->first;
	       }
	  }

	  champID++; // next cluster ID 

	  // From the remaining additional clusters (we erased matched
	  // clusters), add them to the newClusters output map
	  std::map<int,syllo::Cluster>::iterator it;
	  for(it = clusters.begin() ; it != clusters.end() ; it++) {
	       it->second.declareFirstCentroid(curTime);
	       IDs[champID] = true;

#if ENABLE_PFS == 1
	       it->second.particle_filter_init(1000,0,600,0,800,0,360,-10,10);//TODO: height/width
#endif	       
	       newClusters[champID] = it->second;
	       allClusters[champID] = it->second;
	       champID++;
	  }
          return 0;
     }

     int StepClusterPF(std::map<int,syllo::Cluster> &clusters)
     {
	  std::map<int,syllo::Cluster>::iterator it;
	  for (it = clusters.begin() ; it != clusters.end(); it++) {
	       it->second.pf_step();
	  }
          return 0;
     }

     int getFarthestTravel(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster)
     {
	  //double velVarThresh = 0.5;
	  //std::map<int,syllo::Cluster>::iterator it;

	  int champ = -1000;
          int champ_ID = -1;
	  std::map<int,syllo::Cluster>::iterator it;
	  for (it = clusters.begin() ; it != clusters.end(); it++) {
	       if (it->second.getDistTravel() > champ) {
		    champ = it->second.getDistTravel();
		    cluster = it->second;
                    champ_ID = it->first;
	       }
	  }
          return champ_ID;
     }

     int getFastestCluster(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster)
     {
	  double champ = -1000;
	  std::map<int,syllo::Cluster>::iterator it;
	  for (it = clusters.begin() ; it != clusters.end(); it++) {
	       if (it->second.getAge() > 0  && it->second.getVelMag() > champ) {
		    champ = it->second.getVelMag();
		    cluster = it->second;
	       }
	  }
          return 0;
     }

     int getConstVelCluster(std::map<int,syllo::Cluster> &clusters, syllo::Cluster &cluster)
     {
	  double champ = 0.3;
	  std::map<int,syllo::Cluster>::iterator it;
	  int clusterChampID = -1;
	  printf("\n--------------------\n");
	  for (it = clusters.begin() ; it != clusters.end(); it++) {
	       	       
	       printf("Cluster: %d \t age: %d \t Vel: %f \t VelVar: %f\n", it->first, it->second.getAge(), it->second.getVelMag(), it->second.getVelVar());

	       //if (it->second.getAge() > 3 && it->second.getVelMag() > 1.4  && it->second.getAngleVariance() < champ) {
	       if (it->second.getAge() > 3 && it->second.getVelMag() > 1.41  && it->second.getVelVar() < champ) {
		    champ = it->second.getVelVar();
		    cluster = it->second;
		    clusterChampID = it->first;
	       }
	  }
	  printf("==> Winner: %d", clusterChampID);
          return 0;
     }

     int drawClusters(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Cluster> &clusters, int radius) 
     {
	  dst = src;
	  std::map<int,syllo::Cluster>::iterator it;
	  for (it = clusters.begin(); it != clusters.end(); it++ ) {
	       ostringstream convert;
	       convert << it->first;
	       const string& text = convert.str();
	     
	       cv::circle(dst, it->second.getCentroid(), radius, cv::Scalar(255,255,255), 1, 8, 0);
	       cv::putText(dst, text, it->second.getCentroid(), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1, 8, false);
	  }
	  return 0;
     }

     //int BlobMatch(std::map<int,syllo::Blob> &prevBlobs, std::map<int,syllo::Blob> &blobs, std::map<int,syllo::Blob> &newBlobs, int curTime) 
     //{
     //	  if (prevBlobs.size() == 0) {
     //	       std::map<int,syllo::Blob>::iterator it;
     //	       for (it = blobs.begin() ; it != blobs.end(); it++) {
     //		    newBlobs[it->first] = blobs[it->first];
     //	       }
     //	       return 0;
     //	  }
     //
     //	  std::map<int,syllo::Blob>::iterator it;
     //	  for (it = blobs.begin() ; it != blobs.end(); it++) {
     //	       bool matchFound = false;
     //	       std::map<int,syllo::Blob>::iterator itPrev;
     //	       for (itPrev = prevBlobs.begin() ; itPrev != prevBlobs.end(); itPrev++) {
     //		    //printf("%d Prev: %d %d ", i, itPrev->second.getCentroid().x, itPrev->second.getCentroid().y );
     //		    //printf("x:%d y:%d \t x:%d y:%d\n", itPrev->second.getCentroid().x, itPrev->second.getCentroid().y, it->second.getCentroid().x, it->second.getCentroid().y);		    //if ( abs(dist(itPrev->second.getCentroid(), it->second.getCentroid())) < 80) {
     //		    if ( abs(dist(itPrev->second.getCentroid(), it->second.getCentroid())) < 30) {
     //			 //if(abs(itPrev->second.getSize() - it->second.getSize()) < 50) {
     //			 // Match is found, set new blob's ID to the previous blob's ID
     //			 // Add blob to new blobs map
     //			 it->second.setID(itPrev->first);
     //			 newBlobs[it->second.getID()] = it->second;
     //			 prevBlobs.erase(itPrev->first);
     //
     //			 matchFound = true;
     //			 break;
     //			 //}
     //			 //}
     //		    }
     //	       }
     //	       if (!matchFound) {
     //		    // Could not find blob's match
     //		    // Find the next available slot in the map
     //		    int newID = 1;
     //		    while (prevBlobs.count(newID) == 1 || newBlobs.count(newID) == 1) {
     //			 newID++;
     //		    }
     //		    it->second.setID(newID);
     //		    newBlobs[newID] = it->second;
     //	       }
     //	  }
     //}

     // take number image type number (from cv::Mat.type()), get OpenCV's enum string.
     std::string getImgType(int imgTypeInt)
     {
	  int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

	  int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
				   CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
				   CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
				   CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
				   CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
				   CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
				   CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

	  string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
				   "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
				   "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
				   "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
				   "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
				   "CV_32F"  "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
				   "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

	  for(int i=0; i<numImgTypes; i++)
	  {
	       if(imgTypeInt == enum_ints[i]) return enum_strings[i];
	  }
	  return "unknown image type";
     }

     int BlobTemplateMatch(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Blob> &blobs, cv::Mat &templateImg)
     {

	  input.copyTo(output);

	  int matchMethod = CV_TM_CCOEFF;

	  //cv::Mat matchImg;
	  //int match_cols = input.cols - templImg.cols + 1; 
	  //int match_rows = input.rows - templImg.rows + 1;
	  //matchImg.create( match_cols, match_rows, CV_32FC1 );
	  cv::Mat matchImg;
	  matchImg.create(1,1,CV_32FC1);

	  //cv::imshow("input",input);
	  //cv::imshow("template",templateImg);
	  //cv::waitKey(0);

	  //printf("\n\n----------------\n");
	  //printf("%s %d %s %d\n", getImgType(input.type()).c_str(), input.depth(), getImgType(templateImg.type()).c_str(), templateImg.depth());
	  
	  std::map<int,syllo::Blob>::iterator it;
          float champ = -2000;
	  syllo::Blob champBlob;
	  for (it = blobs.begin() ; it != blobs.end(); it++) {
	       //printf("blobs: %d\n",i++);
	       cv::Rect roiRect = cv::Rect(it->second.getCentroid().x-templateImg.cols/2, 
					   it->second.getCentroid().y-templateImg.rows/2, 
					   templateImg.cols, 
					   templateImg.rows);

	//printf("x:%d y:%d x:%d y:%d\n",it->second.getCentroid().x-templImg.cols/2, 
	//	      it->second.getCentroid().y-templImg.rows/2, 
	//	      it->second.getCentroid().x+templImg.cols/2, 
	//	      it->second.getCentroid().y+templImg.rows/2);

	       cv::Mat roi;
	       try {
		    roi = cv::Mat(input,roiRect);
	       } catch(cv::Exception e){continue;}		    
	       cv::matchTemplate(roi, templateImg, matchImg, matchMethod);
	       //cv::imshow("roi",roi);
	       //cv::imshow("template",templateImg);
	       //cv::imshow("match",matchImg);
	       //printf("Correlation: %f\n", matchImg);
	       //printf("\n\n---------\nCorr:\n");
	       //std::cout << matchImg << std::endl;
	       
	       float result = matchImg.at<float>(0,0);
	       if (result > champ) {
		    champ = result;
		    champBlob = it->second;
	       }
	       //cv::waitKey(0);
	  }
	  
	  if (champ != -2000) {
	       cv::circle(output, champBlob.getCentroid(), 15, cv::Scalar(0,0,255), 1, 8, 0);
	  }
	  
	  //cv::normalize( matchImg, matchImg, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
     
	  //// Localizing the best match with minMaxLoc
	  //double minVal; 
	  //double maxVal; 
	  //cv::Point minLoc; 
	  //cv::Point maxLoc; 
	  //cv::Point matchLoc;
     	  //
	  //cv::minMaxLoc( matchImg, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
	  //
	  //// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
	  //if( matchMethod == CV_TM_SQDIFF || matchMethod == CV_TM_SQDIFF_NORMED ) { 
	  //     matchLoc = minLoc; 
	  //} else { 
	  //     matchLoc = maxLoc; 
	  //}
	  //
	  //cv::Rect roi = cv::Rect(matchLoc.x-10, matchLoc.y-10, templImg.cols+20, templImg.rows+20);
	  //cv::Mat subImg(outputImg, roi);
	  return 0;
     }

     int ClusterTemplateMatch(cv::Mat &input, cv::Mat &output, std::map<int,syllo::Cluster> &clusters, cv::Mat &templateImg)
     {

	  input.copyTo(output);

	  int matchMethod = CV_TM_CCOEFF;

	  cv::Mat matchImg;
	  matchImg.create(1,1,CV_32FC1);

	  std::map<int,syllo::Cluster>::iterator it;
          float champ = -2000;
	  syllo::Cluster champCluster;
	  for (it = clusters.begin() ; it != clusters.end(); it++) {
	       cv::Rect roiRect = cv::Rect(it->second.getCentroid().x-templateImg.cols/2-10, 
					   it->second.getCentroid().y-templateImg.rows/2-10, 
					   templateImg.cols, 
					   templateImg.rows);

	       cv::Mat roi;
	       try {
		    roi = cv::Mat(input,roiRect);
	       } catch(cv::Exception e){continue;}		    

	       cv::imshow("test",roi);

	       cv::matchTemplate(roi, templateImg, matchImg, matchMethod);
	       	       
	       float result = matchImg.at<float>(0,0);
	       if (result > champ) {
		    champ = result;
		    champCluster = it->second;
	       }
	  }
	  
	  if (champ != -2000) {
	       cv::circle(output, champCluster.getCentroid(), 15, cv::Scalar(0,0,255), 1, 8, 0);
	  }
	  
	  return 0;
     }

     int drawCircleList(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Blob> &blobs, int radius, cv::Scalar color) 
     {
	  dst = src;
	  std::map<int,syllo::Blob>::iterator it;
	  for (it = blobs.begin(); it != blobs.end(); it++ ) {
	       cv::circle(dst, it->second.getCentroid(), radius, color, 1, 8, 0);
	  }
	  return 0;
     }

     int drawBlobTargets(cv::Mat &src, cv::Mat &dst, std::map<int,syllo::Blob> &blobs, int radius) 
     {
	  dst = src;
	  std::map<int,syllo::Blob>::iterator it;
	  for (it = blobs.begin(); it != blobs.end(); it++ ) {
	       ostringstream convert;
	       convert << it->first;
	       //convert << it->second.getSize();
	       const string& text = convert.str();
	       
               //cv::circle(dst, it->second.getCentroid(), radius, it->second.getID(), -1, 8, 0);
	       cv::putText(dst, text, it->second.getCentroid(), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1, 8, false);
	       //cv::circle(dst, it->second.getCentroid(), radius, it->first, -1, 8, 0);
	  }
	  return 0;
     }


     int RunningGaussian(double alpha, cv::Mat &img1, cv::Mat &img2, cv::Mat &avgs, cv::Mat &varImg)
     {
	  //double alpha = 0.1;
	  //double alpha = 0.05;
	  //double alpha = 0.001;
	  //double alpha = 0.05;
	  //double alpha = 0.05;
	  //double k = 5;
	  
	  cv::Mat foreImg = cv::Mat::zeros(img1.size(), CV_8UC1);
	  
	  if (avgs.empty()) {
	       img1.copyTo(avgs);
	       avgs.copyTo(varImg);
	  }
	  
	  int channels = img1.channels();
	  int nRows = img1.rows;
	  int nCols = img1.cols * channels;
	  if (img1.isContinuous()) {
	       nCols *= nRows;
	       nRows = 1; 
	  }

	  int i,j;
	  uchar *p1, *p2, *p3, *p4;
	  uchar diff;
	  for( i = 0; i < nRows; ++i) {
	       p1 = img1.ptr<uchar>(i);
	       p2 = avgs.ptr<uchar>(i);
	       p3 = foreImg.ptr<uchar>(i);
	       p4 = varImg.ptr<uchar>(i);
	       for ( j = 0; j < nCols; ++j) {
		    p2[j] = alpha*p1[j] + (1-alpha)*p2[j];

		    diff = abs(p1[j] - p2[j]);
		    p4[j] = alpha*diff*diff + (1-alpha)*p4[j];

		    //if ( abs(p1[j] - p2[j]) > 25) {
		    if ( abs(p1[j] - p2[j]) > (25+sqrt(p4[j]))) {
			 //p3[j] = p1[j];
			 p3[j] = 255;
		    }
	       }
	  }

	  foreImg.copyTo(img2);
          return 0;
     }

     // Test function
     void helloWorld()
     {
	  std::cout << "Hello World from a namespace!" << std::endl;
     }

     int drawClusterParticles(std::map<int,syllo::Cluster> &clusters, cv::Mat &input, cv::Mat &output)
     {
	  std::map<int,syllo::Cluster>::iterator it;
	  for (it = clusters.begin() ; it != clusters.end(); it++) {
	       ParticleFilter pf;
	       it->second.getParticleFilter(pf);
	       drawParticles(pf, input, output);
	  }
          return 0;
     }

     int drawParticles(ParticleFilter &pf, cv::Mat &src, cv::Mat &dst)
     {
	  std::vector<Particle> particles;
	  pf.getParticles(particles);

	  dst = src;

	  std::vector<Particle>::iterator it;
	  for (it = particles.begin(); it != particles.end(); it++) {
	       //cv::circle(dst, it->center(), (it->normalized_weight()+1)*5, cv::Scalar(255,255,255), 1, 8, 0);
	       cv::circle(dst, it->center(), 1, cv::Scalar(255,255,255), 1, 8, 0);
	       //cout << "norm weight: " << it->normalized_weight() << endl;
	  }
	  return 0;
     }

     int flipClusterEntity(cv::Point point, std::map<int,syllo::Cluster> &clusters)
     {
          double min_dist = 1000;
          std::map<int,syllo::Cluster>::iterator it, itMin;
          for (it = clusters.begin(); it != clusters.end(); it++) {
               double distance;
               distance = dist(point, it->second.getCentroid());
               if (distance < min_dist) {
                    min_dist = distance;
                    itMin = it;
               }
          }
          
          EntityType type = itMin->second.type();
          if (type == clutter) {
               cout << "Setting clutter " << itMin->first << " to diver" << endl;
               itMin->second.set_type(diver);
          } else if(type == diver) {
               cout << "Setting diver " << itMin->first << " to clutter" << endl;
               itMin->second.set_type(clutter);
          } else {
               cout << "Invalid entity type" << endl;
               cout << "Setting entity " << itMin->first << " to clutter" << endl;
               itMin->second.set_type(clutter);
          }
          return 0;
     }

     int tagClusterEntity(cv::Point point, EntityType target, EntityType others, 
                          std::map<int,syllo::Cluster> &clusters)
     {
          double min_dist = 1000;
          std::map<int,syllo::Cluster>::iterator it, itMin;
          for (it = clusters.begin(); it != clusters.end(); it++) {
               double distance;
               distance = dist(point, it->second.getCentroid());
               it->second.set_type(others);
               if (distance < min_dist) {
                    min_dist = distance;
                    itMin = it;
               }
          }
          
          itMin->second.set_type(target);
          cout << "Setting " << itMin->first << " to " 
               << itMin->second.type() << endl;
          
          return 0;
     }

     cv::Point getTypeCentroid(EntityType type, std::map<int,syllo::Cluster> &clusters)
     {
          std::map<int,syllo::Cluster>::iterator it;
          for (it = clusters.begin(); it != clusters.end(); it++) {
               if (type == it->second.type()) {
                    return it->second.getCentroid();
               }
          }
          return cv::Point(-1,-1);          
     }

}


/*
 * Debug Code
 */
//	 cv::Mat temp;
//	 img.copyTo(temp);
//	 cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
//	 cv::applyColorMap(temp,temp,cv::COLORMAP_JET);
//	 cv::circle(temp, cv::Point(j,i), 5, cv::Scalar(255,255,255), 1, 8, 0);
//	 cv::imshow("temp",temp);
//	 cv::waitKey(0);
