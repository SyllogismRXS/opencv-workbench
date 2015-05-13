#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/syllo/syllo.h>
#include <opencv_workbench/periodic/PeriodicDetect.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     cout << "Periodic Detection" << endl;     

     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input-file>" << endl;
          return -1;
     }
     
     syllo::Stream stream;
     syllo::Status status = stream.open(argv[1]);

     if (status != syllo::Success) {
          cout << "Failed to open: " << argv[1] << endl;
          return -1;
     }
     
     PeriodicDetect pd;
     
     cv::Mat original;
     int frame_num = 0;
     while (stream.read(original)) {          

          // Display images
          //cv::imshow("Original", original);                    
          //if(cv::waitKey(10) >= 0) break;
                    
          pd.add_frame(original);
          
          frame_num++;
     }

     pd.recurrence();
     
     return 0;
}
