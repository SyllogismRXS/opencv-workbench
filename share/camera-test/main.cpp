#include <iostream>
#include <opencv_workbench/utils/Stream.h>
#include <opencv_workbench/utils/ColorMaps.h>
#include <opencv_workbench/syllo/syllo.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     if (argc < 2) {
          cout << "Usage: " << argv[0] << " <input source>" << endl;
          return -1;
     }
     
     syllo::Stream stream;
     syllo::Status status = stream.open(syllo::str2int(argv[1]));
     
     if (status != syllo::Success) {
          cout << "Failed to open: " << argv[1] << endl;
          return -1;
     }

     cout << stream.width() << " x " << stream.height() << endl;
     
     cv::Mat original;
     while (stream.read(original)) {
          cv::imshow("Cam", original);
          if (cv::waitKey(1) == 'q') {
               break;
          }
     }

          return 0;
     }
