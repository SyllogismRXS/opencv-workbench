#include <iostream>
#include <opencv_workbench/utils/BoundingBox.h>

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     //int xmin, int xmax, int ymin, int ymax
     BoundingBox b1(1,5,1,5);
     BoundingBox b2(2,6,2,4);

     if (BoundingBox::overlap(b2,b1)) {
          cout << "Overlap" << endl;
     } else {
          cout << "Not" << endl;
     }
     
     return 0;
}
