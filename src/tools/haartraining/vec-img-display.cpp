#include <iostream>
#include <stdio.h>
#include <stdlib.h> 

#include "cvhaartraining.h"

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
     if (argc < 4) {
          cout << "Invalid parameters" << endl;
          return -1;
     }

     std::string filename = argv[1];
     int winwidth = atoi(argv[2]);
     int winheight = atoi(argv[3]);

     cvShowVecSamples(filename.c_str(), winwidth, winheight, 1.0);

     return 0;
}
