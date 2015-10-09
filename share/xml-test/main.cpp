#include <iostream>

#include <opencv_workbench/utils/AnnotationParser.h>
//#include <opencv_workbench/utils/Object.h>

using std::cout;
using std::endl;

int main(int argc, char * argv[])
{
     if (argc < 2) {
          cout << "Missing xml file argument" << endl;
          return -1;
     }
     AnnotationParser parser;
     int status = parser.ParseFile(argv[1]);
     cout << "Parse status: " << status << endl;
     cout << "================" << endl;
     parser.print();
     return 0;
}
