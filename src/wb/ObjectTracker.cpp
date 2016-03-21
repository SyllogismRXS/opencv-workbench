#include <iostream>

#include "ObjectTracker.h"

using std::cout;
using std::endl;

ObjectTracker::ObjectTracker()
{     
     pixel_tracker_.set_R(20);
     pixel_tracker_.set_P(100);
}

void ObjectTracker::process_frame(std::vector<wb::Blob> &blobs)
{
     // TODO: do something with the blobs!
}
