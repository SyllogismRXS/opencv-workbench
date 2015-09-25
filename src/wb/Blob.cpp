#include <iostream>

#include "Blob.h"

using std::cout;
using std::endl;

namespace wb {

Blob::Blob()
{
     set_id(-1);
}

Blob::Blob(int id)
{     
     set_id(id);
}

}
