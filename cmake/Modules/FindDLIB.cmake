INCLUDE(FindPackageHandleStandardArgs)

set(DLIB_INCLUDE_DIR /usr/local/include)

SET(DLIB_LibrarySearchPaths
  /usr/local/lib
  /usr/lib  
  )

FIND_LIBRARY(DLIB_LIBRARY
  NAMES dlib
  PATHS ${DLIB_LibrarySearchPaths}
  )

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DLIB "Could NOT find DLIB library"
  DLIB_LIBRARY
  DLIB_INCLUDE_DIR
)

SET(DLIB_LIBRARIES)
LIST(APPEND DLIB_LIBRARIES ${DLIB_LIBRARY})

MARK_AS_ADVANCED(
  DLIB_INCLUDE_DIR
  DLIB_LIBRARIES
  DLIB_FOUND
)
