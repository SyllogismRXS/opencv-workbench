cmake_minimum_required (VERSION 2.8)
project (opencv_workbench)

set(OPENCV_WORKBENCH_VERSION_MAJOR "0")
set(OPENCV_WORKBENCH_VERSION_MINOR "0")
set(OPENCV_WORKBENCH_VERSION_PATCH "0")
set(OPENCV_WORKBENCH_VERSION "${OPENCV_WORKBENCH_VERSION_MAJOR}.${OPENCV_WORKBENCH_VERSION_MINOR}.${OPENCV_WORKBENCH_VERSION_PATCH}")
set(OPENCV_WORKBENCH_VERSION_DATE "2012.12.30")

# give OPENCV_WORKBENCH 1 series a few more soversions
set(OPENCV_WORKBENCH_SOVERSION "00")

set(CMAKE_INSTALL_PREFIX /usr/local)

# fetch all the local directories for generated code
get_filename_component(OPENCV_WORKBENCH_LIB_DIR ./lib ABSOLUTE)
get_filename_component(OPENCV_WORKBENCH_BIN_DIR ./bin ABSOLUTE)
get_filename_component(OPENCV_WORKBENCH_INC_DIR ./include ABSOLUTE)
get_filename_component(OPENCV_WORKBENCH_SRC_DIR ./src ABSOLUTE)
get_filename_component(OPENCV_WORKBENCH_SHARE_DIR ./share ABSOLUTE)
get_filename_component(OPENCV_WORKBENCH_BUILD_DIR ./build ABSOLUTE)
get_filename_component(OPENCV_WORKBENCH_SCRIPTS_DIR ./scripts ABSOLUTE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE ( Boost COMPONENTS filesystem system REQUIRED)

set(CMAKE_CXX_FLAGS "-g -Wall")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall") # -fprofile-arcs -ftest-coverage")

set(EXECUTABLE_OUTPUT_PATH ${OPENCV_WORKBENCH_BIN_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OPENCV_WORKBENCH_LIB_DIR})

## set the cmake defaults for libraries and binaries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OPENCV_WORKBENCH_LIB_DIR} CACHE PATH 
  "Output directory for the dynamic libraries" )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OPENCV_WORKBENCH_BIN_DIR} CACHE PATH
  "Output directory for the binaries" )
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OPENCV_WORKBENCH_LIB_DIR} CACHE PATH 
  "Output directory for the static libraries (archives)" )

INSTALL(DIRECTORY ${OPENCV_WORKBENCH_INC_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX}
  FILES_MATCHING REGEX .*\\.h|.*\\.proto
)

#INSTALL(TARGETS ${} 
#  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#)
#
#install(DIRECTORY ${OPENCV_WORKBENCH_LIB_DIR}/ 
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#  )

## let cmake know where the headers are
include_directories(${OPENCV_WORKBENCH_INC_DIR})

include_directories(${OPENCV_WORKBENCH_BUILD_DIR}/src/gui)

include_directories(${OPENCV_WORKBENCH_INC_DIR}/opencv_workbench/syllo-widgets)

## copy to opencv_workbench/include
file(GLOB_RECURSE INCLUDE_FILES RELATIVE ${OPENCV_WORKBENCH_SRC_DIR} src/*.h)
foreach(I ${INCLUDE_FILES})
  configure_file(${OPENCV_WORKBENCH_SRC_DIR}/${I} ${OPENCV_WORKBENCH_INC_DIR}/opencv_workbench/${I} @ONLY)
endforeach()

# Find Qt and add headers and definitions
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

#message("=======================")
#message(${QT_USE_FILE})
#message(${QT_INCLUDE_DIR})

include_directories(${QT_INCLUDE_DIR})

# Find OpenCV
find_package (OpenCV REQUIRED )

if (OpenCV_FOUND)
message("=======================")
message("OpenCV Found")
message("Directories:")
message("${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
message("-----------------------")
endif (OpenCV_FOUND)

# Find Yaml-Cpp
find_package(YamlCpp REQUIRED)
include_directories(${YAMLCPP_INCLUDE_DIR})

# Determine if the BLUEVIEW_SDK_ROOT has been set
message("==================================")
if (DEFINED ENV{BLUEVIEW_SDK_ROOT})  
  message("Found Blueview SDK at:")
  message("$ENV{BLUEVIEW_SDK_ROOT}")  
  # Include the BLUEVIW SDK header files
  include_directories($ENV{BLUEVIEW_SDK_ROOT}/include)
  # Set the Blueview SDK libraries
  set(BLUEVIEW_SDK_LIBS $ENV{BLUEVIEW_SDK_ROOT}/lib/libbvtsdk.so)
  add_definitions(-DENABLE_SONAR=1)
else()
  message("WARNING: Can't find the BlueView SDK")
  message("Set the BLUEVIEW_SDK_ROOT environment variable in your .bashrc")
  message("if you require the blueview library")  
  add_definitions(-DENABLE_SONAR=0)
  #return() # exit quietly if sdk not found
endif()

add_subdirectory(src)
add_subdirectory(share)
