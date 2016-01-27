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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
#        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

########################################################
# Find dlib
########################################################
find_package(DLIB REQUIRED)

message("======================")
IF (DLIB_FOUND)  
  message("FOUND DLIB")
  message("Includes: ${DLIB_INCLUDE_DIR}")
  message("Libraries: ${DLIB_LIBRARIES}")
  include_directories(${DLIB_INCLUDE_DIR})
ELSE()
  error("Can't find DLIB.")
ENDIF()
message("======================")

########################################################
# Find Eigen
########################################################
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

########################################################
# Find Boost
########################################################
find_package(Boost COMPONENTS thread date_time iostreams program_options 
  filesystem system REQUIRED)

if(Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIRS})
endif()
message("===========================")
message("BOOST Directories:")
message("${Boost_INCLUDE_DIRS}")

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

#INSTALL(DIRECTORY ${OPENCV_WORKBENCH_INC_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX}
#  FILES_MATCHING REGEX .*\\.h|.*\\.proto
#)

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
file(GLOB_RECURSE INCLUDE_FILES RELATIVE ${OPENCV_WORKBENCH_SRC_DIR} src/*.h src/*.hpp)
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

include_directories(${OPENCV_WORKBENCH_SRC_DIR}/gui/annotate)

##############################################
# Compiler Optimizations
##############################################
#set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release
      RelWithDebInfo MinSizeRel." FORCE)
endif()

# Setup some options to allow a user to enable SSE and AVX instruction use.  
if (CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" 
                                OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU"
                                OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    option(USE_SSE2_INSTRUCTIONS "Compile your program with SSE2 instructions" OFF)
    option(USE_SSE4_INSTRUCTIONS "Compile your program with SSE4 instructions" OFF)
    option(USE_AVX_INSTRUCTIONS  "Compile your program with AVX instructions"  OFF)
    if(USE_AVX_INSTRUCTIONS)
        add_definitions(-mavx)
        message(STATUS "Enabling AVX instructions")
    elseif (USE_SSE4_INSTRUCTIONS)
        add_definitions(-msse4)
        message(STATUS "Enabling SSE4 instructions")
    elseif(USE_SSE2_INSTRUCTIONS)
        add_definitions(-msse2)
        message(STATUS "Enabling SSE2 instructions")
    endif()
elseif (MSVC OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC") # else if using Visual Studio 
    # Use SSE2 by default when using Visual Studio.
    option(USE_SSE2_INSTRUCTIONS "Compile your program with SSE2 instructions" OFF)
    # Visual Studio 2005 didn't support SSE4 
    if (NOT MSVC80)
        option(USE_SSE4_INSTRUCTIONS "Compile your program with SSE4 instructions" OFF)
    endif()
    # Visual Studio 2005 and 2008 didn't support AVX
    if (NOT MSVC80 AND NOT MSVC90)
        option(USE_AVX_INSTRUCTIONS  "Compile your program with AVX instructions"  OFF)
    endif() 
    include(CheckTypeSize)
    check_type_size( "void*" SIZE_OF_VOID_PTR)
    if(USE_AVX_INSTRUCTIONS)
        add_definitions(/arch:AVX)
        message(STATUS "Enabling AVX instructions")
    elseif (USE_SSE4_INSTRUCTIONS)
        # Visual studio doesn't have an /arch:SSE2 flag when building in 64 bit modes.
        # So only give it when we are doing a 32 bit build.
        if (SIZE_OF_VOID_PTR EQUAL 4)
            add_definitions(/arch:SSE2)  
        endif()
        message(STATUS "Enabling SSE4 instructions")
        add_definitions(-DDLIB_HAVE_SSE2)
        add_definitions(-DDLIB_HAVE_SSE3)
        add_definitions(-DDLIB_HAVE_SSE41)
    elseif(USE_SSE2_INSTRUCTIONS)
        # Visual studio doesn't have an /arch:SSE2 flag when building in 64 bit modes.
        # So only give it when we are doing a 32 bit build.
        if (SIZE_OF_VOID_PTR EQUAL 4)
            add_definitions(/arch:SSE2)
        endif()
        message(STATUS "Enabling SSE2 instructions")
        add_definitions(-DDLIB_HAVE_SSE2)
    endif()
endif()


add_subdirectory(src)
add_subdirectory(share)

###############################################################################
# Installation
# 
###############################################################################
set(PKGCONFIG_INSTALL_ONLY ON CACHE BOOL "Only install pkgconfig file")

if (NOT PKGCONFIG_INSTALL_ONLY)
  set(CMAKE_INSTALL_PREFIX_SHADOW ${CMAKE_INSTALL_PREFIX})
  
  install(DIRECTORY ${OPENCV_WORKBENCH_LIB_DIR}/ 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/opencv_workbench
    )

  # Install header files to system
  # The CMakeLists.txt for each library contains the 
  # commands to install the library
  # TODO, since we removed include directory during development builds, we need
  # to add a new rule here to create the include directory before installation.
  INSTALL(
    DIRECTORY ${OPENCV_WORKBENCH_INC_DIR} 
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    FILES_MATCHING REGEX .*\\.h|.*\\.proto|.*\\.inl
    )
else()
  # If we are only installing the pkgconfig file, then make the shadow install
  # prefix set to the current directory
  set(CMAKE_INSTALL_PREFIX_SHADOW ${CMAKE_SOURCE_DIR})
endif()

IF(UNIX AND NOT APPLE)
  # - Add each library to LIBS_STRING 
  # OPENCV_WORKBENCH_LIBS is a "global" variable that is passed
  # down to each sub directory and every library that is
  # generated in this project appends its name to this
  # variable.
  foreach(I ${OPENCV_WORKBENCH_LIBS})
    #get_filename_component(CUR_LIB ${I} NAME)
    #message("------> : Lib")    
    #message(${I})
    set (LIBS_STRING "${LIBS_STRING} -l${I}")    
  endforeach()    

  # - Use the opencv_workbench.pc.in file to generate the 
  #     the final opencv_workbench.pc file
  configure_file (
    "${CMAKE_MODULE_PATH}/opencv_workbench.pc.in" 
    "${OPENCV_WORKBENCH_BUILD_DIR}/opencv_workbench.pc" 
    @ONLY)

  # - Install the opencv_workbench.pc file to lib/pkgconfig
  install (
    FILES "${CMAKE_CURRENT_BINARY_DIR}/opencv_workbench.pc"
    DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig"
    )
ENDIF()
