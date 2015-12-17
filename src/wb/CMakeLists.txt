###################################################
# OpenCV Workbench Library
###################################################
set( LIBRARY_NAME_1 wb )

ADD_LIBRARY( ${LIBRARY_NAME_1} SHARED
  Point.cpp
  Cluster.cpp
  WB.cpp
  ClusterProcess.cpp
  NDT.cpp
  BlobProcess.cpp
  Blob.cpp
  Entity.cpp
  OpticalFlow.cpp
  )

target_link_libraries(${LIBRARY_NAME_1} 
  ${Boost_LIBRARIES}  
  ${OpenCV_LIBS}   
  track
  workbench-utils
  )

# Push up the OPENCV_WORKBENCH_LIBS variable
set(OPENCV_WORKBENCH_LIBS ${OPENCV_WORKBENCH_LIBS} ${LIBRARY_NAME_1} PARENT_SCOPE)
