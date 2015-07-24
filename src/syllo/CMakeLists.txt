set( LIBRARY_NAME syllo )

file( GLOB SRCS *.cpp)
file( GLOB HDRS *.h)

ADD_LIBRARY( ${LIBRARY_NAME} SHARED
  ${SRCS}
)

target_link_libraries(${LIBRARY_NAME} 
  ${OpenCV_LIBS}
  track
  yaml-cpp  
  )

# Push up the OPENCV_WORKBENCH_LIBS variable
set(OPENCV_WORKBENCH_LIBS ${OPENCV_WORKBENCH_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)