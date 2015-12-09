set( LIBRARY_NAME trajectory )

file(GLOB SRCS *.cpp)

ADD_LIBRARY( ${LIBRARY_NAME} SHARED
  ${SRCS}
)

target_link_libraries(${LIBRARY_NAME} 
  ${OpenCV_LIBS} 
  wb  
  )


# Push up the OPENCV_WORKBENCH_LIBS variable
set(OPENCV_WORKBENCH_LIBS ${OPENCV_WORKBENCH_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)