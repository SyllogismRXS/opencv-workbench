###################################################
# Sonar SDK Library
###################################################
set( LIBRARY_NAME sonar-sdk )

ADD_LIBRARY( ${LIBRARY_NAME} SHARED
  Sonar.h
  Sonar.cpp        
)

target_link_libraries(${LIBRARY_NAME} 
  ${OpenCV_LIBS} 
  ${BLUEVIEW_SDK_LIBS}
  )

# Push up the OPENCV_WORKBENCH_LIBS variable
set(OPENCV_WORKBENCH_LIBS ${OPENCV_WORKBENCH_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)

###################################################
# Utils Library
###################################################
set( LIBRARY_NAME_2 workbench-utils )

ADD_LIBRARY( ${LIBRARY_NAME_2} SHARED
  AnnotationParser.cpp
  ColorMaps.cpp
  OpenCV_Helpers.cpp
  )

target_link_libraries(${LIBRARY_NAME_2} 
  ${Boost_LIBRARIES}  
  ${OpenCV_LIBS} 
  plot
  syllo
  )

# Push up the OPENCV_WORKBENCH_LIBS variable
set(OPENCV_WORKBENCH_LIBS ${OPENCV_WORKBENCH_LIBS} ${LIBRARY_NAME_2} PARENT_SCOPE)