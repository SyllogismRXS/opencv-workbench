###################################################
# Sonar SDK Library
###################################################
set( LIB_NAME sonar-sdk )

ADD_LIBRARY( ${LIB_NAME} SHARED
  Sonar.h
  Sonar.cpp        
)

target_link_libraries(${LIB_NAME} 
  ${OpenCV_LIBS} 
  ${BLUEVIEW_SDK_LIBS}
  )


###################################################
# Utils Library
###################################################
set( LIB_NAME_2 workbench-utils )

ADD_LIBRARY( ${LIB_NAME_2} SHARED
  AnnotationParser.cpp
  )

target_link_libraries(${LIB_NAME_2} 
  ${Boost_LIBRARIES}  
  #${OpenCV_LIBS} 
  )
