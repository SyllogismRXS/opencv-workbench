set( LIB_NAME opencv-workbench-utils )

ADD_LIBRARY( ${LIB_NAME} SHARED
	Stream.h
        Sonar.h
        Sonar.cpp        
)

target_link_libraries(${LIB_NAME} 
  ${OpenCV_LIBS} 
  ${BLUEVIEW_SDK_LIBS}
  )
