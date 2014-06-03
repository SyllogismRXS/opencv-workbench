set( LIB_NAME LARKS )

ADD_LIBRARY( ${LIB_NAME} SHARED
  LARKS.cpp
)

target_link_libraries(${LIB_NAME} 
  ${OpenCV_LIBS} 
  boost_filesystem
  boost_system
)

