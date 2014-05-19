set( LIB_NAME larks )



ADD_LIBRARY( ${LIB_NAME} SHARED
  larks.cpp
)

target_link_libraries(${LIB_NAME} 
  ${OpenCV_LIBS} 
  )
