set( LIB_NAME track )

file(GLOB SRCS *.cpp)

ADD_LIBRARY( ${LIB_NAME} SHARED
  ${SRCS}
)

target_link_libraries(${LIB_NAME} 
  #${OpenCV_LIBS} 
  )

