set( LIB_NAME opencv-built-in )

file( GLOB SRCS *.cpp)
file( GLOB HDRS *.h)

ADD_LIBRARY( ${LIB_NAME} SHARED
  ${SRCS}
)

target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})
