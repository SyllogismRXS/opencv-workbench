set( APP_NAME eigenfaces)

file ( GLOB_RECURSE app_SOURCES *.cpp )
file ( GLOB_RECURSE app_HEADERS *.h )

add_executable(${APP_NAME} 
  ${app_SOURCES}
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}
  )
