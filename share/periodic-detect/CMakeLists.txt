set( APP_NAME periodic-detect)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}
  track
  syllo
  sonar-sdk  
  periodic
  )
