set( APP_NAME ground-plane)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}  
  syllo
  sonar-sdk  
  ${Boost_LIBRARIES}  
  workbench-utils
  plot
  dl
  track
  wb
  )
