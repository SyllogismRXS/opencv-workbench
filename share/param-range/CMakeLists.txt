set( APP_NAME param-range)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}
  track
  syllo
  sonar-sdk  
  ${Boost_LIBRARIES}  
  workbench-utils
  plot
  dl
  wb
  ${YAMLCPP_LIBRARY}
  )
