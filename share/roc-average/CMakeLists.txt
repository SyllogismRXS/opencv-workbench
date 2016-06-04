set( APP_NAME roc-average)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  syllo
  ${OpenCV_LIBS}
  track  
  sonar-sdk  
  ${Boost_LIBRARIES}  
  workbench-utils
  plot
  dl
  wb
  ${YAMLCPP_LIBRARY}
  )
