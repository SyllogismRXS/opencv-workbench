set( APP_NAME range-test)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}
  sonar-sdk  
  ${Boost_LIBRARIES}  
  workbench-utils
  plot
  dl
  )
