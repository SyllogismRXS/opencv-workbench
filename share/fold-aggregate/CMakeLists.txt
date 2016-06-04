set( APP_NAME fold-aggregate)

add_executable(${APP_NAME} 
  main.cpp
  )

message ("BOOST LIBS: ${Boost_LIBRARIES}")
target_link_libraries(${APP_NAME} 
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  track
  syllo
  sonar-sdk    
  workbench-utils
  plot
  dl
  wb
  ${YAMLCPP_LIBRARY}
  )
