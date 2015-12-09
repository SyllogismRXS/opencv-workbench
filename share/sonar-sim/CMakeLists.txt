set( APP_NAME sonar-sim)

find_package(YamlCpp REQUIRED)
if(YAMLCPP_FOUND)
  message("Found YamlCpp")
  message("YamlCpp include directories: ${YAMLCPP_INCLUDE_DIR}")
  message("YamlCpp libraries: ${YAMLCPP_LIBRARY}")  
  include_directories(${YAMLCPP_INCLUDE_DIR})
else()
  message(FATAL_ERROR "Failed to find Yaml-Cpp")
endif()

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}
  track
  syllo
  #sonar-sdk  
  ${Boost_LIBRARIES}  
  workbench-utils
  plot
  wb
  ${YAMLCPP_LIBRARY}
  trajectory  
  )
