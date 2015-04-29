set( APP_NAME xml-test)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  workbench-utils
  ${Boost_LIBRARIES}
  )
