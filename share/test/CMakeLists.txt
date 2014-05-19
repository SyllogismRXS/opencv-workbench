set( APP_NAME test)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}
  opencv-workbench-utils
  LARKS
  )
