set( APP_NAME hungarian-test)

add_executable(${APP_NAME} 
  main.cpp
  )

target_link_libraries(${APP_NAME} 
  ${OpenCV_LIBS}
  track
  )

set( APP_NAME_2 hungarian-blob-test)

add_executable(${APP_NAME_2} 
  main_2.cpp
  )

target_link_libraries(${APP_NAME_2} 
  ${OpenCV_LIBS}
  track
  wb
  workbench-utils
  )
