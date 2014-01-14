# haartraining executable
set( APP_NAME_1 haartraining)
add_executable(${APP_NAME_1} 
  cvhaartraining.cpp
  cvboost.cpp
  cvhaarclassifier.cpp
  cvcommon.cpp
  cvsamples.cpp
  haartraining.cpp
  )

target_link_libraries(${APP_NAME_1} 
  ${OpenCV_LIBS}
  )

# vec-img-display
set( APP_NAME_2 vec-img-display)

add_executable(${APP_NAME_2} 
  cvhaartraining.cpp
  cvboost.cpp
  cvhaarclassifier.cpp
  cvcommon.cpp
  cvsamples.cpp
  vec-img-display.cpp
  )

target_link_libraries(${APP_NAME_2} 
  ${OpenCV_LIBS}
  )

# mergevec
set( APP_NAME_3 mergevec)

add_executable(${APP_NAME_3} 
  cvhaartraining.cpp
  cvboost.cpp
  cvhaarclassifier.cpp
  cvcommon.cpp
  cvsamples.cpp
  mergevec.cpp
  )

target_link_libraries(${APP_NAME_3} 
  ${OpenCV_LIBS}
  )

# convert_cascade
set( APP_NAME_3 convert_cascade)

add_executable(${APP_NAME_3}   
  convert_cascade.c
  )

target_link_libraries(${APP_NAME_3} 
  ${OpenCV_LIBS}
  )
