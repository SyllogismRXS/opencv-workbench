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
