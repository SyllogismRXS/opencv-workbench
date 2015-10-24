set( APP_NAME hog-train-test)

add_executable(${APP_NAME} 
  fhog_object_detector_ex.cpp
  )

target_link_libraries(${APP_NAME} 
  ${DLIB_LIBRARY}  
  blas
  lapack
  )
