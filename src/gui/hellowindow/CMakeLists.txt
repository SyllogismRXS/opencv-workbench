set( APP_NAME hellowindow)

file ( GLOB_RECURSE app_SOURCES *.cpp )
file ( GLOB_RECURSE app_HEADERS *.h )

QT4_WRAP_CPP(app_HEADERS_MOC ${app_HEADERS})

add_executable(${APP_NAME} 
  ${app_SOURCES}
  ${app_HEADERS_MOC}
  )

target_link_libraries(${APP_NAME} ${QT_LIBRARIES})
