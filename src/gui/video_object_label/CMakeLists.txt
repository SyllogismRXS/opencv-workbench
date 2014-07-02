set( APP_NAME video_object_label)

file ( GLOB_RECURSE app_SOURCES *.cpp )
file ( GLOB_RECURSE app_HEADERS *.h )

file ( GLOB_RECURSE app_FORMS *.ui)
file ( GLOB_RECURSE app_RESOURCES *.qrc)

QT4_WRAP_CPP(app_HEADERS_MOC ${app_HEADERS})
QT4_WRAP_UI(app_FORMS_HEADERS ${app_FORMS})
QT4_ADD_RESOURCES(app_RESOURCES_RCC ${app_RESOURCES})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

ADD_LIBRARY( ${APP_NAME}lib SHARED
  ${app_SOURCES}
  ${app_HEADERS_MOC}
  ${app_FORMS_HEADERS}
  ${app_RESOURCES_RCC}
)


add_executable(${APP_NAME} 
  ${app_SOURCES}
  ${app_HEADERS_MOC}
  ${app_FORMS_HEADERS}
  ${app_RESOURCES_RCC}
  )

target_link_libraries(${APP_NAME} 
  ${QT_LIBRARIES}
  ${OpenCV_LIBS}
  opencv-workbench-utils
  )
