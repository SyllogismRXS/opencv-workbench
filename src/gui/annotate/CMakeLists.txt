#set( LIB_NAME annotate )
#
#FIND_PACKAGE(Qt4 REQUIRED)
#set (QT_USE_QTDESIGNER TRUE)
#set (QT_USE_QTGUI TRUE)
#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})
#ADD_DEFINITIONS(-DQT_PLUGIN)
#ADD_DEFINITIONS(-DQT_NO_DEBUG)
#ADD_DEFINITIONS(-DQT_SHARED)
#ADD_DEFINITIONS(-DQDESIGNER_EXPORT_WIDGETS)
#ADD_DEFINITIONS(-DQT_SCRIPT_LIB)
#ADD_DEFINITIONS(-DQT_XML_LIB)
#ADD_DEFINITIONS(-DQT_GUI_LIB)
#ADD_DEFINITIONS(-DQT_CORE_LIB)
#
#file ( GLOB_RECURSE app_SOURCES *.cpp )
#file ( GLOB_RECURSE app_HEADERS *.h )
#
#file ( GLOB_RECURSE app_FORMS *.ui)
#file ( GLOB_RECURSE app_RESOURCES *.qrc)
#
#QT4_WRAP_CPP(app_HEADERS_MOC ${app_HEADERS})
#QT4_WRAP_UI(app_FORMS_HEADERS ${app_FORMS})
#QT4_ADD_RESOURCES(app_RESOURCES_RCC ${app_RESOURCES})
#
#include_directories(${CMAKE_CURRENT_BINARY_DIR})
#
#ADD_LIBRARY( ${LIB_NAME} SHARED
#  ${app_SOURCES}
#  ${app_HEADERS_MOC}
#  ${app_FORMS_HEADERS}
#  ${app_RESOURCES_RCC}
#  )



set( APP_NAME annotate)

file ( GLOB_RECURSE app_SOURCES *.cpp )
file ( GLOB_RECURSE app_HEADERS *.h )

file ( GLOB_RECURSE app_FORMS *.ui)
file ( GLOB_RECURSE app_RESOURCES *.qrc)

SET(QT_USE_QTGUI TRUE)

QT4_WRAP_CPP(app_HEADERS_MOC ${app_HEADERS})
QT4_WRAP_UI(app_FORMS_HEADERS ${app_FORMS})
QT4_ADD_RESOURCES(app_RESOURCES_RCC ${app_RESOURCES})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

include(${QT_USE_FILE})

add_definitions(${QT_DEFINITIONS})
# I'm no expert in libraries so, intuitively I'd say this is useless but it won't compile if I don't define it.
# This clearly needs to get fixed.
add_definitions(-DQDESIGNER_EXPORT_WIDGETS)

#message(${CMAKE_CURRENT_BINARY_DIR})

ADD_LIBRARY( ${APP_NAME}lib SHARED
  ${app_SOURCES}
  ${app_HEADERS_MOC}
  ${app_FORMS_HEADERS}
  ${app_RESOURCES_RCC}
)

target_link_libraries(${APP_NAME}lib
  ${QT_LIBRARIES}
  clickablelabel
  ${OpenCV_LIBS}
  cut_widgetlib
  scuba_face_labellib
  video_object_labellib
  opencv-built-in
  syllo
  track
  ${YAMLCPP_LIBRARY}
  sonar-sdk
  video_windowlib
  workbench-utils
  wb
  )

add_executable(${APP_NAME} 
  ${app_SOURCES}
  ${app_HEADERS_MOC}
  ${app_FORMS_HEADERS}
  ${app_RESOURCES_RCC}
  )

target_link_libraries(${APP_NAME} 
  ${QT_LIBRARIES}
  clickablelabel
  ${OpenCV_LIBS}
  cut_widgetlib
  scuba_face_labellib
  video_object_labellib
  opencv-built-in
  syllo
  track
  ${YAMLCPP_LIBRARY}
  sonar-sdk
  video_windowlib
  workbench-utils
  wb
  )

if (DEFINED BLUEVIEW_SDK_LIBS)
  target_link_libraries(${APP_NAME}    
    ${BLUEVIEW_SDK_LIBS}
    )
endif()
