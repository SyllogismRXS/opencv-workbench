#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
#SET (LIBRARY_NAME sam-math)
#SET (LIB_MAJOR 0)
#SET (LIB_MINOR 0)
#SET (LIB_RELEASE 1)
#
#file( GLOB SRCS *.cpp)
#file( GLOB HDRS *.h)
#
#ADD_LIBRARY(${LIBRARY_NAME} SHARED 
#  ${SRCS}
#  ${HDRS}
#  )
#
#target_link_libraries(${LIBRARY_NAME} 
#  )
#
#SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})
#
#set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION ${LIB_MAJOR} VERSION ${_soversion})
#
#INSTALL(TARGETS ${LIBRARY_NAME}
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#)
#
#set(SAM_LIBS ${SAM_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
