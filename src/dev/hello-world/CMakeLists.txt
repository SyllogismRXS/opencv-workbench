set( APP_NAME hello-world)

file ( GLOB_RECURSE SRCS *.cpp )
add_executable(${APP_NAME} ${SRCS})
