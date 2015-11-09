add_subdirectory(blank)
add_subdirectory(displace)
add_subdirectory(relative)
add_subdirectory(vertical)

# Push up the OPENCV_WORKBENCH_LIBS variable
set(OPENCV_WORKBENCH_LIBS ${OPENCV_WORKBENCH_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
