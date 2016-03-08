add_subdirectory(blank)
#add_subdirectory(displace)
add_subdirectory(relative)
#add_subdirectory(relative-2)
#add_subdirectory(vertical)
#add_subdirectory(stationarysonar)

# Push up the OPENCV_WORKBENCH_LIBS variable
set(OPENCV_WORKBENCH_LIBS ${OPENCV_WORKBENCH_LIBS} ${LIBRARY_NAME} PARENT_SCOPE)
