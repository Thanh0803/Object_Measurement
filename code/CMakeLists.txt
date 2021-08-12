cmake_minimum_required (VERSION 2.8)
project (hello)

find_package (OpenCV REQUIRED)
find_package (realsense2 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_definitions(-std=c++11)


add_executable (main main.cpp)
target_link_libraries (main ${OpenCV_LIBS} ${realsense2_LIBRARY})




