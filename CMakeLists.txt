cmake_minimum_required(VERSION 3.0)
project(lane_detection)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(lane_detection lane_detection.cpp)
target_link_libraries(lane_detection ${OpenCV_LIBS})
