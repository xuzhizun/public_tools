cmake_minimum_required(VERSION 3.2)

project(aurco_marker_generator)

find_package(OpenCV 4 REQUIRED)

add_executable(create_marker create_marker.cpp)
target_link_libraries(create_marker ${OpenCV_LIBS})
