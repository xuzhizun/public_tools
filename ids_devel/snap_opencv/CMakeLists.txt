cmake_minimum_required(VERSION 3.2)

project(snap_opencv)

set(CMAKE_CXX_STANDARD 14)

find_package(ids_peak REQUIRED)

find_package(ids_peak_ipl REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(snap_opencv snap_opencv.cpp)

target_link_libraries(snap_opencv ids_peak ids_peak_ipl -pthread ${OpenCV_LIBS})
