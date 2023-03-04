cmake_minimum_required(VERSION 3.2)

project(continuous_sampling_opencv)

set(CMAKE_CXX_STANDARD 14)

find_package(ids_peak REQUIRED)

find_package(ids_peak_ipl REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(continuous_sampling continuous_sampling.cpp)

target_link_libraries(continuous_sampling ids_peak ids_peak_ipl -pthread ${OpenCV_LIBS})
