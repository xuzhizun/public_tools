cmake_minimum_required(VERSION 2.8)

project(synchronize)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

add_executable(synchronize_single associate_single.cpp )
target_link_libraries(synchronize_single Eigen3::Eigen)

add_executable(synchronize_stereo associate_stereo.cpp )
target_link_libraries(synchronize_stereo Eigen3::Eigen)

add_executable(synchronize_no_nav associate2.cpp )
target_link_libraries(synchronize_no_nav Eigen3::Eigen)

add_executable(synchronize_stereo_no_nav associate_stereo_no_nav.cpp )
target_link_libraries(synchronize_stereo_no_nav Eigen3::Eigen)
