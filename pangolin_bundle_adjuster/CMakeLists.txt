cmake_minimum_required(VERSION 3.2)

project(Bundle_Adjustment)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Ceres REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Pangolin REQUIRED)

include_directories(${CERES_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
	)

add_executable( pangolin_ba main.cpp)

target_link_libraries( pangolin_ba Eigen3::Eigen ${CERES_LIBRARIES} ${Pangolin_LIBRARIES})

