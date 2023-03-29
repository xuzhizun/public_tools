cmake_minimum_required(VERSION 3.2)

project(myReceiveData)

set(CMAKE_C++_STANDARD 11)

include_directories(xspublic)

link_directories(xspublic/xscontroller
xspublic/xscommon
xspublic/xstypes
)


add_executable(myReceiveData myReceiveData.cpp)

target_link_libraries(myReceiveData pthread  xscontroller xscommon xstypes rt dl)



