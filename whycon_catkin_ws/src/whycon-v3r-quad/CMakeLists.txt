# cmake_minimum_required(VERSION <specify CMake version here>)
project(cleanflight1_wrking_V3_Copy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp src/main/flight/test.cpp src/main/flight/test.h)
add_executable(cleanflight1_wrking_V3_Copy ${SOURCE_FILES} src/main/flight/test1.cpp src/main/flight/test1.h)