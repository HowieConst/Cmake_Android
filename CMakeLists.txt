cmake_minimum_required(VERSION 3.28.4)

set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_STANDARD 20)

project(PathFinding)
set(CMAKE_BUILD_TYPE Release)
file(GLOB_RECURSE SOURCE_FILES  src/*.cpp src/*.c)
add_library(PathFinding SHARED ${SOURCE_FILES})
target_include_directories(PathFinding PUBLIC include)

