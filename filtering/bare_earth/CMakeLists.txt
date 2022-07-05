cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(bare_earth)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (bare_earth src/bare_earth.cpp)
target_link_libraries (bare_earth ${PCL_LIBRARIES})
