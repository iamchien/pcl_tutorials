cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(convex_hull_2d)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (convex_hull_2d src/convex_hull_2d.cpp)
target_link_libraries (convex_hull_2d ${PCL_LIBRARIES})
