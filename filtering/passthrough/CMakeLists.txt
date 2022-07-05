cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(passthrough)

find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

#message("\n pcl_include: " ${PCL_INCLUDE_DIRS})
#message("\n pcl_lib: " ${PCL_LIBRARY_DIRS} "\n")

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (passthrough
  src/passthrough.cpp
)
target_link_libraries (passthrough
  ${PCL_LIBRARIES}
)

