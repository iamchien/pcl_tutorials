cmake_minimum_required(VERSION 2.8.3)

project(remove_outliers)

find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (remove_outliers
  src/remove_outliers.cpp
)

add_executable (conditional_removal
  src/conditional_removal.cpp
)

target_link_libraries (remove_outliers
  ${PCL_LIBRARIES}
)

target_link_libraries (conditional_removal
  ${PCL_LIBRARIES}
)
