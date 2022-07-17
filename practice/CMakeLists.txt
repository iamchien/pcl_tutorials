cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(practice)

add_compile_options(-std=c++1z) 

find_package(PCL REQUIRED)
set(Open3D_INCLUDE_DIRS ~/open3d_install/include)
set(Open3D_LIBRARY_DIRS ~/open3d_install/lib)
set(Open3D_DIR ~/open3d_install/lib/cmake/Open3D)
find_package(Open3D REQUIRED)

#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS} -Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0")
#set(CMAKE_CXX_FLAGS_RELEASE "-g0 -O3")

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Open3D_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_process
  src/pcl_process.cpp
)
target_link_libraries (pcl_process
  ${PCL_LIBRARIES}
)
