cmake_minimum_required(VERSION 2.8.3)

project(convert_type)

find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (obj2pcd
  src/obj2pcd.cpp
)
target_link_libraries (obj2pcd
  ${PCL_LIBRARIES}
)

add_executable (obj2ply
  src/obj2ply.cpp
)
target_link_libraries (obj2ply
  ${PCL_LIBRARIES}
)

add_executable (obj2vtk
  src/obj2vtk.cpp
)
target_link_libraries (obj2vtk
  ${PCL_LIBRARIES}
)

add_executable (pcd2ply
  src/pcd2ply.cpp
)
target_link_libraries (pcd2ply
  ${PCL_LIBRARIES}
)

add_executable (pcd2vtk
  src/pcd2vtk.cpp
)
target_link_libraries (pcd2vtk
  ${PCL_LIBRARIES}
)

add_executable (ply2pcd
  src/ply2pcd.cpp
)
target_link_libraries (ply2pcd
  ${PCL_LIBRARIES}
)

add_executable (ply2vtk
  src/ply2vtk.cpp
)
target_link_libraries (ply2vtk
  ${PCL_LIBRARIES}
)

add_executable (vtk2obj
  src/vtk2obj.cpp
)
target_link_libraries (vtk2obj
  ${PCL_LIBRARIES}
)

add_executable (vtk2pcd
  src/vtk2pcd.cpp
)
target_link_libraries (vtk2pcd
  ${PCL_LIBRARIES}
)

add_executable (vtk2ply
  src/vtk2ply.cpp
)
target_link_libraries (vtk2ply
  ${PCL_LIBRARIES}
)
