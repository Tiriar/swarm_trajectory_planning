
cmake_minimum_required(VERSION 2.8)

PROJECT(vtk_voro)

set(VTK_DIR C:/Users/Tiriar/Downloads/VTK_Voro_Libs/VTK-build)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(vtk_voro main.cc  vtktool.cc files.cc  functional.cc  interactor2d.cc   point.cc  segment.cc polygon.cc  doubles.cc voronoi.cc)



if(VTK_LIBRARIES)
  target_link_libraries(vtk_voro ${VTK_LIBRARIES})
else()
  target_link_libraries(vtk_voro vtkHybrid)
endif()
