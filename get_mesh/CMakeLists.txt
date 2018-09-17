cmake_minimum_required(VERSION 2.8.3)
project(get_mesh)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL 1.7 REQUIRED)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${pcl_surface})
add_definitions(${PCL_DEFINITIONS})

add_executable(get_mesh_node src/get_mesh.cpp)



target_link_libraries(get_mesh_node ${catkin_LIBRARIES})
target_link_libraries (get_mesh_node ${PCL_LIBRARIES})
