cmake_minimum_required(VERSION 2.8.3)
project(algorithm)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL 1.7 REQUIRED)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS}
					include/${PROJECT_NAME}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(algorithm_node src/algorithm.cpp src/SimplexSolver.cpp src/exception.cpp)

target_link_libraries(algorithm_node ${catkin_LIBRARIES})
target_link_libraries (algorithm_node ${PCL_LIBRARIES})
