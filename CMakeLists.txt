cmake_minimum_required(VERSION 2.8.3)
project(pointcloud2_segments_viz)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roboskel_msgs
  pcl_conversions
  sensor_msgs
  std_msgs
  pcl_ros
  roscpp
  tf
)

find_package(PCL 1.8.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud2_segments_viz src/pointcloud2_segments_viz.cpp)
target_link_libraries(pointcloud2_segments_viz ${catkin_LIBRARIES} ${PCL_LIBRARIES})
