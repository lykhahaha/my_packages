cmake_minimum_required(VERSION 2.8.3)
project(icp_mapping)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  lidar_pcl
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  velodyne_pointcloud
)

roslaunch_add_file_check(launch)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs lidar_pcl
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -fopenmp -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(icp_mapping src/icp_mapping.cpp)
add_executable(icp_point2plane_mapping src/icp_point2plane_mapping.cpp)
add_executable(queue_counter src/queue_counter.cpp)

target_link_libraries(icp_mapping ${catkin_LIBRARIES})
target_link_libraries(icp_point2plane_mapping ${catkin_LIBRARIES})
target_link_libraries(queue_counter ${catkin_LIBRARIES})

add_dependencies(icp_mapping autoware_msgs_generate_messages_cpp)
