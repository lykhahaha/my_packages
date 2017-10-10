cmake_minimum_required(VERSION 2.8.3)
project(ndt_mapping)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  velodyne_pointcloud
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud autoware_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -fopenmp -Wall ${CMAKE_CXX_FLAGS}")

#add_executable(ndt_matching src/ndt_matching.cpp)
add_executable(mndt src/mndt.cpp)
#add_executable(ndt_mapping src/ndt_mapping.cpp)

#target_link_libraries(ndt_matching ${catkin_LIBRARIES})
target_link_libraries(mndt ${catkin_LIBRARIES})
#target_link_libraries(ndt_mapping ${catkin_LIBRARIES})

#add_dependencies(ndt_matching autoware_msgs_generate_messages_cpp)
add_dependencies(mndt ${catkin_LIBRARIES})
#add_dependencies(ndt_mapping autoware_msgs_generate_messages_cpp)
