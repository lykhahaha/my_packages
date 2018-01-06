cmake_minimum_required(VERSION 2.8.3)
project(loop_closure)

find_package(catkin REQUIRED
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES loop_closure
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -fopenmp -mavx ${CMAKE_CXX_FLAGS}")

add_executable(visualize_descriptor src/visualize_descriptor.cpp)
target_link_libraries(visualize_descriptor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})