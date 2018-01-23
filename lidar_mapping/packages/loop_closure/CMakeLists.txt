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


macro(add_new_node name)
  add_executable(${name} src/${name}.cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endmacro()

add_new_node(visualize_descriptor)
add_new_node(distance_matrix)
add_new_node(binary_reader)
add_new_node(loop_detector)
add_new_node(loop_closure)

# add_executable(visualize_descriptor src/visualize_descriptor.cpp)
# add_executable(distance_matrix src/distance_matrix.cpp)
# add_executable(binary_reader src/binary_reader.cpp)
# add_executable(loop_detector src/loop_detector.cpp)
# target_link_libraries(visualize_descriptor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(distance_matrix ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(binary_reader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(loop_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})