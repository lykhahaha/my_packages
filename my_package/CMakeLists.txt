cmake_minimum_required(VERSION 2.8.3)
project(my_package)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_generation
  pcl_ros
  # pcl_conversion
  roscpp
  roslaunch
  rospy
  sensor_msgs
  std_msgs
  tf
)
#find_package(OpenCV REQUIRED core ocl)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
roslaunch_add_file_check(launch)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  tfMap.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/offsetMapping.cfg
  cfg/transformPoints.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES base_vision
  CATKIN_DEPENDS std_msgs message_runtime autoware_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

## Declare a C++ library
# add_library(base_vision
#   src/${PROJECT_NAME}/base_vision.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -fopenmp -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")

macro(add_new_node name)
  add_executable(${name} src/${name}.cpp)
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endmacro()

macro(add_new_opencv_node name)
  add_executable(${name} src/${name}.cpp)
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
  target_link_libraries(${name} ${catkin_LIBRARIES})
endmacro()

macro(add_new_autoware_node name)
  add_executable(${name} src/${name}.cpp)
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES})
endmacro()

macro(add_new_ceres_node name)
  add_executable(${name} src/${name}.cpp)
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${CERES_LIBRARIES} glog ${GFLAGS_LIBRARIES})
endmacro()

## Add nodes
add_new_ceres_node(pose_optimization_mapping)
add_new_ceres_node(pose_optimization_with_normals_regularization)
add_new_opencv_node(gps2img)
add_new_opencv_node(gps_to_csv)
add_new_opencv_node(imu_to_csv)
add_new_node(imu_publisher)
add_new_node(cloud_frame_conversion)
add_new_autoware_node(offset_mapping)
add_new_autoware_node(csv_offset_mapping)
add_new_node(my_pcl)
add_new_node(submap_merger)
add_new_node(async_submap_merger)
add_new_node(ply_colorizer)
add_new_node(pcd_to_nvtxt)
add_new_autoware_node(transform_pcd)
add_new_autoware_node(pose_data_analysis)
add_new_autoware_node(import_to_bag)
add_new_autoware_node(import_to_bag_nostamp)
add_new_autoware_node(scan_preprocessing)
add_new_autoware_node(map_splitter)
add_new_autoware_node(pcd_merger)
add_new_autoware_node(road_surface)
add_new_node(transform_trajectory)
add_new_node(get_perspective_scan)
add_new_node(quickscript)
add_new_node(scan_from_bag_to_txt)
add_new_node(visualize_pose)