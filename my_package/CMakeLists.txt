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

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")

## Add nodes 
add_executable(gps2img src/gps2img.cpp)
add_dependencies(gps2img ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(gps2img ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(gps_to_csv src/gps_to_csv.cpp)
add_dependencies(gps_to_csv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(gps_to_csv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(imu_to_csv src/imu_to_csv.cpp)
add_dependencies(imu_to_csv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_to_csv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(imu_publisher src/imu_publisher.cpp)
add_dependencies(imu_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_publisher ${catkin_LIBRARIES})

add_executable(cloud_frame_conversion src/cloud_frame_conversion.cpp)
add_dependencies(cloud_frame_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(cloud_frame_conversion ${catkin_LIBRARIES})

add_executable(offset_mapping src/offset_mapping.cpp)
add_dependencies(offset_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(offset_mapping ${catkin_LIBRARIES})

add_executable(csv_offset_mapping src/csv_offset_mapping.cpp)
add_dependencies(csv_offset_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(csv_offset_mapping ${catkin_LIBRARIES})

add_executable(my_pcl src/my_pcl.cpp)
add_dependencies(my_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(my_pcl ${catkin_LIBRARIES})

add_executable(submap_merger src/submap_merger.cpp)
add_dependencies(submap_merger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(submap_merger ${catkin_LIBRARIES})

add_executable(async_submap_merger src/async_submap_merger.cpp)
add_dependencies(async_submap_merger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(async_submap_merger ${catkin_LIBRARIES})

add_executable(ply_colorizer src/ply_colorizer.cpp)
add_dependencies(ply_colorizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(ply_colorizer ${catkin_LIBRARIES})

add_executable(pcd_to_nvtxt src/pcd_to_nvtxt.cpp)
add_dependencies(pcd_to_nvtxt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(pcd_to_nvtxt ${catkin_LIBRARIES})

add_executable(pose_optimization_mapping src/pose_optimization_mapping.cpp)
add_dependencies(pose_optimization_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(pose_optimization_mapping ${catkin_LIBRARIES} ${CERES_LIBRARIES} glog ${GFLAGS_LIBRARIES})

add_executable(transform_pcd src/transform_pcd.cpp)
add_dependencies(transform_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(transform_pcd ${catkin_LIBRARIES})

add_executable(pose_data_analysis src/pose_data_analysis.cpp)
add_dependencies(pose_data_analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(pose_data_analysis ${catkin_LIBRARIES})

add_executable(import_to_bag src/import_to_bag.cpp)
add_dependencies(import_to_bag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(import_to_bag ${catkin_LIBRARIES})

add_executable(import_to_bag_nostamp src/import_to_bag_nostamp.cpp)
add_dependencies(import_to_bag_nostamp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(import_to_bag_nostamp ${catkin_LIBRARIES})

add_executable(scan_preprocessing src/scan_preprocessing.cpp)
add_dependencies(scan_preprocessing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(scan_preprocessing ${catkin_LIBRARIES})

add_executable(map_splitter src/map_splitter.cpp)
add_dependencies(map_splitter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(map_splitter ${catkin_LIBRARIES})

add_executable(pcd_merger src/pcd_merger.cpp)
add_dependencies(pcd_merger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(pcd_merger ${catkin_LIBRARIES})

add_executable(road_surface src/road_surface.cpp)
add_dependencies(road_surface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg autoware_msgs_generate_messages_cpp)
target_link_libraries(road_surface ${catkin_LIBRARIES})

add_executable(pose_optimization_with_normals_regularization src/pose_optimization_with_normals_regularization.cpp)
add_dependencies(pose_optimization_with_normals_regularization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(pose_optimization_with_normals_regularization ${catkin_LIBRARIES} ${CERES_LIBRARIES} glog ${GFLAGS_LIBRARIES})
