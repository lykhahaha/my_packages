cmake_minimum_required(VERSION 2.8.3)
project(lidar_pcl)

find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED)

set(SUBSYS_NAME lidar_pcl)
set(SUBSYS_DESC "Point cloud library for LIDAR mapping")
# set(SUBSYS_DEPS )

set(LIB_NAME "lidar_pcl")

catkin_package(
  #DEPENDS ${SUBSYS_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${LIB_NAME}
  )

set(srcs
  src/data_types.cpp
  src/lidar_pcl.cpp
  src/motion_undistortion.cpp
  src/ndt_lidar_mapping.cpp
)

set(incs 
  "include/lidar_pcl/data_types.h"
  "include/lidar_pcl/lidar_pcl.h"
  "include/lidar_pcl/motion_undistortion.h"
  "include/lidar_pcl/ndt_lidar_mapping.h"
)

set(impl_incs 
  "include/lidar_pcl/impl/ndt_lidar_mapping.hpp"
)

include_directories(${PCL_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_library("${LIB_NAME}" ${srcs} ${incs} ${impl_incs})

target_link_libraries("${LIB_NAME}" ${PCL_LIBRRIES})