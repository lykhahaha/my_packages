cmake_minimum_required(VERSION 2.8.3)
project(lidar_pcl)

find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED)

set(SUBSYS_NAME lidar_pcl)
set(SUBSYS_DESC "Point cloud library for LIDAR")
# set(SUBSYS_DEPS )

set(LIB_NAME "lidar_pcl")

catkin_package(
  #DEPENDS ${SUBSYS_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${LIB_NAME}
  )

set(srcs
  src/lidar_pcl.cpp
)

set(incs 
  "include/lidar_pcl/lidar_pcl.h"
)

# set(impl_incs 
#   "include/impl/lidar_pcl.hpp"
# )

include_directories(${PCL_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")

# add_library("${LIB_NAME}" ${srcs} ${incs} ${impl_incs})
add_library("${LIB_NAME}" ${srcs} ${incs})

target_link_libraries("${LIB_NAME}" ${PCL_LIBRRIES})