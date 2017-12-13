cmake_minimum_required(VERSION 2.8.3)
project(ndt_mapping)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

IF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
SET(FAST_PCL_PACKAGES filters registration)
ENDIF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DOMP_FOUND)
endif()

find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND FAST_PCL_PACKAGES ndt_gpu)
endif()

find_package(catkin REQUIRED COMPONENTS
  lidar_pcl
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  velodyne_pointcloud
  # ndt_registration
  ${FAST_PCL_PACKAGES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud ${FAST_PCL_PACKAGES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

IF(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -fopenmp -Wall ${CMAKE_CXX_FLAGS}")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
# SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -fopenmp -Wall -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")
# SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -fopenmp -Wall -DUSE_GPU_PCL ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -fopenmp -Wall ${CMAKE_CXX_FLAGS}")
ENDIF(PCL_VERSION VERSION_LESS "1.7.2")

#add_executable(ndt_matching src/ndt_matching.cpp)
add_executable(mndt src/mndt.cpp)
add_executable(ndt_mapping src/ndt_mapping.cpp)
add_executable(large_scale_ndt_mapping src/large_scale_ndt_mapping.cpp)
add_executable(ground_removed_ndt_mapping src/ground_removed_ndt_mapping.cpp)
add_executable(custom_ndt_mapping src/custom_ndt_mapping.cpp)
#add_executable(d2d_ndt_mapping src/d2d_ndt_mapping.cpp)

if(CUDA_FOUND)
target_include_directories(ndt_mapping PRIVATE ${CUDA_INCLUDE_DIRS})
endif()

# target_link_libraries(ndt_matching ${catkin_LIBRARIES})
target_link_libraries(mndt ${catkin_LIBRARIES})
target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(large_scale_ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(ground_removed_ndt_mapping ${catkin_LIBRARIES})
target_link_libraries(custom_ndt_mapping ${catkin_LIBRARIES})
# target_link_libraries(d2d_ndt_mapping ${catkin_LIBRARIES})