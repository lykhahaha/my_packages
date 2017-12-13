#include <pcl/point_types.h>
#include "lidar_pcl/ndt_lidar_mapping.h"
#include "lidar_pcl/impl/ndt_lidar_mapping.hpp"

template class PCL_EXPORTS lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZ>;
template class PCL_EXPORTS lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZI>;
template class PCL_EXPORTS lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZRGB>;