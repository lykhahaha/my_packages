#include <pcl/point_types.h>
#include "lidar_pcl/ndt_corrected_lidar_mapping.h"
#include "lidar_pcl/impl/ndt_corrected_lidar_mapping.hpp"

template class PCL_EXPORTS lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZ>;
template class PCL_EXPORTS lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZI>;
template class PCL_EXPORTS lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZRGB>;