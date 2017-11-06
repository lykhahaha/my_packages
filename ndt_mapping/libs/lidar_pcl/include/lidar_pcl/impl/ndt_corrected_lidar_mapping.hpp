#ifndef NDT_CORRECTED_LIDAR_MAPPING_IMPL_H_
#define NDT_CORRECTED_LIDAR_MAPPING_IMPL_H_

#include <lidar_pcl/ndt_corrected_lidar_mapping.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
lidar_pcl::NDTCorrectedLidarMapping<PointT>::NDTCorrectedLidarMapping(): previous_key_({0, 0}),
                                                                         min_add_scan_shift_(1.0),
                                                                         min_add_scan_yaw_diff_(0.005),
                                                                         voxel_leaf_size_(0.1),
                                                                         initial_scan_loaded_(false),
                                                                         is_map_updated_(false)
{
  ndt_.setTransformationEpsilon(0.001);
  ndt_.setStepSize(0.05);
  ndt_.setResolution(2.5);
  ndt_.setMaximumIterations(100);
  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::setTFCalibration(
    double tf_x, double tf_y, double tf_z, double tf_roll, double tf_pitch, double tf_yaw)
{
  Eigen::Translation3f tl_btol(tf_x, tf_y, tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * tf_x, (-1.0) * tf_y, (-1.0) * tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob_ = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::addNewScan(const pcl::PointCloud<PointT> new_scan)
{
  for(PointCloudConstIter item = new_scan.begin(); item < new_scan.end(); item++)
  {
    // Get 2D point
    Key key;
    key.x = int(floor(item->x / map_tile_width_));
    key.y = int(floor(item->y / map_tile_width_));

    world_map_[key].push_back(*item);
  }

  local_map_ += new_scan;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
lidar_pcl::NDTCorrectedLidarMapping<PointT>::updateLocalMap(pose current_pose)
{
  // Update local_key
  Key local_key = {int(floor(current_pose.x / map_tile_width_)),  // .x
                   int(floor(current_pose.y / map_tile_width_))}; // .y

  // Only update local_map_ through world_map_ only if local_key_ changes
  if(local_key != previous_key_)
  {
    // Get local_map_, a 5x5 tile map with the center being the local_key_
    local_map_.clear();
    Key tmp_key;
    for(int x = local_key.x - 2, x_max = local_key.x + 2; x <= x_max; x++)
      for(int y = local_key.y - 2, y_max = local_key.y + 2; y <= y_max; y++)
      {
        tmp_key.x = x;
        tmp_key.y = y;
        local_map_ += world_map_[tmp_key];
      }

    // Update key
    previous_key_ = local_key;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::doNDTMapping(const pcl::PointCloud<PointT> new_scan)
{
  PointCloudPtr new_scan_ptr(new pcl::PointCloud<PointT>(new_scan));
  PointCloudPtr transformed_scan_ptr(new pcl::PointCloud<PointT>());

  // If this is the first scan, just push it into map then exit
  if(initial_scan_loaded_ == false)
  {
    pcl::transformPointCloud(*new_scan_ptr, *transformed_scan_ptr, tf_btol_);
    addNewScan(*transformed_scan_ptr);
    initial_scan_loaded_ = true;
    return;
  }

  // Apply voxelgrid filter to new scan
  PointCloudPtr filtered_scan_ptr(new pcl::PointCloud<PointT>());
  voxel_grid_filter_.setInputCloud(new_scan_ptr);
  voxel_grid_filter_.filter(*filtered_scan_ptr);
  ndt_.setInputSource(filtered_scan_ptr);

  // Update target map for NDT if local_map_ has been updated
  if(is_map_updated_ == true)
  {
    PointCloudPtr local_map_ptr(new pcl::PointCloud<PointT>(local_map_));
    ndt_.setInputTarget(local_map_ptr);
    is_map_updated_ = false;
  }
}

#endif // NDT_CORRECTED_LIDAR_MAPPING_H_