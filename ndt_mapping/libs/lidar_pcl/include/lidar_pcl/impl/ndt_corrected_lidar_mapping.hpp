#ifndef NDT_CORRECTED_LIDAR_MAPPING_IMPL_H_
#define NDT_CORRECTED_LIDAR_MAPPING_IMPL_H_

#include <lidar_pcl/ndt_corrected_lidar_mapping.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
lidar_pcl::NDTCorrectedLidarMapping<PointT>::NDTCorrectedLidarMapping()
  : previous_key_({0, 0})
  , min_add_scan_shift_(1.0)
  , min_add_scan_yaw_diff_(0.005)
  , voxel_leaf_size_(0.1)
  , initial_scan_loaded_(false)
  , is_map_updated_(false)
  , previous_pose_({0., 0., 0., 0., 0., 0.})
  , previous_velocity_({0., 0., 0., 0., 0., 0.})
  , previous_accel_({0., 0., 0., 0., 0., 0.})
{
  local_map_.header.frame_id = "map";
  ndt_.setTransformationEpsilon(0.001);
  ndt_.setStepSize(0.05);
  ndt_.setResolution(2.5);
  ndt_.setMaximumIterations(100);
  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::setTFCalibration(double tf_x, 
                                                              double tf_y, 
                                                              double tf_z, 
                                                              double tf_roll, 
                                                              double tf_pitch, 
                                                              double tf_yaw)
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
    Key key;
    key.x = int(floor(item->x / map_tile_width_));
    key.y = int(floor(item->y / map_tile_width_));
    world_map_[key].push_back(*item);
  }

  local_map_ += new_scan;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
lidar_pcl::NDTCorrectedLidarMapping<PointT>::updateLocalMap(Pose current_pose)
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
template <typename PointT> Vel
lidar_pcl::NDTCorrectedLidarMapping<PointT>::estimateCurrentVelocity(Vel velocity, 
                                                                     Accel acceleration, 
                                                                     double interval)
{
  Vel estimated_velocity;
  estimated_velocity.x = velocity.x + acceleration.x * interval;
  estimated_velocity.y = velocity.y + acceleration.y * interval;
  estimated_velocity.z = velocity.z + acceleration.z * interval;
  estimated_velocity.roll = velocity.roll + acceleration.roll * interval;
  estimated_velocity.pitch = velocity.pitch + acceleration.pitch * interval;
  estimated_velocity.yaw = velocity.yaw + acceleration.yaw * interval;
  return estimated_velocity;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Pose
lidar_pcl::NDTCorrectedLidarMapping<PointT>::estimateCurrentPose(Pose pose, 
                                                                 Vel velocity, 
                                                                 Accel acceleration, 
                                                                 double interval)
{
  Pose estimated_pose;
  estimated_pose.x = pose.x + velocity.x * interval + 0.5 * acceleration.x * interval * interval;
  estimated_pose.y = pose.y + velocity.y * interval + 0.5 * acceleration.y * interval * interval;
  estimated_pose.z = pose.z + velocity.z * interval + 0.5 * acceleration.z * interval * interval;
  estimated_pose.roll = pose.roll + velocity.roll * interval + 0.5 * acceleration.roll * interval * interval;
  estimated_pose.pitch = pose.pitch + velocity.pitch * interval + 0.5 * acceleration.pitch * interval * interval;
  estimated_pose.yaw = pose.yaw + velocity.yaw * interval + 0.5 * acceleration.yaw * interval * interval;
  return estimated_pose;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Eigen::Matrix4f 
lidar_pcl::NDTCorrectedLidarMapping<PointT>::getInitNDTPose(Pose estimated_pose)
{
  Eigen::AngleAxisf init_rotation_x(estimated_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(estimated_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(estimated_pose.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(estimated_pose.x, estimated_pose.y, estimated_pose.z);
  return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::correctLidarScan(pcl::PointCloud<PointT>& scan, 
                                                              Vel velocity,
                                                              Accel acceleration,
                                                              double interval)
{
  // Correct LIDAR scan due to car's linear and angular motion
  pcl::PointCloud<PointT> scan_packet;
  std::vector<pcl::PointCloud<PointT>> scan_packets_vector;
  double base_azimuth = getYawAngle((scan.begin())->x, (scan.begin())->y);
  for(PointCloudConstIter item = scan.begin(); item != scan.end(); item++)
  {
    double current_azimuth = getYawAngle(item->x, item->y);
    if(std::fabs(calculateMinAngleDist(current_azimuth, base_azimuth)) < 0.01) // 0.17 degree is the typical change
    {
      scan_packet.push_back(*item);
    }
    else // new azimuth reached
    {
      scan_packets_vector.push_back(scan_packet);
      scan_packet.clear();
      scan_packet.push_back(*item);
      base_azimuth = current_azimuth;
    }
  }

  scan.clear();
  Pose current_lidar_pose = {0., 0., 0., 0., 0., 0.};
  for(int i = 0, npackets = scan_packets_vector.size(); i < npackets; i++)
  {
    double offset_time = interval * i / npackets;
    Pose this_packet_lidar_pose = {current_lidar_pose.x - velocity.x * offset_time - 0.5 * acceleration.x * interval * interval,
                                   current_lidar_pose.y - velocity.y * offset_time - 0.5 * acceleration.y * interval * interval,
                                   current_lidar_pose.z - velocity.z * offset_time - 0.5 * acceleration.z * interval * interval,
                                   current_lidar_pose.roll - velocity.roll * offset_time - 0.5 * acceleration.roll * interval * interval,
                                   current_lidar_pose.pitch - velocity.pitch * offset_time - 0.5 * acceleration.pitch * interval * interval,
                                   current_lidar_pose.yaw - velocity.yaw * offset_time - 0.5 * acceleration.yaw * interval * interval}; 

    Eigen::Affine3f transform = pcl::getTransformation(this_packet_lidar_pose.x, 
                                                       this_packet_lidar_pose.y, 
                                                       this_packet_lidar_pose.z, 
                                                       this_packet_lidar_pose.roll, 
                                                       this_packet_lidar_pose.pitch, 
                                                       this_packet_lidar_pose.yaw);
    pcl::PointCloud<PointT> corrected_packet;
    pcl::transformPointCloud(scan_packets_vector[npackets-1-i], corrected_packet, transform);
    scan += corrected_packet;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::doNDTMapping(const pcl::PointCloud<PointT> new_scan,
                                                          const ros::Time current_scan_time)
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

  // Update target map for NDT if local_map_ has been updated
  if(is_map_updated_ == true)
  {
    PointCloudPtr local_map_ptr(new pcl::PointCloud<PointT>(local_map_));
    ndt_.setInputTarget(local_map_ptr);
    is_map_updated_ = false;
  }

  // Guess initial pose for NDT iterative calc, assuming <acceleration = const> throughout
  double scan_interval = getScanInterval(current_scan_time, previous_scan_time_);
  Vel estimated_velocity = estimateCurrentVelocity(previous_velocity_, previous_accel_, scan_interval);
  Pose estimated_pose = estimateCurrentPose(previous_pose_, previous_velocity_, previous_accel_, scan_interval);
  correctLidarScan(*filtered_scan_ptr, estimated_velocity, previous_accel_, scan_interval);

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  while(/*some conditions to stop NDT*/1)
  {
    ndt_.setInputSource(filtered_scan_ptr);
    Eigen::Matrix4f init_guess = getInitNDTPose(estimated_pose);
    PointCloudPtr output_cloud(new pcl::PointCloud<PointT>);

    #ifdef USE_FAST_PCL
    ndt_.omp_align(*output_cloud, init_guess);
    double fitness_score = ndt_.getFitnessScore();
    #else
    ndt_.align(*output_cloud, init_guess);
    double fitness_score = ndt_.getFitnessScore();
    #endif

    // Check the NDT result's pose/vel/accel to compare
    t_localizer = ndt_.getFinalTransformation();

    tf::Matrix3x3 mat_l;
    mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    


    // Update localizer_pose.
    // localizer_pose.x = t_localizer(0, 3);
    // localizer_pose.y = t_localizer(1, 3);
    // localizer_pose.z = t_localizer(2, 3);
    // mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);
  }

  // Update base_link pose
  Eigen::Matrix4f t_base_link = t_localizer * tf_ltob_;
  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update ndt_pose.
  // ndt_pose.x = t_base_link(0, 3);
  // ndt_pose.y = t_base_link(1, 3);
  // ndt_pose.z = t_base_link(2, 3);
  // mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  // current_pose.x = ndt_pose.x;
  // current_pose.y = ndt_pose.y;
  // current_pose.z = ndt_pose.z;
  // current_pose.roll = ndt_pose.roll;
  // current_pose.pitch = ndt_pose.pitch;
  // current_pose.yaw = ndt_pose.yaw;


}

#endif // NDT_CORRECTED_LIDAR_MAPPING_H_