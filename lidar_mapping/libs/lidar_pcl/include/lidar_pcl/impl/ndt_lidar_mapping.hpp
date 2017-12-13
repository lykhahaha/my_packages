#ifndef LIDAR_PCL_NDT_LIDAR_MAPPING_IMPL_H_
#define LIDAR_PCL_NDT_LIDAR_MAPPING_IMPL_H_

// #include <lidar_pcl/ndt_lidar_mapping.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
lidar_pcl::NDTCorrectedLidarMapping<PointT>::NDTCorrectedLidarMapping()
  : transformed_scan_ptr_(new pcl::PointCloud<PointT>())
  , previous_key_({0, 0})
  , min_add_scan_shift_(1.0)
  , min_add_scan_yaw_diff_(0.005)
  , voxel_leaf_size_(0.1)
  , fitness_score_(0.)
  , added_scan_num_(0)
  , initial_scan_loaded_(false)
  , is_map_updated_(false)
{
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

  ndt_pose_.x = tf_x;
  ndt_pose_.y = tf_y;
  ndt_pose_.z = tf_z;
  ndt_pose_.roll = tf_roll;
  ndt_pose_.pitch = tf_pitch;
  ndt_pose_.yaw = tf_yaw;

  lidar_previous_pose_.x = tf_x;
  lidar_previous_pose_.y = tf_y;
  lidar_previous_pose_.z = tf_z;
  lidar_previous_pose_.roll = tf_roll;
  lidar_previous_pose_.pitch = tf_pitch;
  lidar_previous_pose_.yaw = tf_yaw;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::addNewScan(const PointCloudPtr new_scan_ptr)
{
  for(PointCloudConstIter item = new_scan_ptr->begin(); item < new_scan_ptr->end(); item++)
  {
    Key key;
    key.x = int(floor(item->x / map_tile_width_));
    key.y = int(floor(item->y / map_tile_width_));
    world_map_[key].push_back(*item);
  }

  local_map_ += *new_scan_ptr;
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
  estimated_velocity.roll  = velocity.roll + acceleration.roll * interval;
  estimated_velocity.pitch = velocity.pitch + acceleration.pitch * interval;
  estimated_velocity.yaw   = velocity.yaw + acceleration.yaw * interval;
  return estimated_velocity;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> Pose
lidar_pcl::NDTCorrectedLidarMapping<PointT>::estimateCurrentPose(Pose pose, 
                                                                 Vel velocity, 
                                                                 // Accel acceleration, 
                                                                 double interval)
{
  Pose estimated_pose;
  estimated_pose.x = pose.x + velocity.x * interval; // + 0.5 * acceleration.x * interval * interval;
  estimated_pose.y = pose.y + velocity.y * interval; // + 0.5 * acceleration.y * interval * interval;
  estimated_pose.z = pose.z + velocity.z * interval; // + 0.5 * acceleration.z * interval * interval;
  estimated_pose.roll  = pose.roll + velocity.roll * interval; // + 0.5 * acceleration.roll * interval * interval;
  estimated_pose.pitch = pose.pitch + velocity.pitch * interval; // + 0.5 * acceleration.pitch * interval * interval;
  estimated_pose.yaw   = pose.yaw + velocity.yaw * interval; // + 0.5 * acceleration.yaw * interval * interval;
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
  // return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
lidar_pcl::NDTCorrectedLidarMapping<PointT>::correctLidarScan(pcl::PointCloud<PointT>& scan, 
                                                              Vel velocity,
                                                              // Accel acceleration,
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
  Pose current_lidar_pose; // = {0., 0., 0., 0., 0., 0.};
  for(int i = 0, npackets = scan_packets_vector.size(); i < npackets; i++)
  {
    double offset_time = interval * i / npackets;
    Pose this_packet_lidar_pose(current_lidar_pose.x - velocity.x * offset_time, // - 0.5 * acceleration.x * interval * interval,
                                current_lidar_pose.y - velocity.y * offset_time, // - 0.5 * acceleration.y * interval * interval,
                                current_lidar_pose.z - velocity.z * offset_time, // - 0.5 * acceleration.z * interval * interval,
                                current_lidar_pose.roll  - velocity.roll  * offset_time, // - 0.5 * acceleration.roll  * interval * interval,
                                current_lidar_pose.pitch - velocity.pitch * offset_time, // - 0.5 * acceleration.pitch * interval * interval,
                                current_lidar_pose.yaw   - velocity.yaw   * offset_time); // - 0.5 * acceleration.yaw   * interval * interval}; 

    Eigen::Affine3f transform = pcl::getTransformation(this_packet_lidar_pose.x, 
                                                       this_packet_lidar_pose.y, 
                                                       this_packet_lidar_pose.z, 
                                                       this_packet_lidar_pose.roll, 
                                                       this_packet_lidar_pose.pitch, 
                                                       this_packet_lidar_pose.yaw);
    // Eigen::Affine3f transform = pcl::getTransformation(0, 0, 0, 0, 0, 0);
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
  transformed_scan_ptr_->clear();

  // If this is the first scan, just push it into map then exit
  if(initial_scan_loaded_ == false)
  {
    pcl::transformPointCloud(*new_scan_ptr, *transformed_scan_ptr_, tf_btol_);
    addNewScan(transformed_scan_ptr_);
    initial_scan_loaded_ = true;
    is_map_updated_ = true;
    added_scan_num_++;
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

  // std::cout << "-----------------------------------------------------------------\n";
  // std::cout << "Prev pose: " << previous_pose_ << std::endl;
  // std::cout << "Prev vel: " << lidar_previous_velocity_ << std::endl;
  // std::cout << "Prev accel: " << lidar_previous_accel_ << std::endl;
  // Guess initial pose for NDT iterative calc, assuming <acceleration = const> throughout
  double scan_interval = getScanInterval(current_scan_time, previous_scan_time_);
  Vel lidar_estimated_velocity = lidar_previous_velocity_; // estimateCurrentVelocity(lidar_previous_velocity_, lidar_previous_accel_, scan_interval);
  Pose vehicle_estimated_pose(previous_pose_.x + pose_diff_.x,
                              previous_pose_.y + pose_diff_.y,
                              previous_pose_.z + pose_diff_.z,
                              previous_pose_.roll + pose_diff_.roll,
                              previous_pose_.pitch + pose_diff_.pitch,
                              previous_pose_.yaw + pose_diff_.yaw);

  // std::cout << "Velocity: " << lidar_estimated_velocity << std::endl;
  // std::cout << "Pose: " << lidar_estimated_pose << std::endl;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Vel ndt_velocity;
  // Accel ndt_acceleration;
  unsigned int iter_num = 0;
  while(iter_num < CORRECTED_NDT_ITERATION_THRESHOLD_)
  {
    std::cout << "-----------------------------------------------------------------\n";
    iter_num++;
    // std::cout << "Matching: " << iter_num << std::endl;
    // std::cout << "Initial guess: " << lidar_estimated_pose << std::endl;
    // std::cout << "Initial Vel: " << lidar_estimated_velocity << std::endl;
    correctLidarScan(*filtered_scan_ptr, lidar_estimated_velocity, scan_interval);
    ndt_.setInputSource(filtered_scan_ptr);
    Eigen::Matrix4f init_guess = getInitNDTPose(vehicle_estimated_pose);
    PointCloudPtr output_cloud(new pcl::PointCloud<PointT>);

    #ifdef USE_FAST_PCL
    ndt_.omp_align(*output_cloud, init_guess);
    #else
    ndt_.align(*output_cloud, init_guess);
    #endif

    // Check the NDT result's pose/vel/accel to compare
    t_localizer = ndt_.getFinalTransformation();

    tf::Matrix3x3 mat_l;
    mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    ndt_pose_.x = t_localizer(0, 3);
    ndt_pose_.y = t_localizer(1, 3);
    ndt_pose_.z = t_localizer(2, 3);
    mat_l.getRPY(ndt_pose_.roll, ndt_pose_.pitch, ndt_pose_.yaw, 1);
    // std::cout << "Result: " << ndt_pose_ << std::endl;

    // Re-update current ndt vel/accel with the newly achieved pose
    // ndt_acceleration.x     = 0; //2.0 * (ndt_pose_.x - lidar_previous_pose_.x - lidar_previous_velocity_.x * scan_interval) / (scan_interval * scan_interval);
    // ndt_acceleration.y     = 0; //2.0 * (ndt_pose_.y - lidar_previous_pose_.y - lidar_previous_velocity_.y * scan_interval) / (scan_interval * scan_interval);
    // ndt_acceleration.z     = 0; //.0 * (ndt_pose_.z - lidar_previous_pose_.z - lidar_previous_velocity_.z * scan_interval) / (scan_interval * scan_interval);
    // ndt_acceleration.roll  = 0; //2.0 * (ndt_pose_.roll - lidar_previous_pose_.roll - lidar_previous_velocity_.roll * scan_interval) / (scan_interval * scan_interval);
    // ndt_acceleration.pitch = 0; //2.0 * (ndt_pose_.pitch - lidar_previous_pose_.pitch - lidar_previous_velocity_.pitch * scan_interval) / (scan_interval * scan_interval);
    // ndt_acceleration.yaw   = 0; //2.0 * (ndt_pose_.yaw - lidar_previous_pose_.yaw - lidar_previous_velocity_.yaw * scan_interval) / (scan_interval * scan_interval);

    ndt_velocity.x     = (ndt_pose_.x - lidar_previous_pose_.x) / scan_interval; // 2.0 * (ndt_pose_.x - lidar_previous_pose_.x) / scan_interval - lidar_previous_velocity_.x;
    ndt_velocity.y     = (ndt_pose_.y - lidar_previous_pose_.y) / scan_interval; //2.0 * (ndt_pose_.y - lidar_previous_pose_.y) / scan_interval - lidar_previous_velocity_.y;
    ndt_velocity.z     = (ndt_pose_.z - lidar_previous_pose_.z) / scan_interval; //2.0 * (ndt_pose_.z - lidar_previous_pose_.z) / scan_interval - lidar_previous_velocity_.z;
    ndt_velocity.roll  = (ndt_pose_.roll - lidar_previous_pose_.roll) / scan_interval; //2.0 * (ndt_pose_.roll - lidar_previous_pose_.roll) / scan_interval - lidar_previous_velocity_.roll;
    ndt_velocity.pitch = (ndt_pose_.pitch - lidar_previous_pose_.pitch) / scan_interval; //2.0 * (ndt_pose_.pitch - lidar_previous_pose_.pitch) / scan_interval - lidar_previous_velocity_.pitch;
    ndt_velocity.yaw   = (ndt_pose_.yaw - lidar_previous_pose_.yaw) / scan_interval; //2.0 * (ndt_pose_.yaw - lidar_previous_pose_.yaw) / scan_interval - lidar_previous_velocity_.yaw;
    // std::cout << "Updated vel: " << ndt_velocity << std::endl;
    // std::cout << "Updated accel: " << ndt_acceleration << std::endl;

    if(  std::fabs(ndt_velocity.x - lidar_estimated_velocity.x) * scan_interval < CORRECTED_NDT_DISTANCE_THRESHOLD_
      && std::fabs(ndt_velocity.y - lidar_estimated_velocity.y) * scan_interval < CORRECTED_NDT_DISTANCE_THRESHOLD_
      && std::fabs(ndt_velocity.z - lidar_estimated_velocity.z) * scan_interval < CORRECTED_NDT_DISTANCE_THRESHOLD_
      && std::fabs(ndt_velocity.roll  - lidar_estimated_velocity.roll)  * scan_interval < CORRECTED_NDT_ANGLE_THRESHOLD_
      && std::fabs(ndt_velocity.pitch - lidar_estimated_velocity.pitch) * scan_interval < CORRECTED_NDT_ANGLE_THRESHOLD_
      && std::fabs(ndt_velocity.yaw   - lidar_estimated_velocity.yaw)   * scan_interval < CORRECTED_NDT_ANGLE_THRESHOLD_)
    {
      break;
    }
    else
    {
      // lidar_estimated_pose = ndt_pose_;
      lidar_estimated_velocity = ndt_velocity;
    }
  }

  #ifdef USE_FAST_PCL
  fitness_score_ = ndt_.omp_getFitnessScore();
  #else
  fitness_score_ = ndt_.getFitnessScore();
  #endif // USE_FAST_PCL

  // Update base_link pose
  pcl::transformPointCloud(*new_scan_ptr, *transformed_scan_ptr_, t_localizer);
  Eigen::Matrix4f t_base_link = t_localizer * tf_ltob_;
  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update current_pose_
  current_pose_.x = t_base_link(0, 3);
  current_pose_.y = t_base_link(1, 3);
  current_pose_.z = t_base_link(2, 3);
  mat_b.getRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw, 1);

  // Check whether to add this scan to map
  double x_diff = current_pose_.x - added_pose_.x;
  double y_diff = current_pose_.y - added_pose_.y;
  double z_diff = current_pose_.z - added_pose_.z;
  double translation_diff = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
  double rotation_diff = current_pose_.yaw - added_pose_.yaw;
  if(translation_diff >= min_add_scan_shift_ || rotation_diff >= min_add_scan_yaw_diff_)
  {
    addNewScan(transformed_scan_ptr_);
    added_scan_num_++;
    added_pose_.x = current_pose_.x;
    added_pose_.y = current_pose_.y;
    added_pose_.z = current_pose_.z;
    added_pose_.roll = current_pose_.roll;
    added_pose_.pitch = current_pose_.pitch;
    added_pose_.yaw = current_pose_.yaw;
    is_map_updated_ = true;
  }

  // Update pose_diff_
  // pose_diff_.x = ndt_pose_.x - lidar_previous_pose_.x;
  // pose_diff_.y = ndt_pose_.y - lidar_previous_pose_.y;
  // pose_diff_.z = ndt_pose_.z - lidar_previous_pose_.z;
  // pose_diff_.roll = ndt_pose_.roll - lidar_previous_pose_.roll;
  // pose_diff_.pitch = ndt_pose_.pitch - lidar_previous_pose_.pitch;
  // pose_diff_.yaw = ndt_pose_.yaw - lidar_previous_pose_.yaw;
  pose_diff_.x = current_pose_.x - previous_pose_.x;
  pose_diff_.y = current_pose_.y - previous_pose_.y;
  pose_diff_.z = current_pose_.z - previous_pose_.z;
  pose_diff_.roll = current_pose_.roll - previous_pose_.roll;
  pose_diff_.pitch = current_pose_.pitch - previous_pose_.pitch;
  pose_diff_.yaw = current_pose_.yaw - previous_pose_.yaw;

  // Update <previous> values
  previous_pose_.x = current_pose_.x;
  previous_pose_.y = current_pose_.y;
  previous_pose_.z = current_pose_.z;
  previous_pose_.roll = current_pose_.roll;
  previous_pose_.pitch = current_pose_.pitch;
  previous_pose_.yaw = current_pose_.yaw;

  lidar_previous_pose_.x = ndt_pose_.x;
  lidar_previous_pose_.y = ndt_pose_.y;
  lidar_previous_pose_.z = ndt_pose_.z;
  lidar_previous_pose_.roll = ndt_pose_.roll;
  lidar_previous_pose_.pitch = ndt_pose_.pitch;
  lidar_previous_pose_.yaw = ndt_pose_.yaw;

  lidar_previous_velocity_.x = ndt_velocity.x;
  lidar_previous_velocity_.y = ndt_velocity.y;
  lidar_previous_velocity_.z = ndt_velocity.z;
  lidar_previous_velocity_.roll = ndt_velocity.roll;
  lidar_previous_velocity_.pitch = ndt_velocity.pitch;
  lidar_previous_velocity_.yaw = ndt_velocity.yaw;

  previous_scan_time_.sec = current_scan_time.sec;
  previous_scan_time_.nsec = current_scan_time.nsec;

  // Finally, update local_map_
  updateLocalMap(current_pose_);
}

#endif // LIDAR_PCL_NDT_LIDAR_MAPPING_IMPL_H_