#ifndef LIDAR_PCL_NDT_LIDAR_MAPPING_H_
#define LIDAR_PCL_NDT_LIDAR_MAPPING_H_

// // Basic libs
// #include <chrono>
// #include <fstream>
#include <cmath>
#include <iostream>
// #include <mutex>
// #include <omp.h>
// #include <sstream>
// #include <string>
#include <unordered_map>
#include <vector>

// // Libraries for system commands
// #include <cstdlib>
// #include <unistd.h>
// #include <sys/types.h>
// #include <pwd.h>

// #include <boost/foreach.hpp> // to read bag file
// #define foreach BOOST_FOREACH

// // ROS libs
// #include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
#include <ros/time.h>
#include <ros/duration.h>
// #include <signal.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>

// #include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// // PCL & 3rd party libs
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_FAST_PCL
#include <fast_pcl/registration/ndt.h>
#include <fast_pcl/filters/voxel_grid.h>
#else
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#endif

// #include <lidar_pcl/lidar_pcl.h>
#include "lidar_pcl/data_types.h"
#include "lidar_pcl/motion_undistortion.h"

namespace lidar_pcl
{
  template<typename PointT>
  class NDTCorrectedLidarMapping
  {
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointT>::const_iterator PointCloudConstIter;

  private:
    std::unordered_map<Key, pcl::PointCloud<PointT>> world_map_;
    pcl::PointCloud<PointT> local_map_;
    PointCloudPtr transformed_scan_ptr_;
    Key previous_key_;
    const double map_tile_width_ = 35.0;

    pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
    pcl::VoxelGrid<PointT> voxel_grid_filter_;
    double min_add_scan_shift_;
    double min_add_scan_yaw_diff_;
    double voxel_leaf_size_;
    double fitness_score_;
    unsigned int added_scan_num_;
    bool initial_scan_loaded_;
    bool is_map_updated_;

    Eigen::Matrix4f tf_btol_, tf_ltob_;
    Eigen::Affine3d relative_pose_tf_;
    Pose ndt_pose_, current_pose_, previous_pose_, added_pose_;
    Pose pose_diff_;
    Pose lidar_previous_pose_;
    // Vel lidar_previous_velocity_;
    // Accel lidar_previous_accel_;
    ros::Time previous_scan_time_;
    const double CORRECTED_NDT_DISTANCE_THRESHOLD_ = 0.3; // 0.3m
    const double CORRECTED_NDT_ANGLE_THRESHOLD_ = 0.1745329; // approx 10 degree
    const unsigned int CORRECTED_NDT_ITERATION_THRESHOLD_ = 1;

    void addNewScan(const PointCloudPtr new_scan);
    void updateLocalMap(Pose current_pose);
    Vel estimateCurrentVelocity(Vel velocity, Accel acceleration, double interval);
    Pose estimateCurrentPose(Pose pose, Vel velocity, double interval);
    Eigen::Matrix4f getInitNDTPose(Pose pose);
    void correctLidarScan(pcl::PointCloud<PointT>& scan, Vel velocity, double interval);
    void motionUndistort(pcl::PointCloud<PointT>& scan, Eigen::Affine3d relative_tf);

  public:
    NDTCorrectedLidarMapping();

    void doNDTMapping(const pcl::PointCloud<PointT> new_scan, const ros::Time current_scan_time);

    void setTFCalibration(double tf_x, double tf_y, double tf_z, 
                          double tf_roll, double tf_pitch, double tf_yaw);

    inline void setNDTTransformationEpsilon(double trans_eps)
    {
      ndt_.setTransformationEpsilon(trans_eps);
    }

    inline void setNDTStepSize(double step_size)
    {
      ndt_.setStepSize(step_size);
    }

    inline void setNDTResolution(double ndt_res)
    {
      ndt_.setResolution(ndt_res);
    }

    inline void setNDTMaximumIterations(double max_iter)
    {
      ndt_.setMaximumIterations(max_iter);
    }

    inline void setMinAddScanShift(double min_add_scan_shift)
    {
      min_add_scan_shift_ = min_add_scan_shift;
    }

    inline void setMinAddScanYawDiff(double min_add_scan_yaw_diff)
    {
      min_add_scan_yaw_diff_ = min_add_scan_yaw_diff;
    }

    inline void setVoxelLeafSize(double voxel_leaf_size)
    {
      voxel_grid_filter_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    }

    inline Pose ndtPose()
    {
      return ndt_pose_;
    }

    inline Pose vehiclePose()
    {
      return current_pose_;
    }

    inline double fitnessScore()
    {
      return fitness_score_;
    }

    inline unsigned int scanNumber()
    {
      return added_scan_num_;
    }

    inline bool isMapUpdated()
    {
      return is_map_updated_;
    }

    inline pcl::PointCloud<PointT> localMap()
    {
      local_map_.header.frame_id = "map";
      return local_map_;
    }

    inline std::unordered_map<Key, pcl::PointCloud<PointT>> worldMap()
    {
      return world_map_;
    }

    inline pcl::PointCloud<PointT> transformedScan()
    {
      transformed_scan_ptr_->header.frame_id = "map";
      return *transformed_scan_ptr_;
    }

    inline int NDTConvergence()
    {
      return ndt_.hasConverged();
    }

    inline unsigned int getFinalNumIteration()
    {
      return ndt_.getFinalNumIteration();
    }

    inline double getScanInterval(ros::Time current_scan_time, ros::Time previous_scan_time)
    {
      ros::Duration scan_duration = current_scan_time - previous_scan_time;
      double seconds = scan_duration.toSec();
      if(seconds != 0)
        return seconds;
      else 
        return seconds + 1e-9; // to avoid dividing with 0
    }
  };
}

#include "lidar_pcl/impl/ndt_lidar_mapping.hpp"

#endif // LIDAR_PCL_NDT_LIDAR_MAPPING_H_