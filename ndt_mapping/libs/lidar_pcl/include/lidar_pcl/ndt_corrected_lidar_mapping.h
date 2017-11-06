#ifndef NDT_CORRECTED_LIDAR_MAPPING_H_
#define NDT_CORRECTED_LIDAR_MAPPING_H_

// // Basic libs
// #include <chrono>
// #include <fstream>
// #include <iostream>
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
#include <lidar_pcl/struct_types.h>

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
    Key previous_key_;
    const double map_tile_width_ = 35.0;

    pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
    pcl::VoxelGrid<PointT> voxel_grid_filter_;
    double min_add_scan_shift_;
    double min_add_scan_yaw_diff_;
    double voxel_leaf_size_;
    bool initial_scan_loaded_;
    bool is_map_updated_;

    Eigen::Matrix4f tf_btol_, tf_ltob_;
    Pose previous_pose_;
    Vel previous_velocity_;
    Accel previous_accel_;
    ros::Time previous_scan_time_;


  public:
    NDTCorrectedLidarMapping();

    void setTFCalibration(double tf_x, double tf_y, double tf_z, 
                          double tf_roll, double tf_pitch, double tf_yaw);
    void addNewScan(const pcl::PointCloud<PointT> new_scan);
    void updateLocalMap(Pose current_pose);
    Vel estimateCurrentVelocity(Vel velocity, Accel acceleration, double interval);
    Pose estimateCurrentPose(Pose pose, Vel velocity, Accel acceleration, double interval);
    Eigen::Matrix4f getInitNDTPose(Pose pose);
    void correctLidarScan(pcl::PointCloud<PointT>& scan, Vel velocity, Accel acceleration, double interval);
    void doNDTMapping(const pcl::PointCloud<PointT> new_scan, const ros::Time current_scan_time);

    inline void NDTsetTransformationEpsilon(double trans_eps)
    {
      ndt_.setTransformationEpsilon(trans_eps);
    }

    inline void NDTsetStepSize(double step_size)
    {
      ndt_.setStepSize(step_size);
    }

    inline void NDTsetResolution(double ndt_res)
    {
      ndt_.setResolution(ndt_res);
    }

    inline void NDTsetMaximumIterations(double max_iter)
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

    inline double getScanInterval(ros::Time current_scan_time, ros::Time previous_scan_time)
    {
      ros::Duration scan_duration = current_scan_time - previous_scan_time;
      return scan_duration.toSec();
    }

    inline double getYawAngle(double _x, double _y)
    {
      return std::atan2(_y, _x) * 180 / 3.14159265359; // in value
    }

    inline double calculateMinAngleDist(double first, double second) // in degree
    {
      double difference = first - second;
      if(difference >= 180.0)
        return difference - 360.0;
      if(difference <= -180.0)
        return difference + 360.0;
      return difference;
    }
  };
}

#endif // NDT_CORRECTED_LIDAR_MAPPING_H_