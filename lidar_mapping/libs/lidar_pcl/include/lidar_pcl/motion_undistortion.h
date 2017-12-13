#ifndef MOTION_UNDISTORTION_H_
#define MOTION_UNDISTORTION_H_

#include <cmath>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <lidar_pcl/data_types.h>

namespace lidar_pcl
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  inline double getYawAngle(double _x, double _y)
  {
    return std::atan2(_y, _x) * 180 / 3.14159265359; // degree value
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  inline double calculateMinAngleDist(double first_angle, double second_angle) // in degree
  {
    double difference = first_angle - second_angle;
    if(difference >= 180.0)
      return difference - 360.0;
    if(difference <= -180.0)
      return difference + 360.0;
    return difference;
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void motionUndistort(pcl::PointCloud<pcl::PointXYZI>& scan, Eigen::Affine3d relative_tf)
  /* Note: This function is tested with Velodyne LiDAR scanners (VLP16, HDL32, and HDL64E)
           where the points in PointCloud are in order of firing sequence
    */
  {
    pcl::PointCloud<pcl::PointXYZI> scan_packet;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_packet_vector;
    double base_azimuth = getYawAngle((scan.begin())->x, (scan.begin())->y);
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin(); item != scan.end(); item++)
    {
      double crnt_azimuth = getYawAngle(item->x, item->y);
      if(std::fabs(calculateMinAngleDist(crnt_azimuth, base_azimuth)) < 0.01) // 0.17 degree is the typical change for 10Hz rotation
      {
        scan_packet.push_back(*item);
      }
      else // new azimuth reached
      {
        scan_packet_vector.push_back(scan_packet);
        scan_packet.clear();
        scan_packet.push_back(*item);
        base_azimuth = crnt_azimuth;
      }
    }

    scan.clear();

    Pose relative_pose;
    pcl::getTranslationAndEulerAngles(relative_tf,
                                      relative_pose.x, relative_pose.y, relative_pose.z, 
                                      relative_pose.roll, relative_pose.pitch, relative_pose.yaw);
    for(int i = 0, npackets = scan_packet_vector.size(); i < npackets; i++)
    { 
      Pose current_packet_pose(-relative_pose.x * i / npackets,
                               -relative_pose.y * i / npackets,
                               -relative_pose.z * i / npackets,
                               -relative_pose.roll * i / npackets,
                               -relative_pose.pitch * i / npackets,
                               -relative_pose.yaw * i / npackets);
      Eigen::Affine3d transform;
      pcl::getTransformation(current_packet_pose.x, 
                             current_packet_pose.y, 
                             current_packet_pose.z, 
                             current_packet_pose.roll, 
                             current_packet_pose.pitch, 
                             current_packet_pose.yaw, transform);
      pcl::PointCloud<pcl::PointXYZI> corrected_packet;
      pcl::transformPointCloud(scan_packet_vector[npackets-1-i], corrected_packet, transform);
      scan += corrected_packet;
    }
  }
} // namespace lidar_pcl

#endif // MOTION_UNDISTORTION_H_