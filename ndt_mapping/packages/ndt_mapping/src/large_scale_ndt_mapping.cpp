// Basic libs
// #include <chrono>
// #include <fstream>
#include <iostream>
// #include <mutex>
// #include <omp.h>
// #include <sstream>
#include <string>
// #include <unordered_map>
#include <vector>
#include <cstdlib>
// #include <unistd.h>
// #include <sys/types.h>
// #include <pwd.h>
// #include <time.h>

// #include <boost/foreach.hpp> // to read bag file
// #define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <ros/time.h>
// #include <ros/duration.h>
// #include <signal.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/Point.h>
// #include <visualization_msgs/Marker.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>

// #include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// // PCL & 3rd party libs
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

// #include <Eigen/Dense>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct velocity
{
  double x;
  double y;
  double z;
};

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static const int max_iter = 300;       // Maximum iterations
static const float ndt_res = 4.0;      // Resolution
static const double step_size = 0.05;   // Step size
static const double trans_eps = 0.0001;  // Transformation epsilon
static const double min_scan_range = 3.0;

// void doNDT()
// {

// }
inline double getYawAngle(double _x, double _y)
{
  return std::atan2(_y, _x) * 180 / 3.14159265359; // degree value
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

int main(int argc, char** argv)
{
  // ROS initialization
  ros::init(argc, argv, "road_surface");
  ros::NodeHandle nh;
  ros::Publisher src_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1, true);

  // if(argc < 3)
  // {
  //   std::cout << "Need 2 pcd files." << std::endl;
  //   std::cout << "Usage: rosrun ndt_mapping large_scale_ndt_mapping \"source.pcd\" \"target.pcd\"" << std::endl;
  //   return -1;
  // }

  // Get PCD
  std::string source_file = "/home/zwu/20oct-1114/scan190.pcd";
  std::string map_file = "/home/zwu/20oct-1114/scan189.pcd";
  pcl::PointCloud<pcl::PointXYZI> src_scan, src_map;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(source_file, src_scan) == -1)
  {
    std::cout << "Couldn't read " << source_file << "." << std::endl;
    return(-1);
  }

  if(pcl::io::loadPCDFile<pcl::PointXYZI>(map_file, src_map) == -1)
  {
    std::cout << "Couldn't read " << map_file << "." << std::endl;
    return(-1);
  }

  // Pre-process data
  pcl::PointCloud<pcl::PointXYZI> scan, map;
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src_scan.begin(); item != src_scan.end(); item++)
    if (sqrt(pow((double)item->x, 2.0) + pow((double)item->y, 2.0)) > min_scan_range)
      scan.push_back(*item);

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src_map.begin(); item != src_map.end(); item++)
    if (sqrt(pow((double)item->x, 2.0) + pow((double)item->y, 2.0)) > min_scan_range)
      map.push_back(*item);

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputSource(scan_ptr);
  ndt.setInputTarget(map_ptr);

  double scan_interval = 0.100085; // sec
  velocity prev_vel = {-5.994904, 0.999151, 0.0};
  velocity crnt_vel = prev_vel;

  Eigen::AngleAxisf init_rotation_x(0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(0, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(-0.6, 0.1, 0);

  Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // Correct map scan using prev_vel
  pcl::PointCloud<pcl::PointXYZI> scan_packet;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_packets_vector;
  double base_azimuth = getYawAngle((map.begin())->x, (map.begin())->y);
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = map.begin(); item != map.end(); item++)
  {
    double crnt_azimuth = getYawAngle(item->x, item->y);
    if(std::fabs(calculateMinAngleDist(crnt_azimuth, base_azimuth)) < 0.01) // 0.17 degree is the typical change
    {
      scan_packet.push_back(*item);
    }
    else // new azimuth reached
    {
      std::cout << base_azimuth << " >>>> " << crnt_azimuth << std::endl;
      scan_packets_vector.push_back(scan_packet);
      scan_packet.clear();
      scan_packet.push_back(*item);
      base_azimuth = crnt_azimuth;
    }
  }

  // assuming T'' = 0 and only linear motion
  map.clear();
  pose crnt_pose = {0, 0, 0, 0, 0, 0};
  for(int i = 0, npackets = scan_packets_vector.size(); i < npackets; i++)
  {
    double offset_time = scan_interval * i / npackets;
    pose this_packet_pose = {crnt_pose.x - prev_vel.x * offset_time,
                             crnt_pose.y - prev_vel.y * offset_time,
                             crnt_pose.z - prev_vel.z * offset_time,
                             crnt_pose.roll, crnt_pose.pitch, crnt_pose.yaw}; 

    Eigen::Affine3f transform = pcl::getTransformation(this_packet_pose.x, 
                                                       this_packet_pose.y, 
                                                       this_packet_pose.z, 
                                                       this_packet_pose.roll, 
                                                       this_packet_pose.pitch, 
                                                       this_packet_pose.yaw);
    pcl::PointCloud<pcl::PointXYZI> corrected_packet;
    pcl::transformPointCloud(scan_packets_vector[npackets-1-i], corrected_packet, transform);
    map += corrected_packet;
  }

  // Start loop mapping/correcting cloud
  // ndt.align(*output_cloud, init_guess);
  // double fitness_score = ndt.getFitnessScore();

  // Eigen::Matrix4f t_localizer = ndt.getFinalTransformation();
  // tf::Matrix3x3 mat_l;
  // mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
  //                static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
  //                static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
  //                static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
  //                static_cast<double>(t_localizer(2, 2)));

  // double x,y,z,roll,pitch,yaw;
  // x = t_localizer(0, 3);
  // y = t_localizer(1, 3);
  // z = t_localizer(2, 3);
  // mat_l.getRPY(roll, pitch, yaw, 1);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
  // map += *transformed_scan_ptr;

  // std::cout << "Fitness score: " << fitness_score << std::endl;
  // std::cout << "Pose: (" << x << ", " << y << ", " << z << ", "
  //                        << roll << ", " << pitch << ", " << yaw << ")" << std::endl;



  // Convert to ROS msgs and publish
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(map, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  src_pub.publish(*map_msg_ptr);

  // pcl::toROSMsg(dst, *cloud_msg_ptr);
  // cloud_msg_ptr->header.frame_id = "map";
  // dst_pub.publish(*cloud_msg_ptr);

  ros::spin();
}