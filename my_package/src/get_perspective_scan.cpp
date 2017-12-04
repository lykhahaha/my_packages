#include <ros/ros.h>
// #include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
// #include <my_package/transformPointsConfig.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_cloud.h>
#include <pcl/common/common.h>
// #include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <fstream>
// #include <sstream>
#include <string>
#include <vector>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_perspective_scan");
  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/my_scan", 100, true);
  
  // Load input
  // std::string filename = argv[1];
  std::string cloudfile = "/home/zwu/1dec-0859/1dec-carpark.pcd";
  pcl::PointCloud<pcl::PointXYZI> src;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(cloudfile, src) == -1)
  {
    std::cout << "Couldn't read " << cloudfile << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << src.size() << " data points from " << cloudfile << std::endl;
  
  // Downsample the source cloud, apply voxelgrid filter
  double voxel_leaf_size = 0.1;
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr(new pcl::PointCloud<pcl::PointXYZI>(src));
  pcl::PointCloud<pcl::PointXYZI> tmp;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(src_ptr);
  voxel_grid_filter.filter(tmp);

  // And set up KDTree
  pcl::PointCloud<pcl::PointXYZ> tmp_xyz;
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    pcl::PointXYZ p;
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    tmp_xyz.push_back(p);
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>(tmp_xyz));
  kdtree.setInputCloud(filtered_ptr);

  // LIDAR pose data
  std::string posefile = "/home/zwu/1dec-datacollection/second/localizing_pose.csv";
  std::ifstream pose_stream;
  pose_stream.open(posefile);

  // Place-holder for csv stream variables
  std::string line, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  std::vector<double> lidar_times;
  std::vector<pose> lidar_poses;
  getline(pose_stream, line);
  std::cout << "File sequence: " << line << std::endl;
  std::cout << "Collecting localized lidar poses and time stamps." << std::endl;
  while(getline(pose_stream, line))
  {
    std::stringstream line_stream(line);

    // Get data value
    getline(line_stream, seq_str, ',');
    getline(line_stream, sec_str, ',');
    getline(line_stream, nsec_str, ',');
    getline(line_stream, x_str, ',');
    getline(line_stream, y_str, ',');
    getline(line_stream, z_str, ',');
    getline(line_stream, roll_str, ',');
    getline(line_stream, pitch_str, ',');
    getline(line_stream, yaw_str);

    double current_time = std::stod(sec_str) + std::stod(nsec_str) / 1e9; // in seconds
    // std::cout << "time: " << current_time << std::endl;
    lidar_times.push_back(current_time);
    pose current_pose({std::stod(x_str), std::stod(y_str), std::stod(z_str),
                       std::stod(roll_str), std::stod(pitch_str), std::stod(yaw_str)});
    // std::cout << "pose: " << current_pose.x << ","
    //                       << current_pose.y << ","
    //                       << current_pose.z << ","
    //                       << current_pose.roll << ","
    //                       << current_pose.pitch << ","
    //                       << current_pose.yaw << std::endl;

    lidar_poses.push_back(current_pose);
  }
  pose_stream.close();
  std::cout << "INFO: Collected " << lidar_times.size() << " data." << std::endl;

  // Camera timestamps
  std::string camerafile = "/home/zwu/1dec-datacollection/second/1dec-6cams-2nd/timestamp.txt";
  std::ifstream cam_stream;
  cam_stream.open(camerafile);
  std::vector<double> camera_times;
  std::cout << "INFO/WARN: Collecting camera timestamps / NO HEADER EXPECTED FOR THIS FILE." << std::endl;
  while(getline(cam_stream, line))
  {
    // uint64_t sec = std::stoull(line.c_str()) / 1000000; // to s
    // uint64_t nsec = std::stoull(line.c_str()) % 1000000; // to us
    double current_time = std::stoull(line.c_str()) / 1e6; // from us to s
    std::cout << "time: " << std::setprecision(6) << current_time << std::endl;
    camera_times.push_back(current_time);
  }
  std::cout << "INFO: Collected " << camera_times.size() << " data." << std::endl;

  // Do interpolation and publish the synthetic perspective pointcloud
  unsigned int lIdx = 0, lIdx_end = lidar_times.size();
  pose pose_diff;
  pose_diff.x = lidar_poses[lIdx+1].x - lidar_poses[lIdx].x;
  pose_diff.y = lidar_poses[lIdx+1].y - lidar_poses[lIdx].y;
  pose_diff.z = lidar_poses[lIdx+1].z - lidar_poses[lIdx].z;
  pose_diff.roll = lidar_poses[lIdx+1].roll - lidar_poses[lIdx].roll;
  pose_diff.pitch = lidar_poses[lIdx+1].pitch - lidar_poses[lIdx].pitch;
  pose_diff.yaw = lidar_poses[lIdx+1].yaw - lidar_poses[lIdx].yaw;
  for(unsigned int cIdx = 0, cIdx_end = camera_times.size(); cIdx < cIdx_end; cIdx++)
  {
    if(camera_times[cIdx] < lidar_times[lIdx])
    {
      std::cout << "INFO: Skipping initial timestamps of camera to catchup with lidar..." << std::endl;
      continue;
    }

    if(camera_times[cIdx] > lidar_times[lIdx+1])
    {
      std::cout << "INFO: Proceeding to next time period of lidar..." << std::endl;
      lIdx++; // to next lidar
      if(lIdx >= lIdx_end)
      {
        std::cout << "INFO: End of camera timestamps. Exit." << std::endl;
        break;
      }
      // Update pose_diff
      pose_diff.x = lidar_poses[lIdx+1].x - lidar_poses[lIdx].x;
      pose_diff.y = lidar_poses[lIdx+1].y - lidar_poses[lIdx].y;
      pose_diff.z = lidar_poses[lIdx+1].z - lidar_poses[lIdx].z;
      pose_diff.roll = lidar_poses[lIdx+1].roll - lidar_poses[lIdx].roll;
      pose_diff.pitch = lidar_poses[lIdx+1].pitch - lidar_poses[lIdx].pitch;
      pose_diff.yaw = lidar_poses[lIdx+1].yaw - lidar_poses[lIdx].yaw;
      cIdx--; // to current cam
      continue;
    }

    // Do interpolation
    double interpolating_ratio = (camera_times[cIdx] - lidar_times[lIdx]) / (lidar_times[lIdx+1] - lidar_times[lIdx]);
    pose interpolating_pose({lidar_poses[lIdx].x + interpolating_ratio * pose_diff.x,
                             lidar_poses[lIdx].y + interpolating_ratio * pose_diff.y,
                             lidar_poses[lIdx].z + interpolating_ratio * pose_diff.z,
                             lidar_poses[lIdx].roll + interpolating_ratio * pose_diff.roll,
                             lidar_poses[lIdx].pitch + interpolating_ratio * pose_diff.pitch,
                             lidar_poses[lIdx].yaw + interpolating_ratio * pose_diff.yaw});
    std::cout << "Pose interpolated: " << interpolating_pose.x << ","
                                       << interpolating_pose.y << ","
                                       << interpolating_pose.z << std::endl;

    // Search around interpolated point for cloud
    // K nearest neighbor search
    pcl::PointXYZ searchPoint;
    searchPoint.x = interpolating_pose.x;
    searchPoint.y = interpolating_pose.y;
    searchPoint.z = interpolating_pose.z;

    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 30.0;
    pcl::PointCloud<pcl::PointXYZ> nearestCloud;
    if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      for(size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      {
        nearestCloud.push_back(filtered_ptr->points[pointIdxRadiusSearch[i]]);
      }
    }

    // Publish
    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    nearestCloud.header.frame_id = "map";
    pcl::toROSMsg(nearestCloud, *scan_msg_ptr);
    scan_pub.publish(*scan_msg_ptr);
  }
  // ros::spin();
  return 0;
}
