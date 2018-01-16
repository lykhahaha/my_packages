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

#define OUTPUT_INTERPOLATED_POSE // to a csv file
// #define VOXEL_GRID_OCCLUSION // do not enable
#define OFFSET_TRANSFORM

#ifdef VOXEL_GRID_OCCLUSION
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#endif

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

inline double calculateMinAngleDist(double first, double second) // in radian
{
  double difference = first - second;
  if(difference >= 3.14159265359)
    return difference - 6.28318530718;
  if(difference <= -3.14159265359)
    return difference + 6.28318530718;
  return difference;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_perspective_scan");
  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/my_scan", 100, true);
  
  // Load input
  // std::string filename = argv[1];
  std::string cloudfile = "/home/zwu/LIDAR-DATA/1dec-carpark1.pcd";
  std::string posefile = "/home/zwu/8jan-datacollection/8jan-datacollection-3/localizing_pose.csv"; // localizing_pose
  std::string camerafile = "/home/zwu/8jan-datacollection/8jan-datacollection-3/timestamp.txt"; // cam timestamp
  std::string file_location = "/home/zwu/8jan-datacollection/8jan-datacollection-3/lidar/"; // output dir
 #ifdef OFFSET_TRANSFORM
  Eigen::Affine3d offset_tf;
  pcl::getTransformation(0, 0, 0, -0.02, 0, -0.01, offset_tf);
 #endif

  pcl::PointCloud<pcl::PointXYZI> src;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(cloudfile, src) == -1)
  {
    std::cout << "Couldn't read " << cloudfile << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << src.size() << " data points from " << cloudfile << std::endl;
  
  // Downsample the source cloud, apply voxelgrid filter
  double voxel_leaf_size = 0.3;
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
  std::ifstream pose_stream;
  pose_stream.open(posefile);

  // Place-holder for csv stream variables
  std::string line, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  std::vector<double> lidar_times;
  std::vector<pose> lidar_poses;
  getline(pose_stream, line);
  std::cout << "File sequence: " << line << std::endl;
  std::cout << "Collecting localized lidar poses and time stamps." << std::endl;
  const double lidar_time_offset = -2.1;

 #ifdef OUTPUT_INTERPOLATED_POSE
  std::ofstream intrpl_pose_stream;
  std::string intrpl_pose_file = "/home/zwu/8jan-datacollection/8jan-datacollection-3/interpolated_pose_8jan_round3.csv";
  intrpl_pose_stream.open(intrpl_pose_file);
  intrpl_pose_stream << "timestamp,x,y,z,roll,pitch,yaw" << std::endl;
 #endif

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
    // std::cout << std::fixed << std::setprecision(9) << current_time << std::endl;
    lidar_times.push_back(current_time + lidar_time_offset);
    Eigen::Affine3d tf_vtow;
    pcl::getTransformation(std::stod(x_str), std::stod(y_str), std::stod(z_str),
                           std::stod(roll_str), std::stod(pitch_str), std::stod(yaw_str),
                           tf_vtow);

    pose current_pose({std::stod(x_str), std::stod(y_str), std::stod(z_str),
                       std::stod(roll_str), std::stod(pitch_str), std::stod(yaw_str)});

    lidar_poses.push_back(current_pose);
  }
  pose_stream.close();
  std::cout << "INFO: Collected " << lidar_times.size() << " data." << std::endl;

  // Camera timestamps
  std::ifstream cam_stream;
  cam_stream.open(camerafile);
  std::vector<double> camera_times;
  std::cout << "INFO/WARN: Collecting camera timestamps / NO HEADER EXPECTED FOR THIS FILE." << std::endl;
  while(getline(cam_stream, line))
  {
    double current_time = std::stoull(line.c_str()) / 1e6; // from us to s
    // std::cout << "time: " << std::fixed << current_time << std::endl;
    camera_times.push_back(current_time);
  }
  std::cout << "INFO: Collected " << camera_times.size() << " data." << std::endl;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Do interpolation and publish the synthetic perspective pointcloud
  unsigned int lIdx = 0, lIdx_end = lidar_times.size();
  pose pose_diff;
  pose_diff.x = lidar_poses[lIdx+1].x - lidar_poses[lIdx].x;
  pose_diff.y = lidar_poses[lIdx+1].y - lidar_poses[lIdx].y;
  pose_diff.z = lidar_poses[lIdx+1].z - lidar_poses[lIdx].z;
  pose_diff.roll = calculateMinAngleDist(lidar_poses[lIdx+1].roll, lidar_poses[lIdx].roll);
  pose_diff.pitch = calculateMinAngleDist(lidar_poses[lIdx+1].pitch, lidar_poses[lIdx].pitch);
  pose_diff.yaw = calculateMinAngleDist(lidar_poses[lIdx+1].yaw, lidar_poses[lIdx].yaw);

  unsigned int file_count = 1;
  for(unsigned int cIdx = 0, cIdx_end = camera_times.size(); cIdx < cIdx_end; cIdx++)
  {
    std::cout << "[CAM/LIDAR]: [" << cIdx << "/" << lIdx << "]" << std::endl;
    std::cout << "Timestamps [C/L]: " << std::fixed << std::setprecision(6) << camera_times[cIdx] << "/"
                                      << std::fixed << std::setprecision(6) << lidar_times[lIdx] << std::endl;
    if(camera_times[cIdx] < lidar_times[lIdx])
    {
      std::cout << "INFO: Skipping initial timestamps of camera to catchup with lidar..." << std::endl;
      std::cout << "---------------------------------------------------------" << std::endl;
      file_count++;
      continue;
    }

    if(camera_times[cIdx] > lidar_times[lIdx+1])
    {
      std::cout << "INFO: Proceeding...\n-------------------------------------------" << std::endl;
      lIdx++; // to next lidar
      if(lIdx >= lIdx_end)
      {
        std::cout << "INFO: End of lidar timestamps. Exit." << std::endl;
        break;
      }
      // Update pose_diff
      pose_diff.x = lidar_poses[lIdx+1].x - lidar_poses[lIdx].x;
      pose_diff.y = lidar_poses[lIdx+1].y - lidar_poses[lIdx].y;
      pose_diff.z = lidar_poses[lIdx+1].z - lidar_poses[lIdx].z;
      pose_diff.roll = calculateMinAngleDist(lidar_poses[lIdx+1].roll, lidar_poses[lIdx].roll);
      pose_diff.pitch = calculateMinAngleDist(lidar_poses[lIdx+1].pitch, lidar_poses[lIdx].pitch);
      pose_diff.yaw = calculateMinAngleDist(lidar_poses[lIdx+1].yaw, lidar_poses[lIdx].yaw);
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
    // std::cout << "Pose interpolated: " << interpolating_pose.x << ","
    //                                    << interpolating_pose.y << ","
    //                                    << interpolating_pose.z << ","
    //                                    << interpolating_pose.roll << ","
    //                                    << interpolating_pose.pitch << ","
    //                                    << interpolating_pose.yaw << std::endl;

    #ifdef OUTPUT_INTERPOLATED_POSE
    intrpl_pose_stream << std::fixed << std::setprecision(0) 
                       << camera_times[cIdx] * 1e6 << ","
                       << std::setprecision(10)
                       << interpolating_pose.x << ","
                       << interpolating_pose.y << ","
                       << interpolating_pose.z << ","
                       << interpolating_pose.roll << ","
                       << interpolating_pose.pitch << ","
                       << interpolating_pose.yaw << std::endl;
    #endif

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
    pcl::PointCloud<pcl::PointXYZI> nearestCloud;
    if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
      {
        nearestCloud.push_back(tmp.points[pointIdxRadiusSearch[i]]);
      }
    }

    // Transform back to local coordinate
    Eigen::Affine3d transform;
    pcl::getTransformation(interpolating_pose.x, 
                           interpolating_pose.y, 
                           interpolating_pose.z, 
                           interpolating_pose.roll, 
                           interpolating_pose.pitch, 
                           interpolating_pose.yaw, transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(nearestCloud, *localCloud, transform.inverse());

   #ifdef VOXEL_GRID_OCCLUSION
    // Set up voxel_grid_occlusion_estimation
    pcl::PointCloud<pcl::PointXYZI>::Ptr localVisibleCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGridOcclusionEstimation<pcl::PointXYZI> vgoe;
    vgoe.setInputCloud(localCloud);
    vgoe.setLeafSize(0.8, 0.8, 0.8);
    vgoe.initializeVoxelGrid();

    // Estimate the occluded space
    std::vector< Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > occluded_voxels;
    // if(vgoe.occlusionEstimationAll(occluded_voxels) == -1)
    if(0)
    {
      std::cout << "ERROR: Failed to apply voxel grid occlusion estimation!" << std::endl;
      *localVisibleCloud = *localCloud;
    }
    else
    {
      // Collect all visible points to a new cloud
      for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = localCloud->begin(); item != localCloud->end(); item++)
      {
        Eigen::Vector3i current_grid_cordinates = vgoe.getGridCoordinates(item->x, item->y, item->z);
        int current_grid_state;
        if(vgoe.occlusionEstimation(current_grid_state, current_grid_cordinates) == -1)
          continue;
        else
        {
          if(current_grid_state == 0)
          {
            localVisibleCloud->push_back(*item);
          }
        }
      }
    }
   #endif

    // Publish
    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
   #ifndef VOXEL_GRID_OCCLUSION
    localCloud->header.frame_id = "map";
    pcl::toROSMsg(*localCloud, *scan_msg_ptr);
   #else
    localVisibleCloud->header.frame_id = "map";
    pcl::toROSMsg(*localVisibleCloud, *scan_msg_ptr);
   #endif
    scan_pub.publish(*scan_msg_ptr);

    // Convert pointcloud to txt file and save
    std::ofstream pointcloud_stream;
    pointcloud_stream.open(file_location + std::to_string(file_count) + ".txt");
   #ifdef OFFSET_TRANSFORM
    pcl::PointCloud<pcl::PointXYZI> localCloudOffset;
    pcl::transformPointCloud(*localCloud, localCloudOffset, offset_tf);
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = localCloudOffset.begin(); item != localCloudOffset.end(); item++)
   #else
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = localCloud->begin(); item != localCloud->end(); item++)
   #endif
    {
      char output_buffer[1000];
      double x = item->x;
      double y = item->y;
      double z = item->z;
      int intensity = item->intensity;
      sprintf(output_buffer, "%.10lf %.10lf %.10lf %d", x, y, z, intensity);
      pointcloud_stream << output_buffer << std::endl;
    }
    std::cout << "Saved " << localCloud->size() << " points to " + file_location + std::to_string(file_count) + ".txt" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    file_count++;
  }
  // ros::spin();
  return 0;
}
