#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <my_package/transformPointsConfig.h>

// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
// #include <pcl/common/angles.h>
// #include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
// #include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
// #include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

ros::Publisher map_pub;
pcl::PointCloud<pcl::PointXYZI> src;

using namespace ros;
using namespace std;

void dynamic_configCb(my_package::transformPointsConfig &config, uint32_t level) 
{
  // Get transform values
  double x = config.x;
	double y = config.y;
	double z = config.z;
	double roll = config.roll;
	double pitch = config.pitch;
	double yaw = config.yaw;
  ROS_INFO("Reconfigure Requested. Proceeding to transform....");

	// Start do tf
  // Create transformation matrix
  Eigen::Affine3f transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

  // Transform pointcloud
  pcl::PointCloud<pcl::PointXYZI> dst;
  dst.header.frame_id = "map";
  pcl::transformPointCloud(src, dst, transform);

  // Show output
  std::cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
  std::cout << "Transformation Matrix: " << std::endl;
  std::cout << transform.matrix() << std::endl;

  // Publish to topic to view in rviz
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(dst));
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  map_pub.publish(*map_msg_ptr);

  // Writing Point Cloud data to PCD file
  std::string out_filename = "my_transformed";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", dst);
  std::cout << "Finished. Saved pointcloud to " << out_filename << ".pcd" << std::endl;
  std::cout << "---------------------------------------\n" << std::endl;

  return;
}

int main(int argc, char** argv)
{
  // Initiate node
  ros::init(argc, argv, "transform_pcd");

  ros::NodeHandle nh;
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 10, true);

  if(argc != 2)
  {
  	std::cout << "Please indicate ply file" << std::endl;
  	std::cout << "Usage: rosrun my_package transform_ply [file.ply]" << std::endl;
  	return -1;
  }
  // Load pcd map to transform
  std::string pcd_filename = argv[1];
  std::cout << "Loading file: " << pcd_filename  << std::endl;
  if(pcl::io::loadPLYFile<pcl::PointXYZI>(pcd_filename, src) == -1)
  {
    std::cout << "Couldn't read " << pcd_filename << "." << std::endl;
    return -1;
  }
  std::cout << "Loaded " << src.size() << " data points from " << pcd_filename << std::endl;

  dynamic_reconfigure::Server<my_package::transformPointsConfig> server;
  dynamic_reconfigure::Server<my_package::transformPointsConfig>::CallbackType f;
  f = boost::bind(&dynamic_configCb, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
