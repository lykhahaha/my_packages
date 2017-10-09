#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

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
#include <pcl/filters/voxel_grid.h>

#include <iostream>

using namespace ros;

int main(int argc, char **argv)
{
  if(argc != 8){
    std::cout << "Usage: rosrun my_package my_pcl [PCD_FILE.pcd] [x] [y] [z] [roll] [pitch] [yaw]" << std::endl;
    exit(1);
  }

  ros::init(argc, argv, "my_pcl");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_scan", 1000, true);

  std::string filename = argv[1];
  std::cout << "Loading file: " << filename << std::endl;

	// pcl::PointCloud<pcl::PointXYZI> input;
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  if(pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *input_ptr) == -1)
  {
    std::cout << "Couldn't read " << filename << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << input_ptr->size() << " data points from " << filename << std::endl;

  double x = std::stod(argv[2]);
  double y = std::stod(argv[3]);
  double z = std::stod(argv[4]);
  double roll = std::stod(argv[5]);
  double pitch = std::stod(argv[6]);
  double yaw = std::stod(argv[7]);

	//Create transformation matrix
	Eigen::Affine3f transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

	// Transform pointcloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::transformPointCloud(*input_ptr, *output_ptr, transform);

	// Publish map to view
	sensor_msgs::PointCloud2::Ptr output_msg_ptr(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*output_ptr, *output_msg_ptr);
	output_msg_ptr->header.frame_id = "map";
  pub.publish(*output_msg_ptr);

  // Downsize the cloud	using voxel grid filter
  // pcl::PointCloud<pcl::PointXYZI>::Ptr output_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  // output_filtered_ptr->header.frame_id = "map";
  // double filter_res = 0.3;
  // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  // voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
  // voxel_grid_filter.setInputCloud(output_ptr);
  // voxel_grid_filter.filter(*output_filtered_ptr);
  // *output_ptr = *output_filtered_ptr;

  // Save point cloud to pcd
  std::string out_filename = "/home/zwu/transformed_scan";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", *output_ptr);
  pcl::io::savePLYFileBinary(out_filename + ".ply", *output_ptr);
  std::cout << output_ptr->size() << " points have been saved to " << out_filename << ".pcd" << std::endl;

  // Show output
  std::cout << "Transformation Matrix: " << std::endl;
	std::cout << transform.matrix() << std::endl;

  ros::spin();

  return 0;
}