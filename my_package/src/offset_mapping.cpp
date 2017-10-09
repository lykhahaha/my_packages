#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <autoware_msgs/tfMap.h>

// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
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

pcl::PointCloud<pcl::PointXYZI> map;
pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
ros::Publisher pub;

const double err_x = 8.0;
const double err_y = 2.5;
const double err_yaw = 0;
const double N_scan = 1735;
int scan_count = 0;

using namespace ros;

static void points_callback(const autoware_msgs::tfMap::Ptr& input)
{
	scan_count++;
	// Get point cloud
	pcl::PointCloud<pcl::PointXYZI> src_cloud;
	pcl::fromROSMsg(input->pointCloud, src_cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(src_cloud));

	tf::Quaternion q(input->poseStamped.pose.orientation.x,
										input->poseStamped.pose.orientation.y,
										input->poseStamped.pose.orientation.z,
										input->poseStamped.pose.orientation.w);
	// Get TF
	double x = input->poseStamped.pose.position.x - scan_count*err_x/N_scan;
	double y = input->poseStamped.pose.position.y - scan_count*err_y/N_scan;
	double z = input->poseStamped.pose.position.z;
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	//Create transformation matrix
	Eigen::Affine3f transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

	// Transform pointcloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::transformPointCloud(*src_cloud_ptr, *dst_cloud_ptr, transform);

	// Add to map
	map += *dst_cloud_ptr;

	// Publish map to view every once in a while
	if((scan_count % 200 == 0) || (scan_count == N_scan))
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
	  pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());
	  map_filtered_ptr->header.frame_id = "map";
	  
	  // Apply voxelgrid filter
	  double filter_res = 0.25;
	  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
	  voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
	  voxel_grid_filter.setInputCloud(map_ptr);
	  voxel_grid_filter.filter(*map_filtered_ptr);

	  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
		pcl::toROSMsg(*map_filtered_ptr, *map_msg_ptr);
		map_msg_ptr->header.frame_id = "map";
	  pub.publish(*map_msg_ptr);
	}

  // Show output
  std::cout << "Number of scan points: " << src_cloud_ptr->size() << std::endl;
  std::cout << "Number of map points: " << map_ptr->size() << std::endl;
  std::cout << "Transformation Matrix: " << std::endl;
	std::cout << transform.matrix() << std::endl;
	std::cout << "Scans processed: " << scan_count << std::endl;
	std::cout << "---------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offset_mapping");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::PointCloud2>("/offset_map", 1000, true);
  ros::Subscriber sub = nh.subscribe("/tf_map", 10000, points_callback);

  // ros::Rate loop_rate(0.1);
  // while(nh.ok())
  // {
  // 	pub.publish(*map_msg_ptr);
		// ros::spinOnce();
  //   loop_rate.sleep();
  // }
  ros::spin();

  return 0;
}
