#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <my_package/offsetMappingConfig.h>

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
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct submap
{
	std::string dir;
	std::string csv;
	Eigen::Affine3f tf;
};

ros::Publisher map_pub;

using namespace ros;
using namespace std;

void dynamic_configCb(my_package::offsetMappingConfig &config, uint32_t level) 
{
  // Working directory and data, relative to /home/username/
  std::vector<submap> sub_map;
  submap sub;
	  sub.dir = "offset-mapping";
	  sub.csv = "tf_map.csv";
	  sub.tf = pcl::getTransformation(0, 0, 0, 0, 0, 0);
  sub_map.push_back(sub);
  // 	sub.dir = "stk-2";
  // 	sub.csv = "tf_map2.csv";
  // 	sub.tf = pcl::getTransformation(-343.872, 25.684, 0, 0, 0, -1.79277); // merge map 2 to 1
  // sub_map.push_back(sub);
  // 	sub.dir = "stk-3";
  // 	sub.csv = "tf_map3.csv";
  // 	sub.tf = pcl::getTransformation(-285.895, -536.755, 0, 0, 0, -1.17457); // merge map 3 to map 1+2
  // sub_map.push_back(sub);
  // 	sub.dir = "stk-5";
  // 	sub.csv = "tf_map5.csv";
  // 	sub.tf = pcl::getTransformation(-150.901, -788.016, 0, 0, 0, -0.862968); // merge map 6 to map 1-5
  // sub_map.push_back(sub);
		// sub.dir = "stk-6";
  // 	sub.csv = "tf_map6.csv";
  // 	sub.tf = pcl::getTransformation(-122.321, -783.393, 0, 0, 0, 0.725139); // merge map 6 to map 1-5
  // sub_map.push_back(sub);
  // 	sub.dir = "stk-7";
  // 	sub.csv = "tf_map7.csv";
  // 	sub.tf = pcl::getTransformation(-62.5062, 45.8435, 0, 0, 0, -1.70206); // merge map 7 to map 1-6
  // sub_map.push_back(sub);

  // Get offset values
 //  const double err_x = 0;
	// const double err_y = 0;
	// const double err_yaw = 0;
  double err_x = config.err_x;
	double err_y = config.err_y;
	double err_yaw = config.err_yaw;
	const double N_scan = config.N_scan;
  ROS_INFO("Reconfigure Requested. Proceeding to re-offset mapping now.");

  // Do processing for every map sections, except the last (7th) one
  pcl::PointCloud<pcl::PointXYZI> map; // full map holder
  map.header.frame_id = "map";
  int scan_count = 0;
  std::ofstream outp;
  std::string out_file = "manual_offset_pose.txt";
  outp.open(out_file);
  std::cout << "Working on merging..." << std::endl;
  for(uint i = 0; i < sub_map.size(); i++)
  {
  	std::string dir = sub_map[i].dir;
  	std::string csv = sub_map[i].csv;
  	std::cout << "Processing " << dir << "/" << csv << std::endl;
		std::ifstream csv_stream(dir + "/" + csv);
	  // Place-holder for variables
		std::string line, pcd_filename, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;

	  while(getline(csv_stream, line))
	  {
	  	if(i != (sub_map.size() - 1) || i == 0)
	  		scan_count++;
	  	pcl::PointCloud<pcl::PointXYZI> src, tmp1, tmp2, dst;
	  	std::stringstream line_stream(line);

	    // Get pcd file
	    getline(line_stream, pcd_filename, ',');
	    std::cout << "Loading file: " << dir << "/" << pcd_filename << std::endl;
	    if(pcl::io::loadPCDFile<pcl::PointXYZI>(dir + "/" + pcd_filename, src) == -1)
	    {
	      std::cout << "Couldn't read " << pcd_filename << "." << std::endl;
	      return;
	    }
	    std::cout << "Loaded " << src.size() << " data points from " << dir << pcd_filename << std::endl;
	    
	    // and the rest of the data
	    getline(line_stream, x_str, ',');
	    double x = std::stod(x_str);
	    getline(line_stream, y_str, ',');
	    double y = std::stod(y_str);
	    getline(line_stream, z_str, ',');
	    double z = std::stod(z_str);
	    getline(line_stream, roll_str, ',');
	    double roll = std::stod(roll_str);
	    getline(line_stream, pitch_str, ',');
	    double pitch = std::stod(pitch_str);
	    getline(line_stream, yaw_str);
	    double yaw = std::stod(yaw_str);

	    // Start do tf
	    // Create transformation matrix
	    Eigen::Affine3f transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

	    // Globalize the tf
	   	Eigen::Affine3f offset_transform = 
	   			pcl::getTransformation(-scan_count*err_x/N_scan, -scan_count*err_y/N_scan, 0, 0, 0, -scan_count*err_yaw/N_scan);

	    // Transform pointcloud
	    // pcl::transformPointCloud(src, tmp1, transform);
	    // pcl::transformPointCloud(tmp1, tmp2, sub_map[i].tf);
	    // pcl::transformPointCloud(tmp2, dst, offset_transform);
	    Eigen::Affine3f global_transform = offset_transform * sub_map[i].tf * transform;
	    pcl::transformPointCloud(src, dst, global_transform);

	    // Add to map
	    map += dst;
	    outp << x - scan_count*err_x/N_scan << "," << y - scan_count*err_y/N_scan << "," << z << std::endl;

	    // Show output
	    std::cout << "Scans processed: " << scan_count << std::endl;
	    std::cout << "Number of scan points: " << dst.size() << std::endl;
	    std::cout << "Number of map points: " << map.size() << std::endl;
	    std::cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
	    std::cout << "Transformation Matrix: " << std::endl;
	    std::cout << global_transform.matrix() << std::endl;
	    std::cout << "---------------------------------------" << std::endl;
	  }
	}

	// Write merged offset map to file  
   
  // Apply voxelgrid filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  pcl::PointCloud<pcl::PointXYZI> map_filtered;
  double filter_res = 0.3;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
  voxel_grid_filter.setInputCloud(map_ptr);
  voxel_grid_filter.filter(map_filtered);
  
  // Writing Point Cloud data to PCD file
  std::string out_filename = "merged_offset_map";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", map);
  std::cout << "Finished. Saved pointcloud to " << out_filename << ".pcd" << std::endl;

  // Publish to topic to view in rviz
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(map_filtered, *map_msg_ptr);
  map_pub.publish(*map_msg_ptr);
  return;
}

int main(int argc, char** argv)
{
  // Initiate node
  ros::init(argc, argv, "offset_mapping");

  ros::NodeHandle nh;
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("offset_map", 1000, true);

  dynamic_reconfigure::Server<my_package::offsetMappingConfig> server;
  dynamic_reconfigure::Server<my_package::offsetMappingConfig>::CallbackType f;
  f = boost::bind(&dynamic_configCb, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
