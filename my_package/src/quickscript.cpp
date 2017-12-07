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
  // ros::init(argc, argv, "get_perspective_scan");
  // ros::NodeHandle nh;
  // ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/my_scan", 100, true);
  
  // Load input
  if(argc != 3)
  {
    std::cout << "Please run: rosrun my_package quickscipt input.pcd output.txt" << std::endl;
    return -1;
  }
  std::string input = argv[1];
  std::string output = argv[2];
  pcl::PointCloud<pcl::PointXYZI> src;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(input, src) == -1)
  {
    std::cout << "Couldn't read " << input << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << src.size() << " data points from " << input << std::endl;

  std::ofstream out_stream;
  out_stream.open(output);
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src.begin(); item != src.end(); item++)
  {
    char output_buffer[1000];
    double x = item->x;
    double y = item->y;
    double z = item->z;
    int intensity = item->intensity;
    sprintf(output_buffer, "%.10lf %.10lf %.10lf %d", x, y, z, intensity);
    out_stream << output_buffer << std::endl;
  }
  std::cout << "Saved " << src.size() << output << std::endl;

  return 0;
}
