// Basic libs
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// // PCL & 3rd party libs
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <my_package/lidar_pcl.h>

const double pi = 3.14159265358979323846;
const int LAYER_NUM = 32;
rosbag::Bag corrected_bag;

static void CallBack(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<lidar_pcl::PointXYZIR> scan_xyzir;
  pcl::fromROSMsg(*input, scan);
  pcl::copyPointCloud(scan, scan_xyzir);

  for(pcl::PointCloud<lidar_pcl::PointXYZIR>::iterator item = scan_xyzir.begin(); item != scan_xyzir.end(); item++)
  {
    double pitch = std::atan2(item->z, sqrt(item->x * item->x + item->y * item->y)); // rad
    double estimate = (pitch * 180 / pi + 30.67) * 3 / 4;
    int ring = std::round(estimate);
    if(std::fabs(estimate - ring) > 0.1)
    {
      // Ring number 2 (angle -28.00) has a problem in px2
      item->ring = 2;
      pitch = -28.0 / 180 * pi; // rad
      double yaw = std::atan2(item->y, item->x);
      double rho = sqrt(item->x * item->x + item->y * item->y + item->z * item->z);
      
      // Re-calculate x, y, z (spherical to cartesian)
      item->x = rho * cos(pitch) * cos(yaw);
      item->y = rho * cos(pitch) * sin(yaw);
      item->z = rho * sin(pitch);
    }
    else
    {
      item->ring = ring;
    }
  }

  // Write back to bag
  sensor_msgs::PointCloud2 scan_msg;
  pcl::toROSMsg(scan_xyzir, scan_msg);
  scan_msg.header.seq = input->header.seq;
  scan_msg.header.stamp.sec = input->header.stamp.sec;
  scan_msg.header.stamp.nsec = input->header.stamp.nsec;
  scan_msg.header.frame_id = "velodyne";
  corrected_bag.write("/points_raw", input->header.stamp, scan_msg); 
}

int main(int argc, char** argv)
{
  // Open bagfile
  if(argc < 3)
  {
    std::cout << "ERROR: missing arg(s)." << std::endl;
    std::cout << "rosrun px2_velo_debug INPUT_BAG OUTPUT_BAG" << std::endl;
    return -1;
  }
  std::cout << "Loading " << argv[1] << std::endl;
  rosbag::Bag inbag(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
  reading_topics.push_back(std::string("/points_raw"));
  rosbag::View view(inbag, rosbag::TopicQuery(reading_topics));

  corrected_bag.open(argv[2], rosbag::bagmode::Write);

  // Looping, processing messages in bag file
  std::cout << "Processing " << argv[1] << std::endl;
  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL)
    {
      std::cout << "No input PointCloud2 available. Waiting..." << std::endl;
      continue;
    }
    CallBack(input_cloud);
  }
  inbag.close();
  corrected_bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  return 0;
}