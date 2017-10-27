// #define PCL_NO_PRECOMPILE // to create a custom pcl point type
// Basic libs
// #include <chrono>
#include <fstream>
#include <iostream>
// #include <memory.h>
#include <sstream>
// #include <mutex>
// #include <omp.h>
#include <sstream>
#include <string>
// #include <stdlib.h>
// #include <cmath>
// #include <unordered_map>
// #include <vector>

// Libraries for system commands
// #include <cstdlib>
// #include <unistd.h>
// #include <sys/types.h>
// #include <pwd.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// #include <ros/time.h>
// #include <ros/duration.h>
// #include <std_msgs/Bool.h>
// // #include <std_msgs/Float32.h>
// #include <pcl/PCLPointField.h>
// #include <sensor_msgs/PointField.h>
// #include <pcl/PCLPointCloud2.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>

// // PCL & 3rd party libs
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <my_package/lidar_pcl.h>

ros::Publisher current_scan_pub;

const int LAYER_NUM = 32;
ros::Rate *r;

static void ndt_mapping_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // CSV file
  // std::string csv_file = "/home/zwu/trash_ws/scan" + std::to_string(input->header.seq) + ".csv";
  // std::ofstream csv_stream;
  // csv_stream.open(csv_file);
  // for(int i = 0; i < LAYER_NUM; i++)
  //   csv_stream << i << ",";
  // csv_stream << std::endl;  

  // bool ring_check[LAYER_NUM] = {false};
  // float ring_angle[LAYER_NUM];

  pcl::PointCloud<lidar_pcl::PointXYZIR> scan;
  lidar_pcl::fromROSMsg(*input, scan);
  std::cout << "Cloud size: " << input->data.size()/32 << " --> " << scan.size() << std::endl;
  // for(int i = 0, i_max = scan.size(); i < i_max; i+=1)
  // {
  //   if(scan.points[i].ring == 0)
  //     std::cout << scan.points[i] << std::endl;
  // }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(scan, *map_msg_ptr);
  current_scan_pub.publish(*map_msg_ptr);
  r->sleep();

  // for(int i = 0, i_max = input->data.size(); i < i_max; i += 32)
  // {
  //   PointXYZIR new_point(&input->data[i]);
  //   if(ring_check[new_point.ring] == false)
  //   {
  //     ring_angle[new_point.ring] = new_point.yaw();
  //     ring_check[new_point.ring] = true;
  //   }
  //   else // ring_check[new_point.ring] == true
  //   {
  //     // print all angle values to file
  //     for(int j = 0; j < LAYER_NUM; j++)
  //     {
  //       if(ring_check[j] == true)
  //         csv_stream << ring_angle[j] << ",";
  //       else 
  //         csv_stream << "nan,";
  //     }
  //     csv_stream << std::endl;

  //     // reset arrays
  //     for(int j = 0; j < LAYER_NUM; j++)
  //       ring_check[j] = false;
      
  //     // and put the newpoint's value in
  //     ring_angle[new_point.ring] = new_point.yaw();
  //     ring_check[new_point.ring] = true;
  //   }
  // }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_preprocessing");

  ros::NodeHandle nh;
  r = new ros::Rate(5);
  // ROS node
  current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 1, true);

  // Open bagfile
  if(argc < 2)
  {
    std::cout << "Please indicate bag file to process." << std::endl;
    return -1;
  }
  std::cout << "Loading " << argv[1] << std::endl;
  rosbag::Bag bag(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
  reading_topics.push_back(std::string("/points_raw"));
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics));

  // Looping, processing messages in bag file
  // int count = 0;
  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL)
    {
      std::cout << "No input PointCloud2 available. Waiting..." << std::endl;
      continue;
    }
    ndt_mapping_callback(input_cloud);
    // count++;
    // if(count > 1)
    // {
    //   std::cout << "Stopping since more than " << count << " messages are processed." << std::endl; 
    //   return 0;
    // }
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  return 0;
}