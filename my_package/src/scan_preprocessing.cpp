// // Basic libs
// #include <chrono>
#include <fstream>
#include <iostream>
#include <memory.h>
// #include <mutex>
// #include <omp.h>
// #include <sstream>
// #include <string>
// #include <unordered_map>
// #include <vector>

// // Libraries for system commands
// #include <cstdlib>
// #include <unistd.h>
// #include <sys/types.h>
// #include <pwd.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

// // ROS libs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// #include <ros/time.h>
// #include <ros/duration.h>
// #include <signal.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>

// // PCL & 3rd party libs
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <typeinfo>

// #include <bitset>

struct PointXYZIR
{
  float x;
  float y;
  float z;
  float intensity;
  int ring;

  PointXYZIR()
  {
    x = 0.;
    y = 0.;
    z = 0.;
    intensity = 0.;
    ring = 0;
  }

  PointXYZIR(const uint8_t * source)
  {
    memcpy(&this->x, source, 4);
    memcpy(&this->y, source + 4, 4);
    memcpy(&this->z, source + 8, 4);
    memcpy(&this->intensity, source + 16, 4);
    memcpy(&this->ring, source + 20, 4);
  }

  friend std::ostream& operator<<(std::ostream& os, const PointXYZIR& ret) 
  { 
    os << "(" << ret.x << "," << ret.y << "," << ret.z << "," 
       << ret.intensity << "," << ret.ring << ")" ;  
    return os;  
  } 
};

typedef struct PointXYZIR PointXYZIR;

static void ndt_mapping_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // pcl::PointCloud<pcl::PointXYZI> scan;
  // pcl::fromROSMsg(*input, scan);
  pcl::PointCloud<PointXYZIR> scan;
  std::cout << input->data.size() << std::endl;
  for(int i = 0, i_max = input->data.size(); i < i_max; i+=32)
  {
    PointXYZIR p(&input->data[i]);
    std::cout << p << std::endl;
    // std::cout << scan.points[i/32] << std::endl;

    scan.push_back(p);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_preprocessing");

  ros::NodeHandle nh;

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
  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL)
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }
    ndt_mapping_callback(input_cloud);
    return 0;
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  return 0;
}