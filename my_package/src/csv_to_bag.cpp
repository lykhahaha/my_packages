/***
Author: TA
Usage: rosrun my_package csv_to_bag CSV_DIR/ OUTPUT_BAG.bag
Csv files must have their timestamps as names
***/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#define LOCAL_TO_WORLD_TF

using namespace std;
using namespace boost::filesystem;

int main(int argc, char** argv)
{
  if(argc < 3)
  {
    std::cout << "ERROR: Usage:" << std::endl;
    std::cout << "rosrun my_package csv_to_bag CSV_DIR/ OUTPUT_BAG.bag" << std::endl;
    return(-1);
  }

  // Output bag file
  rosbag::Bag bag;
  std::string bag_filename = argv[2];
  bag.open(bag_filename, rosbag::bagmode::Write);
  std::string frame_id = "velodyne";

  std::string input_dir = argv[1];
  std::cout << "Reading data from: " << input_dir << std::endl;
  if(!is_directory(input_dir))
  {
    std::cout << "ERROR: " << input_dir << "is not a valid directory." << std::endl;
    return(-1);
  }

#ifdef LOCAL_TO_WORLD_TF
  // Transformation to base link and to world
  Eigen::Affine3d lidar2base_tf;
  pcl::getTransformation(0.002, -0.004, -0.957, 0.0140848, 0.00289725, -1.583066, lidar2base_tf);
  Eigen::Affine3d base2world_tf;
  pcl::getTransformation(0, 0, 0, 3.141593, 0, 0, base2world_tf);
  Eigen::Affine3d lidar2world_tf = base2world_tf * lidar2base_tf;
#endif // LOCAL_TO_WORLD_TF

  // Get a list of file in input_dir then sort
  std::vector<path> file_list;
  std::copy(directory_iterator(input_dir), directory_iterator(), back_inserter(file_list));
  std::sort(file_list.begin(), file_list.end()); 

  // Iterate through saved list
  int seq = 0;
  for(std::vector<path>::const_iterator itr(file_list.begin()), it_end(file_list.end()); itr != it_end; ++itr)
  {
    std::string filename = itr->filename().string();
    std::cout << "Processing: " << filename;

    if(is_regular_file(*itr)) // if it is a file
      std::cout << " [" << file_size(*itr) << " bytes]" << std::endl;
    else
    {
      std::cout << "[directory], skipped." << std::endl;
      continue;
    }

    // Timestamp
    std::string utime = filename;
    utime.resize(filename.size()-4); // erase the .csv extension
    uint64_t sec = std::stoull(utime.c_str()) / 1000000; // us to s
    uint64_t nsec = std::stoull(utime.c_str()) % 1000000 * 1000; // discard sec portion, us to ns
    
    // Process csv
    // Place-holder for variables
    std::ifstream in_stream;
    in_stream.open(itr->string());
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> scan;
    std::string line, x_str, y_str, z_str, intensity_str, ring_str;
    int points_skipped = 0;
    while(getline(in_stream, line)) // Loop through lines in file
    {
      std::stringstream line_stream(line); // convert string to separatable stream
      
      // Get data
      getline(line_stream, x_str, ',');
      getline(line_stream, y_str, ',');
      getline(line_stream, z_str, ',');
      getline(line_stream, intensity_str, ',');
      getline(line_stream, ring_str);
      float x = std::stof(x_str);
      float y = std::stof(y_str);
      float z = std::stof(z_str);
      float intensity = std::stof(intensity_str);
      int ring = std::stoi(ring_str);

      if(!x && !y && !z && !intensity && !ring)
      {
        points_skipped++;
        continue;
      }

      // Convert data to pointXYZI and push it to cloud [scan]
      velodyne_pointcloud::PointXYZIR new_point;
      new_point.x = x;
      new_point.y = y;
      new_point.z = z;
      new_point.intensity = intensity;
      new_point.ring = ring;

      scan.push_back(new_point);
    }

#ifdef LOCAL_TO_WORLD_TF
    pcl::transformPointCloud(scan, scan, lidar2world_tf);
#endif // LOCAL_TO_WORLD_TF

    // Write to bag file
    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(scan, *scan_msg_ptr);
    scan_msg_ptr->header.seq = seq;
    scan_msg_ptr->header.stamp.sec = sec;
    scan_msg_ptr->header.stamp.nsec = nsec;
    scan_msg_ptr->header.frame_id = frame_id;
    bag.write("/points_raw", ros::Time(sec, nsec), *scan_msg_ptr);
    seq++;
    if(seq > 1000) // test 1st
      break;
  }

  bag.close();
  std::cout << "Finished. Wrote bag file to " << bag_filename << std::endl;
  return 0;
}