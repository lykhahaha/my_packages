#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// // #include <dynamic_reconfigure/server.h>
// #include <sensor_msgs/PointCloud2.h>
// // #include <my_package/transformPointsConfig.h>

// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// // #include <pcl/io/ply_io.h>
// // #include <pcl/point_cloud.h>
#include <pcl/common/common.h>
// // #include <pcl/common/angles.h>
// #include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// // #include <pcl/point_types.h>
// // #include <pcl/conversions.h>
// // #include <pcl_ros/transforms.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <fstream>
// #include <sstream>
#include <string>
#include <vector>

// struct pose
// {
//   double x;
//   double y;
//   double z;
//   double roll;
//   double pitch;
//   double yaw;
// };

// inline double calculateMinAngleDist(double first, double second) // in radian
// {
//   double difference = first - second;
//   if(difference >= 3.14159265359)
//     return difference - 6.28318530718;
//   if(difference <= -3.14159265359)
//     return difference + 6.28318530718;
//   return difference;
// }

int main(int argc, char** argv)
{  
  // Load bag
  std::string bagfile = "/home/zwu/1dec-datacollection/first/bags/1dec-carpark.bag";
  std::string bagtopic = "/points_raw";
  std::cout << "Reading bag: " << bagfile << " [topic: " << bagtopic << "]... " << std::flush;
  rosbag::Bag bag(bagfile, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
  reading_topics.push_back(std::string(bagtopic));
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics));
  std::cout << "Done." << std::endl;

  // Load camera timestamps
  std::string camerafile = "/home/zwu/1dec-datacollection/first/1dec-6cams-1st/timestamp.txt";
  std::ifstream cam_stream;
  std::cout << "Loading camera timestamp: " << camerafile << std::endl;
  cam_stream.open(camerafile);
  std::string line;
  std::vector<double> camera_times;
  while(getline(cam_stream, line))
  {
    double time_stamp = std::stoull(line.c_str()) / 1.0e6; // from us to s
    camera_times.push_back(time_stamp);
  }
  cam_stream.close();

  // Do process
  std::cout << "Start processing... " << std::endl;
  std::string output_dir = "/home/zwu/firstrounddata/lidar_scan_offset/";
  const double lidar_timestamp_offset = -2.1;
  int camIdx = 0, camIdxEnd = camera_times.size();
  BOOST_FOREACH(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL) // no data?
    {
      // std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }

    double lidar_timestamp = input_cloud->header.stamp.toSec() + lidar_timestamp_offset;

    // if camera timestamp is too far ahead, we proceed to next cloud msg
    if(camera_times[camIdx] > lidar_timestamp + 0.04)
    {
      std::cout << "Skipping cloud at " << std::fixed << lidar_timestamp << std::endl;
      continue;
    }

    // if lidar timestamp is too far ahead, we increase the camera index
    while(camera_times[camIdx] < lidar_timestamp - 0.04)
    {
      camIdx++;
      if(camIdx >= camIdxEnd)
      {
        std::cout << "End of camera data. Done" << std::endl;
        return 0;
      }
    }

    // if the two timestamps are near, do processing
    while(camera_times[camIdx] < lidar_timestamp + 0.04)
    {
      // check if they are within acceptable margin (1/60 sec)
      if(std::fabs(camera_times[camIdx] - lidar_timestamp) < 1.0 / 60)
      {
        // Matches, convert pointcloud to txt file and save
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*input_cloud, pcl_cloud);
        std::ofstream pointcloud_stream;
        pointcloud_stream.open(output_dir + std::to_string(camIdx+1) + ".txt");
        for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = pcl_cloud.begin(); item != pcl_cloud.end(); item++)
        {
          char output_buffer[500];
          double x = item->x;
          double y = item->y;
          double z = item->z;
          int intensity = item->intensity;
          sprintf(output_buffer, "%.10lf %.10lf %.10lf %d", x, y, z, intensity);
          pointcloud_stream << output_buffer << std::endl;
        }
        std::cout << "Saved " << pcl_cloud.size() << " points to " + output_dir + std::to_string(camIdx+1) + ".txt" << std::endl;
        std::cout << "Timestamps [C/L]: [" << std::fixed << camera_times[camIdx] << "," << lidar_timestamp << "]" << std::endl;
        std::cout << "---------------------------------------------------------" << std::endl;
        
        camIdx++;
        if(camIdx >= camIdxEnd)
        {
          std::cout << "End of camera data. Done" << std::endl;
          return 0;
        }
      }
      else // if not, increase the index of camera timestamp and re-check
      {
        camIdx++;
        if(camIdx >= camIdxEnd)
        {
          std::cout << "End of camera data (!). Done" << std::endl;
          return 0;
        }
      }
    }
  }

  std::cout << "End of lidar data! Done." << std::endl;
  return 0;
}