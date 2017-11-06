#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstddef>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// #include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
  // Initiate and get csv file
  ros::init(argc, argv, "submap_merger");

  if(argc < 2)
  {
    std::cout << "Please indicate the bag file corresponding to the map_pose.csv." << std::endl;
    std::cout << "rosrun my_package submap_merger \"bagfile\"" << std::endl;
    return -1;
  }
  std::string bagfile = argv[1];
  std::string bagtopic = "/points_raw";
  std::cout << "Reading bag: " << bagfile << " [topic: " << bagtopic << "]... " << std::flush;
  rosbag::Bag bag(bagfile, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string(bagtopic));
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics));
  std::cout << "Done." << std::endl;


  std::string csvfile = "map_pose.csv";
  std::cout << "Reading " << csvfile << " in the current directory.\n";
  std::cout << "Please ensure that the format of the csv file is:" << std::endl;
  std::cout << "\t{key, sequence, sec, nsec, x, y, z, roll, pitch, yaw}" << std::endl; 
  // TODO: remove filename in later versions
  std::ifstream csv_stream(csvfile);

  // Place-holder for variables
  std::string line, key_str, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  pcl::PointCloud<pcl::PointXYZI> map;
  unsigned int seq, key;
  double x, y, z, roll, pitch, yaw;

  getline(csv_stream, line); // to skip header line
  std::cout << "Finished. Starting merging submaps..." << std::endl;
  foreach(rosbag::MessageInstance const message, view)
  {
    // get new data and process values
    if(!getline(csv_stream, line))
    {
      break; // end of csv
    }

    std::stringstream line_stream(line);
    getline(line_stream, key_str, ',');
    key = std::stod(key_str);
    getline(line_stream, seq_str, ',');
    seq = std::stod(seq_str);
    getline(line_stream, sec_str, ',');
    getline(line_stream, nsec_str, ','); // sec, nsec values are unused
    getline(line_stream, x_str, ',');
    x = std::stod(x_str);
    getline(line_stream, y_str, ',');
    y = std::stod(y_str);
    getline(line_stream, z_str, ',');
    z = std::stod(z_str);
    getline(line_stream, roll_str, ',');
    roll = std::stod(roll_str);
    getline(line_stream, pitch_str, ',');
    pitch = std::stod(pitch_str);
    getline(line_stream, yaw_str);
    yaw = std::stod(yaw_str);

    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL) // no data?
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }
    else // check whether the sequence match
    {
      if(input_cloud->header.seq != seq)
      {
        std::cout << "Error: input_cloud->header.seq != seq (" << input_cloud->header.seq << " != " << seq << ")" << std::endl;
        return -1;
      }
    }
    // Skip unadded scan (has key = 0)
    if(key == 0)
    {
      std::cout << "Skipping unadded scan." << std::endl;
      continue;
    }

    // Create transformation matrix
    Eigen::Affine3f transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

    // Transform pointcloud
    pcl::PointCloud<pcl::PointXYZI> src, dst;
    pcl::fromROSMsg(*input_cloud, src);
    pcl::transformPointCloud(src, dst, transform);

    // Add to map
    map += dst;

    // Show output
    std::cout << "Number: " << key << "\n";
    std::cout << "Sequence: " << seq << "\n";
    std::cout << "Number of scan points: " << dst.size() << "\n";
    std::cout << "Number of map points: " << map.size() << "\n";
    std::cout << "(" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ")\n";
    std::cout << "Transformation Matrix: \n";
    std::cout << transform.matrix() << std::endl;
    std::cout << "---------------------------------------" << std::endl;
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  std::string out_filename = "merged_map";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", map);
  std::cout << "Finished. Saved " << map.size() << " pointcloud to " << out_filename << ".pcd" << std::endl;
  // pcl::io::savePLYFileBinary(out_filename + ".ply", map);
  return 0;
}
