#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>

#include <fstream>
#include <sstream>
#include <string>
#include <cstddef>

int main(int argc, char** argv)
{
  std::cout << "Reads map_pose.csv in the current directory,\n";
  std::cout << "Load the pcd files indicated by the csv and merge them." << std::endl;
  // Initiate and get csv file
  ros::init(argc, argv, "submap_merger");

	std::string tf_map_csv_file_name = "map_pose.csv";
	std::ifstream csv_stream(tf_map_csv_file_name);
  std::cout << "Processing " << tf_map_csv_file_name << std::endl;

  // Place-holder for variables
	std::string line, pcd_filename, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  pcl::PointCloud<pcl::PointXYZI> map;

  getline(csv_stream, line); // to skip header line
  
  while(getline(csv_stream, line))
  {
    std::stringstream line_stream(line);

    // Get pcd file
		getline(line_stream, pcd_filename, ',');
		std::cout << "Loading file: " << pcd_filename << std::endl;

    pcl::PointCloud<pcl::PointXYZI> src;
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, src) == -1)
    {
      std::cout << "Couldn't read " << pcd_filename << "." << std::endl;
      return(-1);
    }
    std::cout << "Loaded " << src.size() << " data points from " << pcd_filename << std::endl;
    
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

    // Transform pointcloud
    pcl::PointCloud<pcl::PointXYZI> dst, scan_dst;
    pcl::transformPointCloud(src, dst, transform);

    // Add to map
    map += dst;

    // Show output
    std::cout << "Number of scan points: " << dst.size() << std::endl;
    std::cout << "Number of map points: " << map.size() << std::endl;
    std::cout << "(" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
    std::cout << "Transformation Matrix: " << std::endl;
    std::cout << transform.matrix() << std::endl;
    std::cout << "---------------------------------------" << std::endl;
  }
  std::cout << "Finished processing pcd files." << std::endl;
  std::string out_filename = "merged_map";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", map);
  std::cout << "Finished. Saved " << map.size() << " pointcloud to " << out_filename << ".pcd" << std::endl;
  // pcl::io::savePLYFileBinary(out_filename + ".ply", map);
  return 0;
}
