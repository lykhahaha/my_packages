#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <algorithm>

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcd_to_nvtxt");
  ros::NodeHandle n;

  if(argc < 2)
  {
  	std::cout << "Usage: rosrun my_package \"input.pcd\" [output.txt] " << std::endl;
  	return(-1);
  }
  std::string filename = argv[1];
  std::cout << "Loading file: " << filename << std::endl;

	// pcl::PointCloud<pcl::PointXYZI> input;
	pcl::PointCloud<pcl::PointXYZI> input;
  if(pcl::io::loadPCDFile<pcl::PointXYZI> (filename, input) == -1)
  {
    std::cout << "Couldn't read " << filename << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << input.size() << " data points from " << filename << std::endl;

  std::string outfile;
  if(argc < 3)
  {
  	std::cout << "Output text file not indicated. Using input file name." << std::endl;
  	outfile = argv[1];
  	outfile.resize(outfile.size()-4); // erase the .pcd extension
  	outfile += ".txt";
  }
  else 
  	outfile = argv[2];

	std::ofstream txt_stream;
	txt_stream.open(outfile);
	txt_stream << input.size() << std::endl;

	// Iterate through every point and output its [x, y, z, intensity]
	for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = input.begin(); item < input.end(); item += 6)
  	txt_stream << (double)item->x << " " << (double)item->y << " " << (double)item->z << " " << (double)item->intensity << std::endl;

  // Save point cloud to pcd
  std::cout << input.size() << " points data have been saved to " << outfile << std::endl;

  return 0;
}