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

  std::cout << "INFO: Reading data in the current directory." << std::endl;
  std::cout << "Please ensure that txts/ directory IS CREATED in this directory." << std::endl;
  std::cout << "Also ensure that ONLY .pcd files (either binary or ascii) are in this directory." << std::endl;

  std::string p(argc <= 1 ? "." : argv[1]);
  if(boost::filesystem::is_directory(p))
  {
    for(boost::filesystem::directory_iterator itr(p); itr != boost::filesystem::directory_iterator(); ++itr) // Loop through files
    {
      std::string filename = itr->path().filename().string();
      std::cout << "Processing: " << filename;

      if(boost::filesystem::is_regular_file(itr->status())) // if it is a file
        std::cout << " [" << boost::filesystem::file_size(itr->path()) << " bytes]" << std::endl;
      else
      {
        std::cout << "[directory], skipped." << std::endl;
        continue;
      }

    	// pcl::PointCloud<pcl::PointXYZI> input;
    	pcl::PointCloud<pcl::PointXYZI> input;
      if(pcl::io::loadPCDFile<pcl::PointXYZI> (filename, input) == -1)
      {
        std::cout << "Couldn't read " << filename << "." << std::endl;
        return(-1);
      }
      std::cout << "Loaded " << input.size() << " data points from " << filename << std::endl;

      std::string outfile = filename;
    	outfile = filename;
    	outfile.resize(outfile.size()-4); // erase the .pcd extension
    	outfile += ".txt";

    	std::ofstream txt_stream;
    	txt_stream.open("txts/" + outfile);
    	txt_stream << input.size() << std::endl;

    	// Iterate through every point and output its [x, y, z, intensity]
    	for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = input.begin(); item < input.end(); item++)
      	txt_stream << (double)item->x << " " << (double)item->y << " " << (double)item->z << " " << (double)item->intensity << std::endl;

      // Save point cloud to pcd
      std::cout << input.size() << " points data have been saved to txts/" << outfile << std::endl;
    }
  }
  return 0;
}