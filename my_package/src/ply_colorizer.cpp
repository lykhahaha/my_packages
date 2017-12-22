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
  // ros::init(argc, argv, "ply_colorizer");
  // ros::NodeHandle n;

  if(argc != 3)
  {
  	std::cout << "Usage: rosrun my_package [input.pcd] [output.ply]" << std::endl;
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

  std::string csv = "/home/zwu/colormap.csv";
  std::cout << "Processing " << csv << std::endl;
	std::ifstream csv_stream(csv);

	// Load color into array
	int color_map[256][3];
	std::string line, r_str, g_str, b_str;
	int map_count = 0;
	while(getline(csv_stream, line))
  {
  	std::stringstream line_stream(line);

    // Get color RGB
    getline(line_stream, r_str, ',');
    color_map[map_count][0] = std::stod(r_str);
    getline(line_stream, g_str, ',');
    color_map[map_count][1] = std::stod(g_str);
    getline(line_stream, b_str, '\n');
    color_map[map_count][2] = std::stod(b_str);
    map_count++;
	}

	// Get max and min z
	double max_z = -99999;
	double min_z = 99999;
	for(size_t i = 0; i < input.size(); i++)
	{	
		if(input.points[i].z > max_z)
			max_z = input.points[i].z;
		if(input.points[i].z < min_z)
			min_z = input.points[i].z;
	}

  // Create another pointcloud with RGB values
  pcl::PointCloud<pcl::PointXYZRGB> output;
	output.width = input.width;
	output.height = input.height;
	output.points.resize(output.width * output.height);

	for(size_t i = 0; i < output.points.size(); i++)
	{
	  output.points[i].x = input.points[i].x;
	  output.points[i].y = input.points[i].y;
	  output.points[i].z = input.points[i].z;
	  // int value = input.points[i].intensity;
	  int value = (int)((input.points[i].z - min_z)/(max_z - min_z)*255);
		int r = color_map[value][2];
		int g = color_map[value][1];
		int b = color_map[value][0];
		// output.points[i].rgb = (uint32_t)(r) << 16 
  //   											| (uint32_t)(g) << 8
  //   											| (uint32_t)(b);
		output.points[i].r = r;
		output.points[i].g = g;
		output.points[i].b = b;
	}

  // Save point cloud to pcd
  std::string out_filename = argv[2];
  pcl::io::savePLYFileBinary(out_filename, output);
  std::cout << output.size() << " points have been saved to " << out_filename << std::endl;

  return 0;
}