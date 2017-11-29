// #include <ros/ros.h>
// #include <dynamic_reconfigure/server.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <my_package/transformPointsConfig.h>

// // #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
#include <pcl/common/common.h>
// #include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
// #include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>
// #include <sstream>
#include <string>
// #include <vector>

int main(int argc, char** argv)
{
  // Load input
  std::cout << "This node is used to transform trajectory arbitrarily." << std::endl;
  std::cout << "I used it to make the trajectory plotted in python more horizontal." << std::endl;
  if(argc != 4)
  {
    std::cout << "Usage: rosrun my_package transform_trajectory \"roll\" \"pitch\" \"yaw\"" << std::endl;
    return -1;
  }

  std::string filename = "/home/zwu/24nov-1406/map_pose.csv";
  std::cout << "Loading file: " << filename  << std::endl;
  std::ifstream in_stream(filename);

  // Output holder
  std::ofstream out_stream;
  std::string out_csv = "/home/zwu/tilted_trajectory.csv";
  out_stream.open(out_csv);
  out_stream << "key,sequence,sec,nsec,x,y,z,roll,pitch,yaw" << std::endl;

  // Transformation matrix for csv
  double roll = std::stod(argv[1]);
  double pitch = std::stod(argv[2]);
  double yaw = std::stod(argv[3]);
  Eigen::Affine3d global_transform;
  pcl::getTransformation(0, 0, 0, roll, pitch, yaw, global_transform);

  // Place-holder for csv stream variables
  std::string line, key_str, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  Eigen::Affine3d current_pose;
  getline(in_stream, line);
  std::cout << "File sequence: " << line << std::endl;

  while(getline(in_stream, line))
  {
    std::stringstream line_stream(line);

    // Get data value
    getline(line_stream, key_str, ',');
    getline(line_stream, seq_str, ',');
    getline(line_stream, sec_str, ',');
    getline(line_stream, nsec_str, ',');
    getline(line_stream, x_str, ',');
    // double x = std::stod(x_str);
    getline(line_stream, y_str, ',');
    // double y = std::stod(y_str);
    getline(line_stream, z_str, ',');
    // double z = std::stod(z_str);
    getline(line_stream, roll_str, ',');
    // double roll = std::stod(roll_str);
    getline(line_stream, pitch_str, ',');
    // double pitch = std::stod(pitch_str);
    getline(line_stream, yaw_str);
    // double yaw = std::stod(yaw_str);

    // Get transformation
    pcl::getTransformation(std::stod(x_str), std::stod(y_str), std::stod(z_str),
                           std::stod(roll_str), std::stod(pitch_str), std::stod(yaw_str),
                           current_pose);
    Eigen::Affine3d transformed_pose = global_transform * current_pose;
    double x,y,z,roll,pitch,yaw;
    pcl::getTranslationAndEulerAngles(transformed_pose, x, y, z, roll, pitch, yaw);

    // Update
    out_stream << key_str << "," << seq_str << "," << sec_str << "," << nsec_str << ","
               << x << "," << y << "," << z << "," 
               << roll << "," << pitch << "," << yaw << std::endl;
  }
  std::cout << "Finished. Output: /home/zwu/tilted_trajectory.csv" << std::endl;
  return 0;
}
