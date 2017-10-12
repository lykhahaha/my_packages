/***
Author: TA
I wrote this piece of code to input pose data from a .csv file
then calculate pose of scan in the frame of previous scan
***/
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_datatypes.h>

// #include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char** argv)
{
  // const double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;
  // Eigen::Affine3d previous_pose, current_pose;
  // pcl::getTransformation(0,0,0,0,0.01,0, previous_pose);
  // pcl::getTransformation(0,0,1,0,0.025,0, current_pose);
  // Eigen::Affine3d relative_pose = previous_pose.inverse() * current_pose;

  // std::cout << "previous pose:\n";
  // std::cout << previous_pose.matrix() << "\n\n";
  // std::cout << "Previous Pose Inversed:\n";
  // std::cout << previous_pose.matrix().inverse() << "\n\n";
  // std::cout << "current_pose:\n";
  // std::cout << current_pose.matrix() << "\n\n";
  // std::cout << "relative_pose:\n";
  // std::cout << relative_pose.matrix() << std::endl << std::endl;

  // double x, y, z, roll, pitch, yaw;
  // pcl::getTranslationAndEulerAngles(relative_pose, x, y, z, roll, pitch, yaw);
  // std::cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
  // std::cout << "---------------------------------------" << std::endl;    

  // Input csv
	std::string csv_in = "allpose_data.csv";
  std::cout << "Processing " << csv_in << " in the current directory" << std::endl;
	std::ifstream csv_stream(csv_in);

  // Output csv
  std::ofstream out_stream;
  std::string csv_out = "delta_pose.csv";
  out_stream.open(csv_out);
  out_stream << "rostime,dx,dy,dz,droll,dpitch,dyaw" << std::endl;

  // Place-holder for csv stream variables
	std::string line, time_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  Eigen::Affine3d current_pose, previous_pose, relative_pose;

  // Get first pose to previous pose
  {
    getline(csv_stream, line);
    getline(csv_stream, line); // to skip header

    std::stringstream line_stream(line);

    getline(line_stream, time_str, ',');

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

    pcl::getTransformation(x, y, z, roll, pitch, yaw, previous_pose);
  }

	// Get the rest of data from csv
  while(getline(csv_stream, line))
  {
  	std::stringstream line_stream(line);

    // Get data value
    getline(line_stream, time_str, ','); // we dont really need timing here
    
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
    // std::cout << "Original pose data:\n";
    // std::cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;

    // Get transformation
    pcl::getTransformation(x, y, z, roll, pitch, yaw, current_pose);
    relative_pose = previous_pose.inverse() * current_pose;
    pcl::getTranslationAndEulerAngles(relative_pose, x, y, z, roll, pitch, yaw);
    
    // Show output
    std::cout << "previous pose:\n";
    std::cout << previous_pose.matrix() << "\n\n";
    std::cout << "Previous Pose Inversed:\n";
    std::cout << previous_pose.matrix().inverse() << "\n\n";
    std::cout << "current_pose:\n";
    std::cout << current_pose.matrix() << "\n\n";
    std::cout << "relative_pose:\n";
    std::cout << relative_pose.matrix() << std::endl << std::endl;
    std::cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
    std::cout << "---------------------------------------" << std::endl;   

    // Update
    out_stream << time_str << "," << x << "," << y << "," << z << "," 
               << roll << "," << pitch << "," << yaw << std::endl;
    previous_pose = current_pose;
	}

  std::cout << "delta_pose.csv written into the current directory." << std::endl;
  return 0;
}
