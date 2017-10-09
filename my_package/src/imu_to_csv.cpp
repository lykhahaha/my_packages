
//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
//C++ standard libs
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <iomanip>
//Namespaces
using namespace ros;
using namespace std;

std::ofstream outp;
std::string out_file = "/home/zwu/kitti-coor.csv";

void imu2csvCb(const geometry_msgs::TransformStamped& tf)
{
  // Process gps data
  double x = tf.transform.translation.x;
  double y = tf.transform.translation.y;
  double z = tf.transform.translation.z;

  // Out to csv file
  outp << x << "," << y << "," << z << std::endl;

  // Show
  ROS_INFO("Vehicle @(%f, %f, %f).", x, y, z);
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "imu_to_csv");
  ros::NodeHandle nh;

  outp.open(out_file);
  outp.precision(10);
  outp << "x" << "," << "y" << "," << "z" << std::endl;
 
  //Start ROS subscriber...
  ros::Subscriber gps_sub = nh.subscribe("/transform_imu", 10000, imu2csvCb);
  ros::spin();
  return 0;
}