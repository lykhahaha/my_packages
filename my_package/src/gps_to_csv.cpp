
//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include "rds_msgs/msg_novatel_bestpos.h"
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
std::string out_file = "/home/zwu/coordinate.csv";

void gsp2csvCb(const rds_msgs::msg_novatel_bestpos &bestpos)
{
  // Process gps data
  double x = bestpos.latitude;
  double y = bestpos.longitude;
  double h = bestpos.height;
  double t = bestpos.gps_millisecs;

  // Out to csv file
  std::cout << std::fixed;
  std::cout << std::setprecision(6);
  outp << t << "," << x << "," << y << "," << h << std::endl;

  // Show
  ROS_INFO("Vehicle @(%f, %f, %f) at milisec @%f.", x, y, h, t);
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "gps_to_csv");
  ros::NodeHandle nh;

  outp.open(out_file);
  outp.precision(10);
  outp << "time" << "," << "latitude" << "," << "longitude" << "," << "altitude" << std::endl;
 
  //Start ROS subscriber...
  ros::Subscriber gps_sub = nh.subscribe("/gps_bestpos", 10000, gsp2csvCb);
  ros::spin();
  return 0;
}