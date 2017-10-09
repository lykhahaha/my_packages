#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
  ros::Rate loop_rate(50);
  sensor_msgs::Imu msg;

  msg.header.seq = 0;
  msg.header.frame_id = "imu_link";

  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 0.0;
  msg.orientation_covariance[0] = -1;

  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = 0.0;
  msg.angular_velocity.z = 0.0;

  msg.angular_velocity_covariance[0] = 0;

  msg.linear_acceleration.x = 0.5;
  msg.linear_acceleration.y = 0;
  msg.linear_acceleration.z = -9.81;

  while (nh.ok()) 
  {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}