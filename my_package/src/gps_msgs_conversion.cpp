//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//msgs libs
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "rds_msgs/msg_novatel_bestpos.h"
#include "rds_msgs/msg_novatel_inscov.h"
//C++ standard libs
#include <iostream>
#include <stdlib.h>
#include <vector>
//Namespaces
using namespace ros;
using namespace std;
using namespace message_filters;

//Global vars
ros::Publisher gps_pub;
std::vector<double> pos3ition_covariance;

void syncCb(const geometry_msgs::PoseWithCovarianceStamped& gps_pos, const rds_msgs::msg_novatel_bestpos& gps_bestpos)
{
  sensor::NavSatFix gps_msg;

  // Append all applicable data from msg_novatel_bestpos to NavSatFix msg
  gps_msg.header = gps_pos.header;

  gps_msg.NavSatStatus.status = 3;
  gps_msg.NavSatStatus.service = 1;

  gps_msg.latitude = gps_bestpos.latitude;
  gps_msg.longitude = gps_bestpos.longitude;
  gps_msg.altitude = gps_bestpos.height;

  gps_msg.position_covariance = position_covariance;
  gps_msg.position_covariance_type = 3;

  gps_pub.pub(gps_msg);
  return;
}

void insCovCb(const rds_msgs::msg_novatel_inscov& gps_inscov)
{
  position_covariance = gps_inscov.position_covariance;
  return;
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "gps_msgs_conversion");
  ros::NodeHandle nh;
  //Start ROS subscriber...
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> gps_position_sub(nh, "gps_position", 1000);
  message_filters::Subscriber<rds_msgs::msg_novatel_bestpos> best_pos_sub(nh, "/gps_bestpos", 1000);
  message_filters::TimeSynchronizer<Image, Image> sync(gps_position_sub, best_pos_sub, 50);
  sync.registerCallback(boost::bind(&syncCb, _1, _2));

  ros::Subscriber ins_cov_sub = nh.subscribe("/gps_inscov", 1000, insCovCb);
  //and ROS publisher...
  gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}