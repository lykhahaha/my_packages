#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	sensor_msgs::PointCloud2 output = *input;
  output.header.frame_id = "horizontal_vlp16_link";
  pub.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_frame_conversion");

  ros::NodeHandle nh;
  
  pub = nh.advertise<sensor_msgs::PointCloud2>("/horizontal_laser_3d", 1000);
  ros::Subscriber points_sub = nh.subscribe("velodyne_points", 10000, points_callback);

  ros::spin();

  return 0;
}
