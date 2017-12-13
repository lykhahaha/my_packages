/*This node funtion(s):
	+ Detects arrows and outputs the biggest arrow's direction
*/

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include "rds_msgs/msg_novatel_bestpos.h"
//OpenCV libs
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//C++ standard libs
#include <iostream>
#include <stdlib.h>
#include <math.h>
//Namespaces
using namespace ros;
using namespace cv;
using namespace std;

double tl_latitude = 1.344261; // Top-left location
double tl_longitude = 103.703863; 
double br_latitude = 1.333980; // Bottom-right location
double br_longitude = 103.723592;
double scale = 0.845; // means 1 meter = 'scale' pixels on img
int width, height;
cv::Mat img;

void gps2img_Cb(const rds_msgs::msg_novatel_bestpos &bestpos)
{
  // Process gps data to put on image
	double x = (bestpos.longitude - tl_longitude)/(br_longitude - tl_longitude)*width;
  double y = (bestpos.latitude - tl_latitude)/(br_latitude - tl_latitude)*height;
  cv::circle(img, cv::Point(x, y), 3, cv::Scalar(255, 0, 0), 1, 8, 0);

  // Show
  ROS_INFO("Vehicle @%.0f,%.0f on image.", x, y);
  cv::imshow("src", img);
  cv::waitKey(10);
}

void onMouse(int event, int x, int y, int, void*)
{
	if(event == EVENT_LBUTTONDOWN)
		cv::imwrite("/home/zwu/map-with-gps.png", img);
	return;
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "gps2img");
  ros::NodeHandle nh;
 
  //Initiate windows
	cv::namedWindow("src", WINDOW_NORMAL);
	// cv::resizeWindow("src", 640, 480);
	// cv::moveWindow("src", 0, 0);
	cv::setMouseCallback("src", onMouse);
	cv::startWindowThread();
	// Get image
  img = cv::imread("/home/zwu/catkin_ws/src/my_gps_visualization/src/map.png");
  if(img.empty()) 
  {
  	ROS_INFO("Cannot load image.");
  	return 0;
  }
  width = img.cols;
  height = img.rows;
	cv::waitKey(10);
  //Start ROS subscriber...
  ros::Subscriber gps_sub = nh.subscribe("/gps_bestpos", 1000, gps2img_Cb);
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}