/*This node funtion(s):
	+ Detects arrows and outputs the biggest arrow's direction
*/

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
//OpenCV libs
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//C++ standard libs
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <math.h>
//Namespaces
using namespace ros;
using namespace cv;
using namespace std;
//ROS params
std::string subscribed_image_topic;
std::string subscribed_laser_topic;
std::string published_topic;
bool debug;
//Image transport vars
cv_bridge::CvImagePtr cv_ptr;
//ROS var
vector<geometry_msgs::Point> destination_position;
//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Point> out_contours, hull;
std::vector<cv::Vec4i> hierarchy;
std::vector<int> contour_index;
cv::Mat src, hsv, dst, gray, dst2, detected_edges;
cv::Mat lower_hue_range, upper_hue_range;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
cv::Scalar up_lim1, low_lim1, up_lim2, low_lim2;
cv::Point arrow_direction, biggest_arrow_direction, arrow_center;
cv::Rect rect;
cv::RotatedRect mr;
int height, width;
int biggest_arrow_index;
int min_area = 500;
int cannyThreshold, accumulatorThreshold;
double area, mr_area, hull_area;
double max_angle_difference = 10*180/CV_PI;
double box_size = 0.2; //0.2m
double offset_distance = 0.2; //The robot will go to the point 0.2m in front of the hole of the box
const double eps = 0.15;

//Classes
class Box
{
public:
  geometry_msgs::Pose box_pose;
  ros::Time box_time;
  Box(geometry_msgs::Pose new_box_pose, ros::Time new_box_time)
    : box_pose(new_box_pose), box_time(new_box_time)
  {}
  double box_angle = -std::tan(box_pose.position.y/box_pose.position.x);
  int occurance = 0;
};

class Shape
{
public:
  int type;       //type = 0 for circles; type = 1 for arrows
  int direction;  //direction = 0 for left; direction = 1 for right; for circles, this parameter has no meaning
  cv::Point center;
  Shape(int new_shape_type, cv::Point new_shape_center, int new_shape_direction)
    : type(new_shape_type), center(new_shape_center), direction(new_shape_direction)
  {}
};

std::vector<Box> detected_box_vector;
std::vector<Shape> detected_shape_vector;

void arrowCb(const geometry_msgs::Vector3ConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //Get the image in OpenCV format
  src = cv_ptr->image;
  if(src.empty())
  {
    if(debug) ROS_INFO("Empty input. Looping...");
    return;
  }
  width = src.cols;
  height = src.rows;
  //Start the shape detection code
  cv::blur(src,src,Size(3,3));
  cv::cvtColor(src,hsv,COLOR_BGR2HSV);
  //Detect stuffs and show output on screen (in debug mode)
  detected_shape_vector.clear();
  detect_arrow();
  detect_circle();
  
  //Do the matching
  if(detected_shape_vector.empty() || detected_box_vector.empty()) return; //if either vector is empty: exit
  for(vector<Box>::iterator it = detected_box_vector.begin(); it != detected_box_vector.end(); ++it)
  {
    std::vector<Shape> matched_shape;
    for(vector<Shape>::iterator it2 = detected_shape_vector.begin(); it2 != detected_shape_vector.end(); it2++)
    {
      double shape_angle_pos = (2*((double)it2->center.x)/width-1)*39*180/CV_PI;        //horizontal view of camera is 78 degrees
      if(fabs(shape_angle_pos - it->box_angle) < max_angle_difference)
        matched_shape.push_back(*it2);    //Record all shapes that match the angle with the box
    }
    if(matched_shape.empty()) continue; //no matches
    Shape chosen_shape = *matched_shape.begin();
    if(matched_shape.size() > 1) //If there are many shapes that matched: choose the bottom-most one in the image (thus, the one with the highest y-value)
    {
      for(vector<Shape>::iterator it2 = matched_shape.begin()+1; it2 != matched_shape.end(); it2++)
        if(chosen_shape.center.y < it2->center.y) chosen_shape = *it2;
    }

    // The pair is: *it (for box) and chosen_shape (for shape)
    // Calculate the destination location for navigation node
    if(chosen_shape.type == 0) //Circle
    {
      geometry_msgs::Point temp_destination_holder;
      //Transform quaternion data into angle data (yaw)
      tf::Quaternion q(it->box_pose.orientation.x, it->box_pose.orientation.y, it->box_pose.orientation.z, it->box_pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      //Calculate destination for navigation node from data
      temp_destination_holder.x = it->box_pose.position.x - offset_distance*sin(yaw); //x' = x - d*sin(yaw)
      temp_destination_holder.y = it->box_pose.position.y - offset_distance*cos(yaw); //y' = y - d*cos(yaw)
      destination_position.push_back(temp_destination_holder);
    }else if((chosen_shape.type == 1) && (chosen_shape.direction == 0)) //Arrow with left direction
    {
      geometry_msgs::Point temp_destination_holder;
      //Transform quaternion data into angle data (yaw)
      tf::Quaternion q(it->box_pose.orientation.x, it->box_pose.orientation.y, it->box_pose.orientation.z, it->box_pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      //Calculate destination for navigation node from data
      temp_destination_holder.x = it->box_pose.position.x + (box_size/2)*(-cos(yaw)+ sin(yaw)) - offset_distance*cos(yaw); //x' = x - a/2*cos(yaw) + a/2*sin(yaw) - d*cos(yaw)
      temp_destination_holder.y = it->box_pose.position.y + (box_size/2)*(sin(yaw) + cos(yaw)) + offset_distance*sin(yaw); //y' = y + a/2*sin(yaw) + a/2*cos(yaw) + d*sin(yaw)
      destination_position.push_back(temp_destination_holder);
    }else if((chosen_shape.type == 1) && (chosen_shape.direction == 1)) //Arrow with right direction
    {
      geometry_msgs::Point temp_destination_holder;
      //Transform quaternion data into angle data (yaw)
      tf::Quaternion q(it->box_pose.orientation.x, it->box_pose.orientation.y, it->box_pose.orientation.z, it->box_pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      //Calculate destination for navigation node from data
      temp_destination_holder.x = it->box_pose.position.x + (box_size/2)*(cos(yaw) + sin(yaw)) + offset_distance*cos(yaw); //x' = x + a/2*cos(yaw) + a/2*sin(yaw) + d*cos(yaw)
      temp_destination_holder.y = it->box_pose.position.y + (box_size/2)*(-sin(yaw)+ cos(yaw)) - offset_distance*sin(yaw); //y' = y - a/2*sin(yaw) + a/2*cos(yaw) - d*sin(yaw)
      destination_position.push_back(temp_destination_holder);
    }
  }
}

void laserCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  ros::Time current_time = ros::Time::now();
  Box new_box(msg->pose, current_time);
  detected_box_vector.push_back(new_box);
  while(!detected_box_vector.empty()) 
  {
    ros::Duration box_age = current_time - detected_box_vector[0].box_time;
    if(box_age.toSec() > 1) detected_box_vector.erase(detected_box_vector.begin());
    else break;
  }
  std::vector<Box>::iterator it = detected_box_vector.begin();
  while(it < (detected_box_vector.end()-1))
  {
    geometry_msgs::Point current_box_pos = it->box_pose.position;
    geometry_msgs::Point next_box_pos = (it+1)->box_pose.position;
    double distance_currentbox_nextbox = sqrt(pow((current_box_pos.x - next_box_pos.x),2) + pow((current_box_pos.y - next_box_pos.y),2) + pow((current_box_pos.z - next_box_pos.z),2));
    if(distance_currentbox_nextbox < 0.5) 
    {
      detected_box_vector.erase(it+1);
      it->occurance++;
    }
    else it++;
  }
}

static void onMouse(int event, int x, int y, int, void*)
{
  if(event == EVENT_LBUTTONDOWN)
  { 
    Vec3b pixel = hsv.at<Vec3b>(cv::Point(x,y));
    std::cout << "\tAt point [" << x << "," << y << "]: (" << (float)pixel.val[0] << ", " << (float)pixel.val[1] << ", " << (float)pixel.val[2] << ")\n";
  }
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "arrow");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_arrow_topic", subscribed_arrow_topic);
  pnh.getParam("subscribed_laser_topic", subscribed_laser_topic);
  pnh.getParam("debug", debug);
  pnh.getParam("published_topic", published_topic);
  //Start ROS subscriber...
  ros::Subscriber arrow_sub = nh.Subscribe(subscribed_arrow_topic, 1, arrowCb);
  ros::Subscriber laser_sub = nh.subscribe(subscribed_laser_topic, 1, laserCb);
  //...and ROS publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>(published_topic, 1000);
  ros::Rate r(30);
  while (nh.ok())
  {
  	//Publish every object detected
    for(vector<geometry_msgs::Pose>::iterator it = destination_position.begin(); it != destination_position.end(); it++)
      pub.publish(*it);
    //Reinitialize the object counting vars
    destination_position.clear();

    ros::spinOnce();
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}