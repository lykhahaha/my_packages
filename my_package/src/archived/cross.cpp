/*This node funtion(s):
	+ Detect symbols: blue cross, red cross, gray circle
*/

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
//OpenCV libs
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//C++ standard libs
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
//Namespaces
using namespace ros;
using namespace cv;
using namespace std;
//ROS params
std::string subscribed_image_topic;
std::string object_shape, object_color;
std::string published_topic;
bool debug;
//Image transport vars
cv_bridge::CvImagePtr cv_ptr;
//ROS var
sensor_msgs::RegionOfInterest object;
//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Mat src, hsv, hls;
cv::Scalar up_lim, low_lim, up_lim_wrap, low_lim_wrap;
cv::Mat lower_hue_range;
cv::Mat upper_hue_range;
cv::Mat color;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
cv::Rect rect;
cv::RotatedRect mr;
int height, width;
int min_area = 500;
double area, r_area, mr_area;
const double eps = 0.15;

//Functions
void reduce_noise(cv::Mat* dst)
{
  cv::morphologyEx(*dst, *dst, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*dst, *dst, cv::MORPH_OPEN, str_el);
}

sensor_msgs::RegionOfInterest object_return()
{
  sensor_msgs::RegionOfInterest obj;
  obj.x_offset = (rect.tl()).x;
  obj.y_offset = (rect.tl()).y;
  obj.height = rect.height;
  obj.width = rect.width;
  obj.do_rectify = true;
  return obj;
}

void object_found()
{
  if(rect.width*rect.height > object.height*object.width)
    object = object_return();         //Push the object to the vector
  if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
}

void detect_symbol()
{
  //Filter desired color
  //********************
  //In case of red color
  cv::inRange(hsv, cv::Scalar(0, 100, 70), cv::Scalar(25, 255, 255), lower_hue_range);
  cv::inRange(hsv, cv::Scalar(163, 100, 70), cv::Scalar(179, 255, 255), upper_hue_range);
  cv::addWeighted(lower_hue_range,1.0,upper_hue_range,1.0,0.0,color);
  //Reduce noise
  reduce_noise(&color);
  if(debug) cv::imshow("red", color);
  //Finding shapes
  cv::findContours(color.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;

    rect = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    area = cv::contourArea(contours[i]);
    mr_area = (mr.size).height*(mr.size).width;

    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);
    double hull_area = contourArea(hull);

    //Circle detection
    if((std::fabs(area/mr_area - 3.141593/4) < 0.08) && (std::fabs(area/hull_area - 1) < 0.04)
                                                    && (std::fabs((float)rect.height/rect.width - 1) < 0.7)
                                                    && (hierarchy[i][2] != -1))
      object_found();
  }
  //********************
  //In case of blue color
  cv::inRange(hsv, cv::Scalar(100, 100, 100), cv::Scalar(120, 230, 230), color);
  reduce_noise(&color);
  if(debug) cv::imshow("blue", color);
  //Finding shapes
  cv::findContours(color.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;

    rect = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    area = cv::contourArea(contours[i]);
    mr_area = (mr.size).height*(mr.size).width;

    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);
    double hull_area = contourArea(hull);

    //Circle detection
    if((std::fabs(area/mr_area - 3.141593/4) < 0.08) && (std::fabs(area/hull_area - 1) < 0.04)
                                                    && (std::fabs((float)rect.height/rect.width - 1) < 0.7)
                                                    && (hierarchy[i][2] != -1))
      object_found();
  }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
  if (src.empty()) return;
  //Start the shape detection code
  cv::blur(src,src,Size(1,1));
  cv::cvtColor(src,hsv,COLOR_BGR2HSV);
  width = src.cols;
  height = src.rows;
  //Detect stuffs
  detect_symbol();
  //Show output on screen in debug mode
  if(debug) cv::imshow("src", src);
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "detection_roi");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("debug", debug);
  pnh.getParam("published_topic", published_topic);
  //Initiate windows
  if(debug)
  {
   /* cv::namedWindow("color",WINDOW_AUTOSIZE);
    cv::namedWindow("src",WINDOW_AUTOSIZE);*/
    cv::namedWindow("red",WINDOW_NORMAL);
    cv::resizeWindow("red",640,480);
    cv::moveWindow("red", 0, 600);
    cv::namedWindow("blue",WINDOW_NORMAL);
    cv::resizeWindow("blue",640,480);
    cv::moveWindow("blue", 700, 0);
    cv::namedWindow("src",WINDOW_NORMAL);
    cv::resizeWindow("src",640,480);
    cv::moveWindow("src", 0, 0);
    cv::startWindowThread();
  }
  //Start ROS subscriber...
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscribed_image_topic, 1, imageCb);
  //...and ROS publisher
  ros::Publisher pub = nh.advertise<sensor_msgs::RegionOfInterest>(published_topic, 1000);
  ros::Rate r(30);
  while (nh.ok())
  {
  	//Publish every object detected
    pub.publish(object);
    ros::spinOnce();
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}