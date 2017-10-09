/*This node funtion(s):
	+ Detect 3x3 green matrix board
  + Give output as ROI
*/

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <base_vision/armor_colorConfig.h>
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
//Namespaces
using namespace ros;
using namespace cv;
using namespace std;
//ROS params
std::string subscribed_image_topic;
std::string armor_color;
std::string published_topic;
int detection_mode;
bool debug;
//Image transport vars
cv_bridge::CvImagePtr cv_ptr;
//ROS var
vector<sensor_msgs::RegionOfInterest> object;
//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Point> approx;  
cv::Mat src, hsv, dst;
cv::Scalar up_lim, low_lim;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));
cv::Rect rect;
cv::RotatedRect mr;
// cv::RNG rng(12345);
int height, width;
int min_area = 200;
float min_height_ratio = 0.03;
double area, r_area, mr_area;
const double eps = 0.15;
const double angle_error = 10; //degree

//Class
class myRect{
public:
  std::vector<cv::Point> vertices;
  cv::Point center;
  cv::Rect ROI;
  double max_neighbor_distance;
  unsigned int cluster_group;
  int index;
  myRect(std::vector<cv::Point>, int);
};

myRect::myRect(std::vector<cv::Point> vertices, int contourIdx): 
  center((vertices[0] + vertices[1] + vertices[2] + vertices[3])*0.25), 
  vertices(vertices),
  index(contourIdx)
{
  cv::Point diagonal = vertices[2] - vertices[0];
  max_neighbor_distance = 2*sqrt(pow(diagonal.x, 2) + pow(diagonal.y, 2));
  cluster_group = 0;
  ROI = cv::boundingRect(vertices);
}
std::vector<myRect> greenRect;
std::vector<std::vector<myRect> > matrix_board_groups;

//Functions
inline void reduce_noise(cv::Mat* dst)
{
  cv::morphologyEx(*dst, *dst, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*dst, *dst, cv::MORPH_OPEN, str_el);
}
inline bool are_lines_parallel(double angle1, double angle2)
{
  double del_angle = std::fabs(angle1 - angle2);
  if((del_angle < angle_error) || (std::fabs(del_angle - 180.0) < angle_error))
    return true;
  return false;
}
bool is_trapezoid(std::vector<cv::Point> vertices)
{
  if(vertices.size() != 4)
    return false;
  double edge_angles[4];
  for(int i = 0; i < 4; i++)
    edge_angles[i] = atan2(vertices[(i+1)%4].y - vertices[i].y, 
                            vertices[(i+1)%4].x - vertices[i].x + 1e-10) * 180.0 / 3.14159265359;
  if(are_lines_parallel(edge_angles[0], edge_angles[2]) && are_lines_parallel(edge_angles[1], edge_angles[3]))
    return true;
  return false;
}

void found_matrix3x3board(std::vector<myRect> cluster)
{
  sensor_msgs::RegionOfInterest obj; //Place-holder for a new ROI
  int TLx = width, TLy = height, BRx = 0, BRy = 0; //Place-holder for top-left & bot-right coordinates of ROI

  //Find the ROI that envelop the cluster
  for(int i = 1; i < cluster.size(); i++)
  {
    if(TLx > cluster[i].ROI.tl().x) TLx = cluster[i].ROI.tl().x;
    if(TLy > cluster[i].ROI.tl().y) TLy = cluster[i].ROI.tl().y;
    if(BRx < cluster[i].ROI.br().x) BRx = cluster[i].ROI.br().x;
    if(BRy < cluster[i].ROI.br().y) BRy = cluster[i].ROI.br().y;
  }
  if((TLx < 0) || (TLy < 0) || (BRx - TLx) < 0 || (BRy - TLy) < 0)
    return;
  obj.x_offset = TLx;
  obj.y_offset = TLy;
  obj.width = BRx - TLx;
  obj.height = BRy - TLy;
  obj.do_rectify = true;
  object.push_back(obj);  //Push the object to the vector
  if(debug) cv::rectangle(src, cv::Point(TLx, TLy), cv::Point(BRx, BRy), cv::Scalar(0,0,255), 2, 8, 0);
  return;
}

void detect_matrix3x3board()
{
  cv::inRange(hsv, low_lim, up_lim, dst);
  //Reduce noise
  // reduce_noise(&dst);
  if(debug) cv::imshow("green", dst);
  //Finding every small green rectangular screen
  cv::findContours(dst.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  // ROS_INFO("New loop........");
  for(int i = 0; i < contours.size(); i++)
  {
    area = cv::contourArea(contours[i]);
    //Delete the contours that are too small
    if(area < min_area)
      continue;

    // Approximate contour with accuracy proportional to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.05, true);

    // Skip non-convex objects 
    if (!cv::isContourConvex(approx))
      continue;
    if(is_trapezoid(approx))
    {
      // cv::drawContours(src, contours, i, cv::Scalar(0,0,255), 4, 8, hierarchy, 0);
      greenRect.push_back(myRect(approx, i));
    }
  }
  //Merge the detected screens together to find the matrix board
  //Do clustering here
  int cluster_number = 0;
  for(int i = 0; i < greenRect.size(); i++)
  {
    if(greenRect[i].cluster_group == 0) 
      greenRect[i].cluster_group = ++cluster_number;
    for(int j = i + 1; j < greenRect.size(); j++)
    {
      //Already incluster, skip
      if(greenRect[j].cluster_group == greenRect[i].cluster_group) 
        continue;

      //Find distance between 2 rects' centers
      cv::Point center_distance_vector = greenRect[j].center - greenRect[i].center;
      double center_distance = sqrt(pow(center_distance_vector.x, 2) + pow(center_distance_vector.y, 2));

      //If 2 rects are close -> group into 1 cluster
      if(center_distance < greenRect[i].max_neighbor_distance && center_distance < greenRect[j].max_neighbor_distance)
        if(greenRect[j].cluster_group == 0) //In case greenRect[j] is a newly accessed rect
          greenRect[j].cluster_group = greenRect[i].cluster_group;
        else //In case greenRect[j] is an already accessed rect of a different cluster
        {  //In that case, merge the 2 clusters
          cluster_number--;
          int cluster_remove_number = greenRect[j].cluster_group;
          for(int k = 0; k < greenRect.size(); k++)
          {
            if(greenRect[k].cluster_group == cluster_remove_number)
              greenRect[k].cluster_group = greenRect[i].cluster_group;
            else if(greenRect[k].cluster_group > cluster_remove_number)
              greenRect[k].cluster_group -= 1;
          }
        }
    }
  }
  //Visualization/debugging
  for(int i = 1; i <= cluster_number; i++)
  {
    //Visualize each cluster (debug mode)
    // cv::Scalar rndColor = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    std::vector<myRect> new_cluster;
    for(int j = 0; j < greenRect.size(); j++)
    {
      if(greenRect[j].cluster_group == i)
      {
        new_cluster.push_back(greenRect[j]);
        // if(debug) cv::drawContours(src, contours, greenRect[j].index, rndColor, 4, 8, hierarchy, 0);
      }
    }

    //Forward cluster to output
    found_matrix3x3board(new_cluster);
  }
  greenRect.clear();
  return;
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
  if(src.empty())
  {
    if(debug) ROS_INFO("Empty input. Looping...");
    return;
  }
  width = src.cols;
  height = src.rows;
  //Crop image, note that this doesnt copy the data
  // croppedRef = src(cv::Rect(0, height/4, width, 3*height/4));
  // croppedRef.copyTo(cropped); //change src to crop below to crop the image b4 processing

  //Start the shape detection code
  cv::blur(src,src,Size(2,2));
  cv::cvtColor(src,hsv,COLOR_BGR2HSV);
  //Detect stuffs
  detect_matrix3x3board();
  //Show output on screen in debug mode
  if(debug) 
  {
    cv::imshow("src", src);
  }
}

void dynamic_configCb(base_vision::armor_colorConfig &config, uint32_t level) 
{
  min_area = config.min_area;
  //Process appropriate parameter for armor color
  low_lim = cv::Scalar(config.green_H_low, config.green_S_low, config.green_V_low);
  up_lim = cv::Scalar(config.green_H_high, config.green_S_high, config.green_V_high);
  ROS_INFO("Reconfigure Requested.");
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "armor_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("debug", debug);
  pnh.getParam("published_topic", published_topic);
  dynamic_reconfigure::Server<base_vision::armor_colorConfig> server;
  dynamic_reconfigure::Server<base_vision::armor_colorConfig>::CallbackType f;
  f = boost::bind(&dynamic_configCb, _1, _2);
  server.setCallback(f);
  
  //Initiate windows
  if(debug)
  {
   /* cv::namedWindow("color",WINDOW_AUTOSIZE);
    cv::namedWindow("src",WINDOW_AUTOSIZE);*/
    cv::namedWindow("src",WINDOW_NORMAL);
    cv::resizeWindow("src",640,480);
    cv::moveWindow("src", 0, 0);
    cv::namedWindow("green",WINDOW_NORMAL);
    cv::resizeWindow("green",640,480);
    cv::moveWindow("green", 0, 600);
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
    for(vector<sensor_msgs::RegionOfInterest>::iterator it = object.begin(); it != object.end(); it++)
      pub.publish(*it);
    //Reinitialize the object counting vars
    object.clear();

    ros::spinOnce();
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}