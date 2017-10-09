/*This node funtion(s):
	+ Detect armor plates of enemy team's hero and infantries
  + Give output as ROI
*/

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
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
bool pubd = false;
ros::Subscriber csub;
//Image transport vars
cv_bridge::CvImagePtr cv_ptr;
//ROS var
vector<sensor_msgs::RegionOfInterest> object;
//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours, circle_contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<int> contour_index, circle_contours_index;
cv::Mat src, /*croppedRef, cropped,*/ hsv, gray, dst;
cv::Scalar up_lim, low_lim, up_lim_wrap, low_lim_wrap;
cv::Scalar low_black, up_black, low_white, up_white;
cv::Mat lower_hue_range, upper_hue_range;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
cv::Rect rect;
cv::RotatedRect mr;
int height, width;
int min_area = 300;
float min_height_ratio = 0.08;
double area, r_area, mr_area;
float circle_accuracy = 0.035; // Actually it is circle max error, will be adjusted in dynamic_configCb()
const double eps = 0.15;
const float circle_to_checkbox_ratio = 0.2;

//Functions
void reduce_noise(cv::Mat* dst)
{
  cv::morphologyEx(*dst, *dst, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*dst, *dst, cv::MORPH_OPEN, str_el);
}

cv::Mat doAdaptiveThreshold()
{
  // Extract the S-channel of HSV to threshold
  std::vector<cv::Mat> input_channels;
  cv::split(hsv, input_channels);
  cv::Mat input = input_channels[1];

  int ksize = 7; // Must be odd and positive
  cv::GaussianBlur(input, input, cv::Size(ksize, ksize), 0);

  cv::Mat output;
  int adaptiveMethod = CV_ADAPTIVE_THRESH_MEAN_C;
  int thresholdType = CV_THRESH_BINARY;
  int blockSize = 61; // only values of 3, 5, 7, ...
  double C = 3;
  cv::adaptiveThreshold(input, output, 255, adaptiveMethod, thresholdType, blockSize, C);
  return output;
}

void armor_found(int a, int b, float LED_length)
{
  cv::Rect recta = cv::boundingRect(contours[a]);
  cv::Rect rectb = cv::boundingRect(contours[b]);
  sensor_msgs::RegionOfInterest obj;
  if(recta.tl().x > rectb.tl().x)
  {
    cv::Rect temp = recta;
    recta = rectb;
    rectb = temp;
  }
  float x_offset = (recta.tl().x);
    obj.x_offset = x_offset;
  float y_offset = ((recta.tl().y + rectb.tl().y)*0.5 - LED_length*0.5 /*+ height/4*/); //note that the cropping makes change to the coordinate, which explains the height/4 component
    obj.y_offset = y_offset;
  float armor_height = 2*LED_length;
    obj.height = armor_height;
  float armor_width = fabs(recta.tl().x - rectb.br().x);
    obj.width = armor_width;
    obj.do_rectify = true;
  object.push_back(obj);  //Push the object to the vector
  if(debug) cv::rectangle(src, cv::Point(x_offset, y_offset), cv::Point(x_offset + armor_width, y_offset + armor_height), cv::Scalar(0,255,0), 2, 8, 0);
  return;
}

void detect_armor_mode_0()
{
  //*********************************
  //Filter desired color (color = indicated in the launch file)
  if(armor_color == "red")
  {//In case of red color
    cv::inRange(hsv, low_lim, up_lim, lower_hue_range);
    cv::inRange(hsv, low_lim_wrap, up_lim_wrap, upper_hue_range);
    cv::addWeighted(lower_hue_range,1.0,upper_hue_range,1.0,0.0,dst);
  }
  else cv::inRange(hsv, low_lim, up_lim, dst);
  //Reduce noise
  reduce_noise(&dst);
  if(debug) cv::imshow("LEDs", dst);
  //Finding shapes
  cv::findContours(dst.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour: is it LED?
  for(int i = 0; i < contours.size(); i++)
  {
    rect = cv::boundingRect(contours[i]);
    area = cv::contourArea(contours[i]);
    //Delete the contours that are too small or not seem to be the LED on the armor
    if(rect.height < height*min_height_ratio)
    {
      if(debug) cv::drawContours(dst, contours, i, Scalar(0,0,0),-1);
      continue;
    }
    if(rect.height < rect.width*2)
    {
      if(debug) cv::drawContours(dst, contours, i, Scalar(0,0,0),-1);
      continue;
    }
    mr = cv::minAreaRect(contours[i]);
    mr_area = (mr.size).height*(mr.size).width;
    if(area/mr_area < 0.5)
    {
      if(debug) cv::drawContours(dst, contours, i, Scalar(0,0,0),-1);
      continue;
    }
    //If contour is good, save the contour for further processing
    if(debug) cv::drawContours(src, contours, i, cv::Scalar(255,0,255), 2);
    contour_index.push_back(i);
  }
  // if(contour_index.empty()) return;
  //*********************************
  //Filter the numbering circle (color = black)
  cv::inRange(hsv, low_black, up_black, dst);
  //Reduce noise
  // reduce_noise(&dst);
  //Finding shapes
  cv::findContours(dst.clone(), circle_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour, is it a circle?
  for(int i = 0; i < circle_contours.size(); i++)
  {
    //Skip small objects
    area = cv::contourArea(circle_contours[i]);

    rect = cv::boundingRect(circle_contours[i]);
    if(rect.height < height*min_height_ratio) continue;
    
    mr = cv::minAreaRect(circle_contours[i]);
    mr_area = (mr.size).height*(mr.size).width;

    vector<Point> hull;
    convexHull(circle_contours[i], hull, 0, 1);
    double hull_area = contourArea(hull);

    if((std::fabs(area/mr_area - 3.141593/4) < circle_accuracy) && (std::fabs(area/hull_area - 1) < circle_accuracy) 
        && (std::fabs((float)rect.height/rect.width - 1) < 0.35))
    {//Circle found
      // cv::Point object_center = (rect.tl() + rect.br() + cv::Point(1,1))*0.5;
      if(debug) cv::drawContours(src, circle_contours, i, cv::Scalar(0,255,255), 2);
      //Save the circle for further processing
      circle_contours_index.push_back(i);
    }
  }
  if(debug) cv::imshow("black", dst);
  if(contour_index.empty() || circle_contours_index.empty()) return;

  //*********************************
  //Final process: merging sets of LEDs and circles
  for(int i = 0; i < contour_index.size()-1; i++)
    for(int j = i+1; j < contour_index.size(); j++)
    {
      if(i==j) continue;
      cv::RotatedRect rect1 = cv::minAreaRect(contours[contour_index[i]]);
      cv::RotatedRect rect2 = cv::minAreaRect(contours[contour_index[j]]);

      //Check angles of 2 LEDs
      //***Note: since there is a circle between the LEDs as an indicator, angle checking is no longer really needed
      //if(fabs(rect1.angle - rect2.angle) > 5.0) continue;

      //Check sizes of 2 LEDs
      if(fabs(max(rect1.size.height, rect1.size.width)/max(rect2.size.height, rect2.size.width) - 1.0) > 0.2) continue;

      //Check distance between 2 LEDs
      float LED_length = max(rect1.size.height, rect1.size.width);
      if((fabs(rect1.center.y - rect2.center.y) > LED_length/2.0)||(fabs(rect1.center.x - rect2.center.x) > 5.0*LED_length)) continue;

      // Check if there is a circle between the LEDs
      bool circle_check = false;
      for(int k = 0; k < circle_contours_index.size(); k++)
        if(cv::pointPolygonTest(circle_contours[circle_contours_index[k]], (rect1.center + rect2.center)*0.5, false) > 0)
        {
          circle_check = true;
          circle_contours_index.erase(circle_contours_index.begin()+k);
          break;
        }
      if(circle_check == false) continue;

      // Finished checking, armor confirmed
      armor_found(contour_index[i], contour_index[j], LED_length);
      // cv::line(dst, contour_rrect[i].center, contour_rrect[j].center, Scalar(255,255,255),20);
      if(contour_index.size() <= 2)
      {
        contour_index.clear();
        return;
      }
      // contour_index.erase(contour_index.begin()+j);
      // contour_index.erase(contour_index.begin()+i);
      // // if(contour_index.empty()) return;
      // j=j-2;
    }
  contour_index.clear();
  circle_contours_index.clear();
  return;
}

void detect_armor_mode_1()
{
  //Filter the numbering circle (color = black)
  if(armor_color == "blue") 
  {
    cv::inRange(hsv, low_black, up_black, dst);
    // cv::imshow("inrange", dst);
    // cv::imshow("adt", doAdaptiveThreshold());
    cv::bitwise_or(doAdaptiveThreshold(), dst, dst);
  } 
  else cv::inRange(hsv, low_black, up_black, dst);
  //Finding shapes
  cv::findContours(dst.clone(), circle_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for(int i = 0; i < circle_contours.size(); i++)
  {
    //Skip small objects
    area = cv::contourArea(circle_contours[i]);

    rect = cv::boundingRect(circle_contours[i]);
    if(rect.height < height*min_height_ratio) continue;
    
    mr = cv::minAreaRect(circle_contours[i]);
    mr_area = (mr.size).height*(mr.size).width;

    vector<Point> hull;
    convexHull(circle_contours[i], hull, 0, 1);
    double hull_area = contourArea(hull);

    if((std::fabs(area/mr_area - 3.141593/4) < circle_accuracy) && (std::fabs(area/hull_area - 1) < circle_accuracy) 
        && (std::fabs((float)rect.height/rect.width - 1) < 0.4))
    { //Circle found
      cv::Point object_center = (rect.tl() + rect.br() + cv::Point(1,1))*0.5;
      if(debug) cv::drawContours(src, circle_contours, i, cv::Scalar(0,255,255), 2);

      cv::Point armor_top_left = rect.tl() - cv::Point(rect.width*circle_to_checkbox_ratio, rect.height*circle_to_checkbox_ratio);
      cv::Point armor_bot_right = rect.br() + cv::Point(1,1) + cv::Point(rect.width*circle_to_checkbox_ratio, rect.height*circle_to_checkbox_ratio);
      cv::Rect armor_roi(armor_top_left, armor_bot_right);

      //Abort if the roi is bigger than the image frame itself
      // if(!((armor_roi & cv::Rect(0, 0, width, height)) == armor_roi)) continue;

      // Fix the roi to fit the image frame
      armor_roi = armor_roi & cv::Rect(0, 0, width, height);

      if(debug) cv::rectangle(src, armor_top_left, armor_bot_right, cv::Scalar(0,0,255), 2, 8, 0);  //Uncomment to see the region tested for black background
      int no_positive_pixels = cv::countNonZero(dst(armor_roi));
      if((float)no_positive_pixels/(armor_roi.height*armor_roi.width - area) > 0.95)
      { //Armor found
        sensor_msgs::RegionOfInterest obj;
        cv::Point offset = armor_top_left;
        // if(offset.x < 0) offset.x = 0;
        // else if(offset.x > width) offset.x = width;
        // if(offset.y < 0) offset.y = 0;
        // else if(offset.y > height) offset.y = height;
        obj.x_offset = offset.x;
        obj.y_offset = offset.y;
        obj.height = armor_bot_right.y - armor_top_left.y;
        obj.width = armor_bot_right.x - armor_top_left.x;
        obj.do_rectify = true;

        object.push_back(obj);  //Push the object to the vector
        if(debug) cv::rectangle(src, offset, offset + cv::Point(obj.width, obj.height), cv::Scalar(0,255,0), 2, 8, 0);
      }
    }
  }
  if(debug) 
  {
    cv::imshow("black", dst);
  }
  return;
}

void detect_armor_mode_2()
{
  cv::Mat black, white;
  cv::inRange(gray, low_black, up_black, black);
  cv::inRange(gray, low_white, up_white, white);
  //Finding shapes
  cv::findContours(white.clone(), circle_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for(int i = 0; i < circle_contours.size(); i++)
  {
    //Skip small objects
    area = cv::contourArea(circle_contours[i]);

    rect = cv::boundingRect(circle_contours[i]);
    if(rect.height < height*min_height_ratio) continue;
    
    mr = cv::minAreaRect(circle_contours[i]);
    mr_area = (mr.size).height*(mr.size).width;

    vector<Point> hull;
    convexHull(circle_contours[i], hull, 0, 1);
    double hull_area = contourArea(hull);

    if((std::fabs(area/mr_area - 3.141593/4) < circle_accuracy) && (std::fabs(area/hull_area - 1) < circle_accuracy) 
        && (std::fabs((float)rect.height/rect.width - 1) < 0.4))
    { //Circle found
      cv::Point object_center = (rect.tl() + rect.br() + cv::Point(1,1))*0.5;
      if(debug) cv::drawContours(src, circle_contours, i, cv::Scalar(0,255,255), 2);
      cv::Point armor_top_left = rect.tl() - cv::Point(rect.width*circle_to_checkbox_ratio, rect.height*circle_to_checkbox_ratio);
      cv::Point armor_bot_right = rect.br() + cv::Point(1,1) + cv::Point(rect.width*circle_to_checkbox_ratio, rect.height*circle_to_checkbox_ratio);
      cv::Rect armor_roi(armor_top_left, armor_bot_right);

      //Abort if the roi is bigger than the image frame itself
      // if(!((armor_roi & cv::Rect(0, 0, width, height)) == armor_roi)) continue;

      // Fix the roi to fit the image frame
      armor_roi = armor_roi & cv::Rect(0, 0, width, height);

      if(debug) cv::rectangle(src, armor_top_left, armor_bot_right, cv::Scalar(0,0,255), 2, 8, 0);  //Uncomment to see the region tested for black background
      int no_positive_pixels = cv::countNonZero(black(armor_roi));
      if((float)no_positive_pixels/(armor_roi.height*armor_roi.width - area) > 0.95)
      { //Armor found
        sensor_msgs::RegionOfInterest obj;
        cv::Point offset = armor_top_left;
        // if(offset.x < 0) offset.x = 0;
        // else if(offset.x > width) offset.x = width;
        // if(offset.y < 0) offset.y = 0;
        // else if(offset.y > height) offset.y = height;
        obj.x_offset = offset.x;
        obj.y_offset = offset.y;
        obj.height = armor_bot_right.y - armor_top_left.y;
        obj.width = armor_bot_right.x - armor_top_left.x;
        obj.do_rectify = true;

        object.push_back(obj);  //Push the object to the vector
        if(debug) cv::rectangle(src, offset, offset + cv::Point(obj.width, obj.height), cv::Scalar(0,255,0), 2, 8, 0);
      }
    }
  }
  if(debug) 
  {
    cv::imshow("black", black);
    cv::imshow("white", white);
  }
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
  cv::blur(src,src,Size(1,1));
  if(!detection_mode) //If mode 0
  {
    cv::cvtColor(src, hsv, COLOR_BGR2HSV);
    detect_armor_mode_0();
  }
  else if(detection_mode == 1)
  {
    cv::cvtColor(src, hsv, COLOR_BGR2HSV);
    detect_armor_mode_1(); 
  }
  else // Else mode 2
  {
    cv::cvtColor(src, gray, COLOR_BGR2GRAY);
    detect_armor_mode_2();
  }
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
  if(detection_mode != 2) //If mode 0 or 1
  {
    if(armor_color == "blue") 
    {
      low_lim = cv::Scalar(config.blue_H_low,config.blue_S_low,config.blue_V_low);
      up_lim = cv::Scalar(config.blue_H_high,config.blue_S_high,config.blue_V_high);

      low_black = cv::Scalar(config.black_H_low_b, config.black_S_low_b, config.black_V_low_b);
      up_black = cv::Scalar(config.black_H_high_b, config.black_S_high_b, config.black_V_high_b);

      circle_accuracy = 0.08;
    }
    else if(armor_color == "red")
    {
      low_lim = cv::Scalar(config.red_H_low1, config.red_S_low, config.red_V_low);
      up_lim = cv::Scalar(config.red_H_high1, config.red_S_high, config.red_V_high);
      low_lim_wrap = cv::Scalar(config.red_H_low2, config.red_S_low, config.red_V_low);
      up_lim_wrap = cv::Scalar(config.red_H_high2, config.red_S_high, config.red_V_high);

      low_black = cv::Scalar(config.black_H_low_r, config.black_S_low_r, config.black_V_low_r);
      up_black = cv::Scalar(config.black_H_high_r, config.black_S_high_r, config.black_V_high_r);

      circle_accuracy = 0.035;
    }
  }
  else //If mode 2
  {
    low_black = cv::Scalar(config.black_low_mode2);
    up_black = cv::Scalar(config.black_high_mode2);
    low_white = cv::Scalar(config.white_low_mode2);
    up_white = cv::Scalar(config.white_high_mode2);
  }
  ROS_INFO("Reconfigure Requested.");
}

void colorCb(const std_msgs::Bool clr)
{
  if (clr.data) {
    armor_color = "blue";
    ros::param::set("/armor_detection/armor_color", armor_color);
  }
  else {
    armor_color = "red";
    ros::param::set("/armor_detection/armor_color", armor_color);
  }
  pubd = true;
  csub.shutdown();
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "armor_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  // pnh.getParam("armor_color", armor_color); // for test here
  // emily changing this to setting armor_color after subscribing to the topic /color on arduino
  ros::param::set("armor_detection/start", 1);
  csub = nh.subscribe("/color", 10, colorCb);
  while (!pubd) ros::spinOnce();
  pnh.getParam("mode", detection_mode);
  pnh.getParam("debug", debug);
  pnh.getParam("published_topic", published_topic);
  dynamic_reconfigure::Server<base_vision::armor_colorConfig> server;
  dynamic_reconfigure::Server<base_vision::armor_colorConfig>::CallbackType f;
  f = boost::bind(&dynamic_configCb, _1, _2);
  server.setCallback(f);
  ROS_INFO("Operating mode: %d", detection_mode);
  
  //Initiate windows
  if(debug)
  {
    // Resize the output window, smaller, so that they fit my screen
    int org_w = 1024, org_h = 576; // (1024, 576) as original
    int win_w = org_w*0.75, win_h = org_h*0.75; // as value: (768, 432)

    cv::namedWindow("src", WINDOW_NORMAL);
    cv::resizeWindow("src",win_w, win_h); 
    cv::moveWindow("src", 0, 0);
    cv::namedWindow("black", WINDOW_NORMAL);
    cv::resizeWindow("black",win_w, win_h);
    cv::moveWindow("black", 0, 600);
    if(!detection_mode)
    {
      cv::namedWindow("LEDs", WINDOW_NORMAL);
      cv::resizeWindow("LEDs",win_w, win_h);
      cv::moveWindow("LEDs", 800, 0);
    }
    else if(detection_mode == 2)
    {
      cv::namedWindow("white", WINDOW_NORMAL);
      cv::resizeWindow("white",win_w, win_h);
      cv::moveWindow("white", 800, 0); 
    }
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
