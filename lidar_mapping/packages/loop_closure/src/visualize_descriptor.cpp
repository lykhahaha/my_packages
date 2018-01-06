#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include "M2DP.h"

void m2dpCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
{
  std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
  // Get input data
  pcl::PointCloud<pcl::PointXYZI> input_cloud;
  pcl::fromROSMsg(*input_msg, input_cloud);

  // Filter
  pcl::PointCloud<pcl::PointXYZI> processed_cloud;
  for(auto itr = input_cloud.begin(), itr_end = input_cloud.end(); itr != itr_end; itr++)
  {
    double rho = itr->x * itr->x + itr->y * itr->y;
    if(rho > 3.0 && rho < 30.0)
    {
      processed_cloud.push_back(*itr);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(processed_cloud));
   pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
   voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
   voxel_grid_filter.setInputCloud(processed_cloud_ptr);
   voxel_grid_filter.filter(*processed_cloud_ptr);
  // Get descriptor
  Eigen::MatrixXd m2dp;
  if(!getM2DP(*processed_cloud_ptr, m2dp))
  {
    std::cout << "ERROR: could not get M2DP descriptor!" << std::endl;
    return;
  }

  // Convert to opencv matrix
  cv::Mat m2dp_cv;
  cv::eigen2cv(m2dp, m2dp_cv);

  // Draw
  int image_size = 192; // 4*16+16*8
  cv::Mat visual_image(image_size * 4, image_size, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::line(visual_image, cv::Point(0, image_size*2), cv::Point(image_size, image_size*2), cv::Scalar(255, 255, 255));

  m2dp_cv = m2dp_cv * 500;
  for(int i = 1; i < image_size; i++)
  {
    cv::line(visual_image,
             cv::Point(i - 1, image_size * 2 - std::round(m2dp_cv.at<double>(i - 1))),
             cv::Point(i, image_size * 2 - std::round(m2dp_cv.at<double>(i))),
             cv::Scalar(255, 255, 0), 1, 8, 0);
  }

  // Display
  cv::imshow("m2dp", visual_image);
  std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
  std::cout << "Callback took: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << "ms" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descriptor_visualizer");
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub = nh.subscribe("/points_raw", 1000, m2dpCallback);

  // Display
  cv::namedWindow("m2dp", cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();
  ros::spin();
}