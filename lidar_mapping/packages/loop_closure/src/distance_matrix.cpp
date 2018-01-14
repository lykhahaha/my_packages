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

Eigen::Matrix<double, 192, Eigen::Dynamic> all_descriptors;
Eigen::MatrixXd distance_matrix;
int scan_num = 0;

double map2grayscale(double x)
{
  x *= 150;
  return (x > 255 ? 255.0 : x);
}

void m2dpCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
{
  if(scan_num >= 1000)
  {
    std::cout << "scan_num >= 1000, exitting..." << std::endl;
    return;
  }

  std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
  // Get input data
  pcl::PointCloud<pcl::PointXYZI> input_cloud;
  pcl::fromROSMsg(*input_msg, input_cloud);

  // Filter
  pcl::PointCloud<pcl::PointXYZI> processed_cloud;
  for(auto itr = input_cloud.begin(), itr_end = input_cloud.end(); itr != itr_end; itr++)
  {
    double rho = itr->x * itr->x + itr->y * itr->y;
    if(rho > 3.0 && rho < 120.0)
    {
      processed_cloud.push_back(*itr);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(processed_cloud));
  // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  // voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
  // voxel_grid_filter.setInputCloud(processed_cloud_ptr);
  // voxel_grid_filter.filter(*processed_cloud_ptr);

  // Get descriptor
  Eigen::VectorXd m2dp;
  if(!getM2DP(*processed_cloud_ptr, m2dp))
  {
    std::cout << "ERROR: could not get M2DP descriptor!" << std::endl;
    return;
  }
  m2dp = m2dp.array().abs();
  // std::cout << "Descriptors: (" << all_descriptors.rows() << ", " << all_descriptors.cols() << ")\n";
  // std::cout << "M2DP: (" << m2dp.rows() << ", " << m2dp.cols() << ")" << std::endl;

  if(scan_num != 0)
  {
    // Calculate distance to all prev descriptors, shape: (1, scan_num)
    Eigen::MatrixXd L2_distances = (all_descriptors.colwise() - m2dp).array().square().colwise().sum();

    // Map the distances to [0, 255], grayscale 8bit
    L2_distances = L2_distances.unaryExpr(&map2grayscale);
    // std::cout << L2_distances << std::endl;

    // Add to distance matrix
    distance_matrix.conservativeResize(distance_matrix.rows() + 1, distance_matrix.cols() + 1);
    distance_matrix.block(0, scan_num, scan_num, 1) = L2_distances.transpose();
    distance_matrix.block(scan_num, 0, 1, scan_num) = L2_distances;
    distance_matrix(scan_num, scan_num) = 0;
  }
  else
  {
    distance_matrix.conservativeResize(1, 1);
    distance_matrix(0, 0) = 0;
  }

  // Convert to opencv matrix
  cv::Mat cv_image(scan_num + 1, scan_num + 1, CV_8UC1);
  cv::eigen2cv(distance_matrix, cv_image);
  cv_image.convertTo(cv_image, CV_8UC1);

  // Display
  cv::imshow("m2dp distance matrix", cv_image);
  std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
  std::cout << "Callback took: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << "ms" << std::endl;

  // Update
  all_descriptors.conservativeResize(Eigen::NoChange, all_descriptors.cols() + 1);
  all_descriptors.col(scan_num) = m2dp;
  scan_num++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descriptor_visualizer");
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub = nh.subscribe("/velodyne_points", 1000, m2dpCallback);

  // Initialize
  cv::namedWindow("m2dp distance matrix", cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();

  // Run
  ros::spin();
}