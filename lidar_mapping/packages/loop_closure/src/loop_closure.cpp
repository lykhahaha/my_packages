#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
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

ros::Publisher local_cloud_pub;
tf::TransformListener* tf_listener;
pcl::PointCloud<pcl::PointXYZI> local_cloud;
Eigen::Affine3d local_tf;

Eigen::Matrix<double, 192, Eigen::Dynamic> all_descriptors;
Eigen::MatrixXd distance_matrix;
int scan_num = 0;
int loop_location[2000][2];
int loop_num = 0;
int local_cloud_num = 0;

double map2grayscale(double x)
{
  x *= 200;
  return (x > 255 ? 255.0 : x);
}

void loopClosureCb(const sensor_msgs::PointCloud2::ConstPtr& input_msg)
{
  if(scan_num >= 2000)
  {
    std::cout << "scan_num >= 2000, exitting..." << std::endl;
    return;
  }

  std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();

  pcl::PointCloud<pcl::PointXYZI> input_cloud;
  pcl::fromROSMsg(*input_msg, input_cloud);

  // Filter
  pcl::PointCloud<pcl::PointXYZI> processed_cloud;
  for(auto itr = input_cloud.begin(), itr_end = input_cloud.end(); itr != itr_end; itr++)
  {
    double rho = itr->x * itr->x + itr->y * itr->y;
    if(rho > 3.2 && rho < 120.0)
    {
      processed_cloud.push_back(*itr);
    }
  }

  tf::StampedTransform tf_transform;
  try
  {
    tf_listener->lookupTransform("world", "velodyne", input_msg->header.stamp, tf_transform);
  }
  catch(tf::ExtrapolationException &e)
  {
    std::cout << "Exception throws!" << std::endl;
    return;
  }
  tf::Vector3 tf_trl = tf_transform.getOrigin();
  tf::Matrix3x3 tf_rot = tf_transform.getBasis();
  double x = tf_trl.x();
  double y = tf_trl.y();
  double z = tf_trl.z();
  double roll, pitch, yaw;
  tf_rot.getRPY(roll, pitch, yaw, 1);

  Eigen::Affine3d current_tf;
  pcl::getTransformation(x, y, z, roll, pitch, yaw, current_tf);

  if(local_cloud_num == 0)
  {
    local_tf = current_tf;
    local_cloud += processed_cloud;
    local_cloud_num++;
  }
  else
  {
    Eigen::Affine3d relative_tf = local_tf.inverse() * current_tf;  
    pcl::transformPointCloud(processed_cloud, processed_cloud, relative_tf);
    local_cloud += processed_cloud;
    local_cloud_num++;

    if(local_cloud_num == 5)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(local_cloud));
      pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
      voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
      voxel_grid_filter.setInputCloud(local_cloud_ptr);
      voxel_grid_filter.filter(*local_cloud_ptr);

      // Get descriptor
      Eigen::VectorXd m2dp;
      if(!getM2DP(*local_cloud_ptr, m2dp))
      {
        std::cout << "ERROR: could not get M2DP descriptor!" << std::endl;
        return;
      }

      m2dp = m2dp.array().abs();

      cv::Mat cv_image(scan_num + 1, scan_num + 1, CV_8UC1);
      cv::Mat bgr_image;
      if(scan_num != 0)
      {
        // Calculate distance to all prev descriptors, shape: (1, scan_num)
        Eigen::MatrixXd L2_distances = (all_descriptors.colwise() - m2dp).array().square().colwise().sum();

        // Map the distances to [0, 255], grayscale 8bit
        L2_distances = L2_distances.unaryExpr(&map2grayscale);
        Eigen::MatrixXd::Index min_row, min_col;
        double min_l2 = L2_distances.minCoeff(&min_row, &min_col); // min_col = 1 in all cases

        // Add to distance matrix
        distance_matrix.conservativeResize(distance_matrix.rows() + 1, distance_matrix.cols() + 1);
        distance_matrix.block(0, scan_num, scan_num, 1) = L2_distances.transpose();
        distance_matrix.block(scan_num, 0, 1, scan_num) = L2_distances;
        distance_matrix(scan_num, scan_num) = 0;

        cv::eigen2cv(distance_matrix, cv_image);
        cv_image.convertTo(cv_image, CV_8UC1);
        cv::cvtColor(cv_image, bgr_image, CV_GRAY2BGR);
        if(min_l2 < 1 && (scan_num - (int)min_col > 30))
        {
          loop_location[loop_num][0] = scan_num;
          loop_location[loop_num][1] = (int)min_col;
          loop_num++;
          printf("Loop closure: %.3f at (%d, %d)\n", min_l2, int(min_row), int(min_col));
        }

        for(int i = 0; i < loop_num; i++)
        {
          // bgr_image.at<cv::Vec3b>(loop_location[i][0], loop_location[i][1]) = cv::Vec3b(0, 255, 0); 
          // bgr_image.at<cv::Vec3b>(loop_location[i][1], loop_location[i][0]) = cv::Vec3b(0, 255, 0);

          cv::circle(bgr_image, cv::Point(loop_location[i][0], loop_location[i][1]), 2, cv::Scalar(0, 255, 0), -1);
          cv::circle(bgr_image, cv::Point(loop_location[i][1], loop_location[i][0]), 2, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("m2dp distance matrix", bgr_image);
      }
      else // first frame
      {
        distance_matrix.conservativeResize(1, 1);
        distance_matrix(0, 0) = 0;
        cv::eigen2cv(distance_matrix, cv_image);
        cv::imshow("m2dp distance matrix", cv_image);
      }

      // Publish and update
      sensor_msgs::PointCloud2 local_msg;
      pcl::toROSMsg(*local_cloud_ptr, local_msg);
      local_msg.header.frame_id = "map";
      local_cloud_pub.publish(local_msg);

      cv::waitKey(5);
      all_descriptors.conservativeResize(Eigen::NoChange, all_descriptors.cols() + 1);
      all_descriptors.col(scan_num) = m2dp;
      scan_num++;
      local_cloud_num = 0;
    }
  }
  std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
  std::cout << "Callback took " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << "ms" << std::endl;
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loop_closure");
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub = nh.subscribe("/velodyne_points", 1000, loopClosureCb);
  local_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_cloud", 1000, true);
  tf_listener = new tf::TransformListener();
    
  // Initialize
  cv::namedWindow("m2dp distance matrix", cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();
  cv::waitKey(5);

  // Run
  ros::spin();
}