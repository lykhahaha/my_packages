// Basic libs
// #include <chrono>
// #include <fstream>
#include <iostream>
// #include <mutex>
// #include <omp.h>
// #include <sstream>
#include <string>
// #include <unordered_map>
#include <vector>
#include <cstdlib>
// #include <unistd.h>
// #include <sys/types.h>
// #include <pwd.h>
#include <time.h>

// #include <boost/foreach.hpp> // to read bag file
// #define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
// #include <ros/time.h>
// #include <ros/duration.h>
// #include <signal.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
// #include <visualization_msgs/Marker.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>

// // PCL & 3rd party libs
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

int findRoadSurfaceNormalVector(pcl::PointCloud<pcl::PointXYZI> cloud,
                                std::vector<uint32_t>& inlier_indices,
                                const uint32_t MIN_NUM_INLIERS = 5000,
                                const double DIST_THRESHOLD = 0.2,
                                const uint32_t MAX_ITERATION = 50)
{
  double optimal_cost = 99999999.0;
  srand(time(NULL));
  for(uint32_t iteration = 1; iteration <= MAX_ITERATION; iteration++)
  {
    // Get 3 random points
    unsigned int idx[3];
    idx[0] = rand() % cloud.size();
    idx[1] = rand() % cloud.size();
    while(idx[1] == idx[0]) 
      idx[1] = rand() % cloud.size(); // re-roll until idx[1] != idx[0]
    idx[2] = rand() % cloud.size();
    while(idx[2] == idx[0] || idx[2] == idx[1])
      idx[2] = rand() % cloud.size(); // re-roll

    // Calculate normal vector w.r.t plane
    Eigen::Vector3d A(cloud.points[idx[0]].x, cloud.points[idx[0]].y, cloud.points[idx[0]].z);
    Eigen::Vector3d B(cloud.points[idx[1]].x, cloud.points[idx[1]].y, cloud.points[idx[1]].z);
    Eigen::Vector3d C(cloud.points[idx[2]].x, cloud.points[idx[2]].y, cloud.points[idx[2]].z);
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AC = C - A;
    Eigen::Vector3d n = (AB.cross(AC)).normalized();
    std::cout << "n >> " << std::endl;
    std::cout << n << std::endl;

    // Iterate through all points
    double current_cost = 0.0;
    std::vector<uint32_t> currrent_inlier_indices;
    for(uint32_t i = 0, i_end = cloud.size(); i < i_end; i++)
    {
      Eigen::Vector3d P(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      double point2plane_dist = std::fabs(n.dot(P - A));
      if(point2plane_dist < DIST_THRESHOLD)
      { 
        currrent_inlier_indices.push_back(i);
        current_cost += point2plane_dist;
      }
    }
    std::cout << "Cost: " << current_cost << std::endl;
    if(currrent_inlier_indices.size() >= MIN_NUM_INLIERS
      && current_cost/currrent_inlier_indices.size() < optimal_cost)
    {
      optimal_cost = current_cost/currrent_inlier_indices.size();
      inlier_indices = currrent_inlier_indices;
    }
  }

  if(inlier_indices.size() < 0)
  {
    std::cout << "RANSAC FAILED!" << std::endl;
    return 0;
  }
  std::cout << "RANSAC SUCCEEDED!" << std::endl;
  return 1;
}

int main(int argc, char** argv)
{
  // ROS initialization
  ros::init(argc, argv, "road_surface");
  ros::NodeHandle nh;
  ros::Publisher src_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 1, true);
  ros::Publisher dst_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_scan", 1, true);

  if(argc < 2)
  {
    std::cout << "Please indicate a pcd file." << std::endl;
    return -1;
  }

  // Get PCD
  std::string filename = argv[1];
  pcl::PointCloud<pcl::PointXYZI> src;
  std::cout << "Processing " << filename << "... " << std::flush;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(filename, src) == -1)
  {
    std::cout << "Couldn't read " << filename << "." << std::endl;
    return(-1);
  }
  else std::cout << "Done." << std::endl;

  // Start processing
  // Normally, z = -2.5 is the road surface, filtering out z > -1 or z < -4
  std::cout << "Filtering points near low ground... " << std::flush;
  pcl::PointCloud<pcl::PointXYZI> tmp = src;
  // for(int i = 0, i_end = src.size(); i < i_end; i++)
  // {
  //  // if(fabs(src.points[i].z + 2.5) < 1.5)
  //    tmp.push_back(src.points[i]);
  // }
  std::cout << "Done." << std::endl;

  // RANSAC
  // const uint32_t MIN_NUM_INLIERS = 10000; // points needed to consider good
  // const double DIST_THRESHOLD = 0.05;
  // const uint32_t MAX_ITERATION = 50;
  std::vector<uint32_t> inlier_indices;
  int ransac_success = findRoadSurfaceNormalVector(tmp, inlier_indices);
  
  // Optimize found plane, ref: https://gist.github.com/ialhashim/0a2554076a6cf32831ca
  // Copy coordinates to matrix in Eigen format
  Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, inlier_indices.size());
  pcl::PointCloud<pcl::PointXYZI> dst;
  for(uint32_t i = 0, i_end = inlier_indices.size(); i < i_end; ++i)
  {
    coord.col(i) = Eigen::Vector3d(tmp.points[inlier_indices[i]].x, tmp.points[inlier_indices[i]].y, tmp.points[inlier_indices[i]].z);
    dst.push_back(tmp.points[inlier_indices[i]]); // also push to pointcloud to visualize later
  }

  // Calculate centroid
  Eigen::Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean()); // point-in-plane

  // Subtract centroid
  coord.row(0).array() -= centroid(0);
  coord.row(1).array() -= centroid(1);
  coord.row(2).array() -= centroid(2);

  // we only need the left-singular matrix here
  // http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector3d plane_normal = svd.matrixU().rightCols<1>(); // normal-vector-of-plane
  std::cout << "Result:\n";
  std::cout << "\tn = \n" << plane_normal << "\n";
  std::cout << "\tPlane size: " << inlier_indices.size() << std::endl;

  // Convert to ROS msgs and publish
  std::cout << "Finished processing. Publishing to /current_scan and /filtered_scan." << std::endl;

  sensor_msgs::PointCloud2::Ptr cloud_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(src, *cloud_msg_ptr);
  cloud_msg_ptr->header.frame_id = "map";
  src_pub.publish(*cloud_msg_ptr);

  pcl::toROSMsg(dst, *cloud_msg_ptr);
  cloud_msg_ptr->header.frame_id = "map";
  dst_pub.publish(*cloud_msg_ptr);

  ros::spin();
}