/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 Localization and mapping program using Normal Distributions Transform
 Yuki KITSUKAWA
 */

#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <omp.h>
#include <math.h>

#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

// Here are the functions I wrote. De-comment to use
#define MY_SPLIT_PCD // split pcd into many small pcd files for faster processing time
#define MY_OFFSET_MAPPING // do not use this, this is to extract scans and poses to close loop

static int k = 0; // key-scan count
#ifdef MY_OFFSET_MAPPING
std::ofstream tf_map_csv_file;
std::string tf_map_csv_file_name = "/home/zwu/offset-mapping-icp/tf_map.csv";
#endif // MY_OFFSET_MAPPING

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

// global variables
static pose previous_pose, guess_pose, current_pose, icp_pose, added_pose, localizer_pose;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw; // current_pose - previous_pose

static double current_velocity_x = 0.0;
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

static pcl::PointCloud<pcl::PointXYZI> map;

//static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
static pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
// Default values for ICP
static int maximum_iterations = 100;
static double transformation_epsilon = 0.0001;
static double max_correspondence_distance = 1.0;
static double euclidean_fitness_epsilon = 0.01;
static double ransac_outlier_rejection_threshold = 1.0;

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 1.0;

// static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
//     t5_start, t5_end;
// static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher icp_map_pub;
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 3.0;
static double min_add_scan_shift = 1.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool isMapUpdate = true;
static double fitness_score;

static double wrapToPm(double a_num, const double a_max)
{
    if (a_num >= a_max)
        a_num -= 2.0 * a_max;
    return a_num;
}

static double wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

// output function : output map as a pcd small files
#ifdef MY_SPLIT_PCD
static void output(pcl::PointCloud<pcl::PointXYZI> submap)
{
  static double name = 0;
  std::ostringstream interFilename;
  interFilename << "/home/zwu/mapping-icp/icpmap-" << name << ".pcd";
  std::string filename = interFilename.str();
  name++;
  
  // double filter_res = 0.25;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(submap));
  // pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  map_ptr->header.frame_id = "map";
  // map_filtered->header.frame_id = "map";
  
  // Apply voxelgrid filter
  // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  // voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
  // voxel_grid_filter.setInputCloud(map_ptr);
  // voxel_grid_filter.filter(*map_filtered);
  
  // Writing Point Cloud data to PCD file
  pcl::io::savePCDFileBinary(filename, *map_ptr);
}
#endif

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (r > min_scan_range)
      scan.push_back(p);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if(initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
#ifdef MY_OFFSET_MAPPING
    pcl::io::savePCDFileBinary("/home/zwu/offset-mapping-icp/scan0.pcd", *scan_ptr);

    // outputing into csv
    tf_map_csv_file << "/home/zwu/offset-mapping-icp/scan0.pcd" << "," 
                    << _tf_x << "," << _tf_y << "," << _tf_z << "," 
                    << _tf_roll << "," << _tf_pitch << "," << _tf_yaw
                    << std::endl;
    return;
#endif // MY_OFFSET_MAPPING
  }

  // Apply voxelgrid filter
  if(voxel_leaf_size == 0.0)
    *filtered_scan_ptr = *scan_ptr;
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
  }
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  if(isMapUpdate == true)
  {
    icp.setInputTarget(map_ptr);
    isMapUpdate = false;
  }

  double icp_score[11];
  Eigen::Matrix4f icp_tf[11];

#pragma omp parallel for 
  for(int dt = 0; dt <= 10; dt++) // for translation, forward
  {
    double array_score[21];
    Eigen::Matrix4f array_tf[21];

#pragma omp parallel for private(icp)
    for(int dr = 0; dr <= 20; dr++) // for rotation
    {
      // Set map to icp
      icp.setInputTarget(map_ptr);

      // Setting point cloud to be aligned.
      icp.setInputSource(filtered_scan_ptr);

      icp.setMaximumIterations(maximum_iterations);
      icp.setTransformationEpsilon(transformation_epsilon);
      icp.setMaxCorrespondenceDistance(max_correspondence_distance);
      icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
      icp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);
      double d_tln = 0.0 + 1.0*dt;
      double d_rtn = -0.25 + 0.025*dr;

      // Calculate guessed pose for matching scan to map
      pose guess_pose;
      guess_pose.x = previous_pose.x + d_tln*cos(d_rtn); // previous_pose.x + diff_x;
      guess_pose.y = previous_pose.y + d_tln*sin(d_rtn); // previous_pose.y + diff_y;
      guess_pose.z = previous_pose.z; // previous_pose.z + diff_z;
      guess_pose.roll = previous_pose.roll;
      guess_pose.pitch = previous_pose.pitch;
      guess_pose.yaw = previous_pose.yaw + d_rtn;
      
      // Guess the initial gross estimation of the transformation
      Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);
      Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ());
      Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;
      
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      icp.align(*output_cloud, init_guess);
      array_score[dr] = icp.getFitnessScore();
      array_tf[dr] = icp.getFinalTransformation();
    }

    // Find best alignment in array
    icp_score[dt] = array_score[0];
    std::cout << "array_score:" << std::endl;
    for(int dr = 1; dr <= 20; dr++)
    {
      std::cout << array_score[dr] << "--";
      if(icp_score[dt] > array_score[dr])
      {
        icp_score[dt] = array_score[dr];
        icp_tf[dt] = array_tf[dr];
      }
    }
    std::cout << std::endl;
  }
  std::cout << "mp result:" << std::endl;
  // Find best alignment
  fitness_score = icp_score[0];
  t_localizer = icp_tf[0];
  for(int dt = 1; dt <= 10; dt++)
  {
    std::cout << icp_score[dt] << std::endl;
    if(fitness_score > icp_score[dt])
    {
      fitness_score = icp_score[dt];
      t_localizer = icp_tf[dt];
      std::cout << icp_tf[dt] << std::endl;
    }
  }

  // fitness_score = icp.getFitnessScore();
  // t_localizer = icp.getFinalTransformation();  // localizer
  t_base_link = t_localizer * tf_ltob;         // base_link

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update icp_pose.
  icp_pose.x = t_base_link(0, 3);
  icp_pose.y = t_base_link(1, 3);
  icp_pose.z = t_base_link(2, 3);
  mat_b.getRPY(icp_pose.roll, icp_pose.pitch, icp_pose.yaw, 1);

  current_pose.x = icp_pose.x;
  current_pose.y = icp_pose.y;
  current_pose.z = icp_pose.z;
  current_pose.roll = icp_pose.roll;
  current_pose.pitch = icp_pose.pitch;
  current_pose.yaw = icp_pose.yaw;

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = current_pose.yaw - previous_pose.yaw;
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;
  
  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift)
  {
    #ifdef MY_OFFSET_MAPPING
    // Output the cloud into pcd here
    static double pcd_count = 1; // naming purpose
    std::ostringstream scan_name;
    scan_name << "/home/zwu/offset-mapping-icp/scan" << pcd_count << ".pcd";
    std::string filename = scan_name.str();
    pcd_count++;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    scan_filtered_ptr->header.frame_id = "map";
    pcl::io::savePCDFileBinary(filename, *scan_ptr);

    // also output pose into csv 
    tf_map_csv_file << filename << "," << localizer_pose.x << "," << localizer_pose.y << "," << localizer_pose.z
                                << "," << localizer_pose.roll << "," << localizer_pose.pitch << "," << localizer_pose.yaw
                                << std::endl;
#endif // MY_OFFSET_MAPPING
    map += *transformed_scan_ptr;
    k++;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;
    isMapUpdate = true;
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  icp_map_pub.publish(*map_msg_ptr);

  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  // std::cout << "ICP has converged: " << icp.hasConverged() << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  // std::cout << "Number of iteration: " << icp.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "Number of key scans: " << k << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

#ifdef MY_SPLIT_PCD
  // Split map into small pcd files for faster processing
  static double submap_size = 0;
  static std::list< pcl::PointCloud<pcl::PointXYZI> > recent_scan_update;
  static pcl::PointCloud<pcl::PointXYZI> submap;
  if(isMapUpdate)
  {
    submap_size++;
    submap += *transformed_scan_ptr;
    if(recent_scan_update.size() < 40) // 15
      recent_scan_update.push_back(*transformed_scan_ptr);
    else
    {
      recent_scan_update.pop_front();
      recent_scan_update.push_back(*transformed_scan_ptr);
    }
    std::cout << "Sub-map updated." << std::endl;
  }
  else std::cout << "Unchanged." << std::endl;

  std::cout << "Recent_scan_update_size: " << recent_scan_update.size() << std::endl;
  std::cout << "Submap_size: " << submap_size << std::endl;

  if(submap_size > 40) // 25
  {
    output(submap);
    submap_size = 0;
    submap.clear();
    map.clear();
    // Add recent scan updates to the current map to enhance mapping
    for(std::list< pcl::PointCloud<pcl::PointXYZI> >::iterator it = recent_scan_update.begin(); it != recent_scan_update.end(); ++it) 
      map += *it;
    recent_scan_update.clear();
  }
#endif
}

void mySigintHandler(int sig) // Publish the map/final_submap if node is terminated
{
  std::string filename = "/home/zwu/mapping-icp/new_map.pcd";
  std::cout << "Writing map to pcd file." << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  map_ptr->header.frame_id = "map";
  pcl::io::savePCDFileBinary(filename, *map_ptr);
  std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
  std::cout << "Done. Node will now shutdown." << std::endl;
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  icp_pose.x = 0.0;
  icp_pose.y = 0.0;
  icp_pose.z = 0.0;
  icp_pose.roll = 0.0;
  icp_pose.pitch = 0.0;
  icp_pose.yaw = 0.0;

  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  ros::init(argc, argv, "icp_mapping", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);
  ros::NodeHandle private_nh("~");

  // setting parameters
  private_nh.getParam("maximum_iterations", maximum_iterations);
  private_nh.getParam("transformation_epsilon", transformation_epsilon);
  private_nh.getParam("max_correspondence_distance", max_correspondence_distance);
  private_nh.getParam("euclidean_fitness_epsilon", euclidean_fitness_epsilon);
  private_nh.getParam("ransac_outlier_rejection_threshold", ransac_outlier_rejection_threshold);
  private_nh.getParam("voxel_leaf_size", voxel_leaf_size);
  private_nh.getParam("min_scan_range", min_scan_range);
  private_nh.getParam("min_add_scan_shift", min_add_scan_shift);

  std::cout << "ICP Mapping Parameters: " << std::endl;
  std::cout << "maximum_iterations: " << maximum_iterations << std::endl;
  std::cout << "transformation_epsilon: " << transformation_epsilon << std::endl;
  std::cout << "max_correspondence_distance: " << max_correspondence_distance << std::endl;
  std::cout << "euclidean_fitness_epsilon: " << euclidean_fitness_epsilon << std::endl;
  std::cout << "ransac_outlier_rejection_threshold: " << ransac_outlier_rejection_threshold << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;

  if (nh.getParam("tf_x", _tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_y", _tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_z", _tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_roll", _tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_pitch", _tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_yaw", _tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  std::cout << "(tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  map.header.frame_id = "map";

  icp_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/icp_map", 1000, true);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

#ifdef MY_OFFSET_MAPPING  
  tf_map_csv_file.open(tf_map_csv_file_name);
#endif // MY_OFFSET_MAPPING

  ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);

  ros::spin();
  return 0;
}