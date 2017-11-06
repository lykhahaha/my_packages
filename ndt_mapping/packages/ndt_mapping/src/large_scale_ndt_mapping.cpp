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

// #define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".


// Basic libs
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <omp.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// Libraries for system commands
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL & 3rd party libs
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_FAST_PCL
#include <fast_pcl/registration/ndt.h>
#include <fast_pcl/filters/voxel_grid.h>
#else
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#endif

#include <lidar_pcl/lidar_pcl.h>
#include <lidar_pcl/struct_types.h>
#include <lidar_pcl/ndt_corrected_lidar_mapping.h>

// Here are the functions I wrote. De-comment to use
#define TILE_WIDTH 35 // Maximum range of LIDAR 32E is 70m
#define EXTRACT_POSE // do not use this, this is to extract scans and poses to close loop
static int add_scan_number = 1; // added frame count

#ifdef EXTRACT_POSE
std::ofstream csv_stream;
std::string csv_filename = "map_pose.csv";
#endif // EXTRACT_POSE

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct velocity
{
  double x;
  double y;
  double z;
};

// global variables
static pose previous_pose, guess_pose, current_pose, ndt_pose, added_pose, localizer_pose;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff, diff_x, diff_y, diff_z, diff_yaw; // current_pose - previous_pose
static double secs; // scan duration
static velocity current_velocity;

static std::unordered_map<Key, pcl::PointCloud<pcl::PointXYZI>> world_map;
// static pcl::PointCloud<pcl::PointXYZI> map;
static pcl::PointCloud<pcl::PointXYZI> local_map;
static std::mutex mtx;
static Key local_key, previous_key;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static int max_iter = 300;       // Maximum iterations
static float ndt_res = 2.8;      // Resolution
static double step_size = 0.05;   // Step size
static double trans_eps = 0.001;  // Transformation epsilon

static float _start_time = 0; // 0 means start playing bag from beginnning
static float _play_duration = -1; // negative means play everything
static double min_scan_range = 2.0;
static double min_add_scan_shift = 1.0;
static double min_add_scan_yaw_diff = 0.005;
static std::string _bag_file;
static std::string work_directory;

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 0.1;

static ros::Publisher ndt_map_pub, current_scan_pub;

static int initial_scan_loaded = 0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool isMapUpdate = true;

static double fitness_score;

// File name get from time
std::time_t process_begin = std::time(NULL);
std::tm* pnow = std::localtime(&process_begin);

inline double getYawAngle(double _x, double _y)
{
  return std::atan2(_y, _x) * 180 / 3.14159265359; // degree value
}

inline double calculateMinAngleDist(double first, double second) // in degree
{
  double difference = first - second;
  if(difference >= 180.0)
    return difference - 360.0;
  if(difference <= -180.0)
    return difference + 360.0;
  return difference;
}

void correctLIDARscan(pcl::PointCloud<pcl::PointXYZI>& scan, velocity vel, double scan_interval)
{
  // Correct scan using vel
  pcl::PointCloud<pcl::PointXYZI> scan_packet;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_packets_vector;
  double base_azimuth = getYawAngle((scan.begin())->x, (scan.begin())->y);
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin(); item != scan.end(); item++)
  {
    double crnt_azimuth = getYawAngle(item->x, item->y);
    if(std::fabs(calculateMinAngleDist(crnt_azimuth, base_azimuth)) < 0.01) // 0.17 degree is the typical change
    {
      scan_packet.push_back(*item);
    }
    else // new azimuth reached
    {
      scan_packets_vector.push_back(scan_packet);
      scan_packet.clear();
      scan_packet.push_back(*item);
      base_azimuth = crnt_azimuth;
    }
  }

  // assuming T'' = 0 and only linear motion
  scan.clear();
  pose crnt_pose = {0, 0, 0, 0, 0, 0};
  for(int i = 0, npackets = scan_packets_vector.size(); i < npackets; i++)
  {
    double offset_time = scan_interval * i / npackets;
    pose this_packet_pose = {crnt_pose.x - vel.x * offset_time,
                             crnt_pose.y - vel.y * offset_time,
                             crnt_pose.z - vel.z * offset_time,
                             crnt_pose.roll, crnt_pose.pitch, crnt_pose.yaw}; 

    Eigen::Affine3f transform = pcl::getTransformation(this_packet_pose.x, 
                                                       this_packet_pose.y, 
                                                       this_packet_pose.z, 
                                                       this_packet_pose.roll, 
                                                       this_packet_pose.pitch, 
                                                       this_packet_pose.yaw);
    pcl::PointCloud<pcl::PointXYZI> corrected_packet;
    pcl::transformPointCloud(scan_packets_vector[npackets-1-i], corrected_packet, transform);
    scan += corrected_packet;
  }
}

static void add_new_scan(const pcl::PointCloud<pcl::PointXYZI> new_scan)
{
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = new_scan.begin(); item < new_scan.end(); item++)
  {
    // Get 2D point
    Key key;
    key.x = int(floor(item->x / TILE_WIDTH));
    key.y = int(floor(item->y / TILE_WIDTH));

    world_map[key].push_back(*item);
  }

  local_map += new_scan;
}

static void ndt_mapping_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
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
  // lidar_pcl::fromROSMsg(*input, tmp); // note here

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (r > min_scan_range)
    {
      scan.push_back(p);
    }
  }

  correctLIDARscan(scan, current_velocity, secs);
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if(initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    add_new_scan(*transformed_scan_ptr);
    initial_scan_loaded = 1;
#ifdef EXTRACT_POSE

    // outputing into csv
    csv_stream << add_scan_number << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
               << _tf_x << "," << _tf_y << "," << _tf_z << "," 
               << _tf_roll << "," << _tf_pitch << "," << _tf_yaw
               << std::endl;
    add_scan_number++;
    return;
#endif // EXTRACT_POSE
  }

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_ptr(new pcl::PointCloud<pcl::PointXYZI>(local_map));
  
  ndt.setInputSource(filtered_scan_ptr);

  if (isMapUpdate == true)
  {
    ndt.setInputTarget(local_map_ptr);
    isMapUpdate = false;
  }

  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

#ifdef USE_FAST_PCL
  ndt.omp_align(*output_cloud, init_guess);
  fitness_score = ndt.getFitnessScore();
#else
  ndt.align(*output_cloud, init_guess);
  fitness_score = ndt.getFitnessScore();
#endif

  // fitness_score = ndt.getFitnessScore();

  t_localizer = ndt.getFinalTransformation();

  t_base_link = t_localizer * tf_ltob;

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

  // Update ndt_pose
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = current_pose.yaw - previous_pose.yaw;
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  
  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if(shift >= min_add_scan_shift || diff_yaw >= min_add_scan_yaw_diff)
  {
#ifdef EXTRACT_POSE

    // outputing into csv
    csv_stream << add_scan_number << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
               << localizer_pose.x << "," << localizer_pose.y << "," << localizer_pose.z << ","
               << localizer_pose.roll << "," << localizer_pose.pitch << "," << localizer_pose.yaw
               << std::endl;
#endif // EXTRACT_POSE

    // add_new_scan(*transformed_scan_ptr);
    add_new_scan(*output_cloud);
    add_scan_number++;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;
    isMapUpdate = true;
  }
#ifdef EXTRACT_POSE
  else
  {
    // outputing into csv, with add_scan_number = 0
    csv_stream << 0 << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
               << localizer_pose.x << "," << localizer_pose.y << "," << localizer_pose.z << ","
               << localizer_pose.roll << "," << localizer_pose.pitch << "," << localizer_pose.yaw
               << std::endl;
  }
#endif // EXTRACT_POSE

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  // The rest of the code
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  secs = scan_duration.toSec();

  current_velocity.x = diff_x / secs;
  current_velocity.y = diff_y / secs;
  current_velocity.z = diff_z / secs;

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*local_map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
  transformed_scan_ptr->header.frame_id = "map";
  pcl::toROSMsg(*transformed_scan_ptr, *scan_msg_ptr);
  current_scan_pub.publish(*scan_msg_ptr);

  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Sequence number: " << input->header.seq << "\n";
  std::cout << "Number of scan points: " << scan_ptr->size() << " points.\n";
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points.\n";
  std::cout << "Local map: " << local_map.points.size() << " points.\n";
  std::cout << "NDT has converged: " << ndt.hasConverged() << "\n";
  std::cout << "Fitness score: " << fitness_score << "\n";
  std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << "\n";
  // std::cout << "Guessed posed: " << "\n";
  // std::cout << "(" << guess_pose.x << ", " << guess_pose.y << ", " << guess_pose.z << ", " << guess_pose.roll
  //           << ", " << guess_pose.pitch << ", " << guess_pose.yaw << ")\n";
  std::cout << "(x,y,z,roll,pitch,yaw):" << "\n";
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")\n";
  // std::cout << "Transformation Matrix:\n";
  // std::cout << t_localizer << "\n";
  std::cout << "Shift: " << shift << "\n";
  std::cout << "Yaw Diff: " << diff_yaw << "\n";
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

static void map_maintenance_callback(pose local_pose)
{
  // Get local_key
  local_key = {int(floor(local_pose.x / TILE_WIDTH)),  // .x
               int(floor(local_pose.y / TILE_WIDTH))}; // .y
  // local_key.x = int(floor(local_pose.x / TILE_WIDTH));
  // local_key.y = int(floor(local_pose.y / TILE_WIDTH));

  // Only update local_map through world_map only if local_key changes
  if(local_key != previous_key)
  {
    std::lock_guard<std::mutex> lck(mtx);
    // Get local_map, a 3x3 tile map with the center being the local_key
    local_map.clear();
    Key tmp_key;
    for(int x = local_key.x - 2, x_max = local_key.x + 2; x <= x_max; x++)
      for(int y = local_key.y - 2, y_max = local_key.y + 2; y <= y_max; y++)
      {
        tmp_key.x = x;
        tmp_key.y = y;
        local_map += world_map[tmp_key];
      }

    // Update key
    previous_key = local_key;
  }
}

void mySigintHandler(int sig) // Publish the map/final_submap if node is terminated
{
  char buffer[100];
  std::strftime(buffer, 100, "%Y%b%d_%H%M", pnow);
  std::string filename = work_directory + "map_" + std::string(buffer) + ".pcd";
  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Writing the last map to pcd file before shutting down node..." << std::endl;

  pcl::PointCloud<pcl::PointXYZI> last_map;
  for (auto& item: world_map) 
    last_map += item.second;

  last_map.header.frame_id = "map";
  pcl::io::savePCDFileBinary(filename, last_map);
  std::cout << "Saved " << last_map.points.size() << " data points to " << filename << ".\n";
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Done. Node will now shutdown." << std::endl;

  // Write a config file
  char cbuffer[100];
  std::ofstream config_stream;
  std::strftime(cbuffer, 100, "%b%d-%H%M", pnow);
  std::string config_file = "config@" + std::string(cbuffer) + ".txt";
  config_stream.open(work_directory + config_file);
  std::strftime(cbuffer, 100, "%c", pnow);

  std::time_t process_end = std::time(NULL);
  double process_duration = difftime(process_end, process_begin); // calculate processing duration
  int process_hr = int(process_duration / 3600);
  int process_min = int((process_duration - process_hr * 3600) / 60);
  double process_sec = process_duration - process_hr * 3600 - process_min * 60;

  config_stream << "Created @ " << std::string(cbuffer) << std::endl;
  config_stream << "Map: " << _bag_file << std::endl;
  config_stream << "Start time: " << _start_time << std::endl;
  config_stream << "Play duration: " << _play_duration << std::endl;
  config_stream << "Corrected end scan pose: ?\n" << std::endl;
  config_stream << "###\nResolution: " << ndt_res << std::endl;
  config_stream << "Step Size: " << step_size << std::endl;
  config_stream << "Transformation Epsilon: " << trans_eps << std::endl;
  config_stream << "Leaf Size: " << voxel_leaf_size << std::endl;
  config_stream << "Minimum Scan Range: " << min_scan_range << std::endl;
  config_stream << "Minimum Add Scan Shift: " << min_add_scan_shift << std::endl;
  config_stream << "Minimum Add Scan Yaw Change: " << min_add_scan_yaw_diff << std::endl;
#ifdef TILE_WIDTH
  config_stream << "Tile-map type used. Size of each tile: " 
                << TILE_WIDTH << "x" << TILE_WIDTH << std::endl;
  config_stream << "Size of local map: 5 tiles x 5 tiles." << std::endl;
#endif // TILE_WIDTH
  config_stream << "Time taken: " << process_hr << " hr "
                                  << process_min << " min "
                                  << process_sec << " sec" << std::endl;


  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_mapping", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);
  ros::NodeHandle private_nh("~");

  // setting parameters
  private_nh.getParam("bag_file", _bag_file);
  private_nh.getParam("start_time", _start_time);
  private_nh.getParam("play_duration", _play_duration);
  private_nh.getParam("resolution", ndt_res);
  private_nh.getParam("step_size", step_size);  
  private_nh.getParam("transformation_epsilon", trans_eps);
  private_nh.getParam("max_iteration", max_iter);
  private_nh.getParam("voxel_leaf_size", voxel_leaf_size);
  private_nh.getParam("min_scan_range", min_scan_range);
  private_nh.getParam("min_add_scan_shift", min_add_scan_shift);
  private_nh.getParam("min_add_scan_yaw_diff", min_add_scan_yaw_diff);
  private_nh.getParam("tf_x", _tf_x);
  private_nh.getParam("tf_y", _tf_y);
  private_nh.getParam("tf_z", _tf_z);
  private_nh.getParam("tf_roll", _tf_roll);
  private_nh.getParam("tf_pitch", _tf_pitch);
  private_nh.getParam("tf_yaw", _tf_yaw);

  std::cout << "\nNDT Mapping Parameters:" << std::endl;
  std::cout << "bag_file: " << _bag_file << std::endl;
  std::cout << "start_time: " << _start_time << std::endl;
  std::cout << "play_duration: " << _play_duration << std::endl;
  std::cout << "ndt_res: " << ndt_res << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_epsilon: " << trans_eps << std::endl;
  std::cout << "max_iter: " << max_iter << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
  std::cout << "min_add_scan_yaw_diff: " << min_add_scan_yaw_diff << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")\n" << std::endl;

  lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZI> ndt;
  ndt.setTFCalibration(_tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw);
  ndt.setNDTTransformationEpsilon(trans_eps);
  ndt.setNDTStepSize(step_size);
  ndt.setNDTResolution(ndt_res);
  ndt.setNDTMaximumIterations(max_iter);

  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1000, true);
  current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 1, true);

  const char *home_directory;
  if((home_directory = getenv("HOME")) == NULL)
    home_directory = getpwuid(getuid())->pw_dir; // get home directory

  work_directory = std::string(home_directory) + "/ndt_custom/";
  std::cout << "Results are stored in: " << work_directory << std::endl;

#ifdef EXTRACT_POSE // map_pose.csv
  csv_stream.open(work_directory + csv_filename);
  csv_stream << "key,sequence,sec,nsec,x,y,z,roll,pitch,yaw" << std::endl;
#endif // EXTRACT_POSE

  // Open bagfile with topics, timestamps indicated
  std::cout << "Loading " << _bag_file << std::endl;
  rosbag::Bag bag(_bag_file, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string("/points_raw"));
  ros::Time rosbag_start_time = ros::TIME_MAX;
  ros::Time rosbag_stop_time = ros::TIME_MIN;
  if(_play_duration <= 0)
  {
    rosbag::View tmp_view(bag);
    rosbag_start_time = tmp_view.getBeginTime() + ros::Duration(_start_time);
    rosbag_stop_time = ros::TIME_MAX;
  }
  else
  {
    rosbag::View tmp_view(bag);
    rosbag_start_time = tmp_view.getBeginTime() + ros::Duration(_start_time);
    ros::Duration sim_duration(_play_duration);
    rosbag_stop_time = rosbag_start_time + sim_duration;
  }
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics), rosbag_start_time, rosbag_stop_time);
  const int msg_size = view.size();
  int msg_pos = 0;

  // Looping, processing messages in bag file
  std::chrono::time_point<std::chrono::system_clock> t1, t2, t3;
  std::cout << "Finished preparing bagfile. Starting mapping..." << std::endl;
  std::cout << "Note: if the mapping does not start immediately, check the subscribed topic names.\n" << std::endl;
  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL)
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }

    t1 = std::chrono::system_clock::now();

    double r;
    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;

    pcl::fromROSMsg(*input_cloud, tmp);

    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
      p.x = (double)item->x;
      p.y = (double)item->y;
      p.z = (double)item->z;
      p.intensity = (double)item->intensity;

      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (r > min_scan_range)
      {
        scan.push_back(p);
      }
    }

    ndt.doNDTMapping(scan, input_cloud->header.stamp);
    t2 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(ndt.localMap(), *map_msg_ptr);
    ndt_map_pub.publish(*map_msg_ptr);

    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(ndt.transformedScan(), *scan_msg_ptr);
    current_scan_pub.publish(*scan_msg_ptr);

    std::cout << "-----------------------------------------------------------------\n";
    std::cout << "Sequence number: " << input_cloud->header.seq << "\n";
    std::cout << "Number of scan points: " << scan.size() << " points.\n";
    std::cout << "Number of filtered scan points: " << ndt.transformedScan().size() << " points.\n";
    std::cout << "Local map: " << ndt.localMap().size() << " points.\n";
    std::cout << "NDT has converged: " << ndt.NDTConvergence() << "\n";
    std::cout << "Fitness score: " << ndt.fitnessScore() << "\n";
    std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << "\n";
    // std::cout << "Guessed posed: " << "\n";
    // std::cout << "(" << guess_pose.x << ", " << guess_pose.y << ", " << guess_pose.z << ", " << guess_pose.roll
    //           << ", " << guess_pose.pitch << ", " << guess_pose.yaw << ")\n";
    std::cout << "(x,y,z,roll,pitch,yaw):" << "\n";
    std::cout << "(" << ndt.ndtPose().x << ", " << ndt.ndtPose().y << ", " << ndt.ndtPose().z << ", "
              << ndt.ndtPose().roll << ", " << ndt.ndtPose().pitch << ", " << ndt.ndtPose().yaw << ")\n";
    // std::cout << "Transformation Matrix:\n";
    // std::cout << t_localizer << "\n";
    std::cout << "-----------------------------------------------------------------" << std::endl;

    msg_pos++;
    std::cout << "---Number of key scans: " << ndt.scanNumber() << "\n";
    std::cout << "---Processed: " << msg_pos << "/" << msg_size << "\n";
    std::cout << "---NDT Mapping took: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1.0 << "ms."<< std::endl;
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  mySigintHandler(0);

  return 0;
}
