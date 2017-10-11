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

#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <omp.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/package.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

// #include <autoware_msgs/ConfigNdtMapping.h>
// #include <autoware_msgs/ConfigNdtMappingOutput.h>

// Here are the functions I wrote. De-comment to use
#define TILE_WIDTH 100
// #define MY_DESLANT_TRANSFORM // set z, roll, pitch to 0
#define MY_OUTPUT_DATA_CSV
#define MY_EXTRACT_SCANPOSE // do not use this, this is to extract scans and poses to close loop
static int k = 0; // key frame count

#ifdef MY_EXTRACT_SCANPOSE
std::ofstream tf_map_csv_file;
std::string tf_map_csv_file_name = "map_pose.csv";
#endif // MY_EXTRACT_SCANPOSE

#ifdef MY_OUTPUT_DATA_CSV
std::ofstream output_csv_file;
std::string csv_file_name = "allpose_data.csv";
#endif

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct Key
{
  int x;
  int y;
};

bool operator==(const Key& lhs, const Key& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const Key& lhs, const Key& rhs)
{
    return lhs.x != rhs.x || lhs.y != rhs.y;
}

namespace std
{
  template <>
  struct hash<Key> // custom hashing function for Key<(x,y)>
  {
    std::size_t operator()(const Key& k) const
    {
      return hash<int>()(k.x) ^ (hash<int>()(k.y) << 1);
    }
  };
}

// global variables
static pose previous_pose, guess_pose, current_pose, ndt_pose, added_pose, localizer_pose;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff, diff_x, diff_y, diff_z, diff_yaw; // current_pose - previous_pose

static double current_velocity_x = 0.0;
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

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

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 0.1;

static ros::Publisher ndt_map_pub;

static int initial_scan_loaded = 0;

static double min_scan_range = 2.0;
static double min_add_scan_shift = 1.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool isMapUpdate = true;

static double fitness_score;

static std::string work_directory;

// Function to set z, roll, pitch of ndt-mapping = 0 at all times
#ifdef MY_DESLANT_TRANSFORM
Eigen::Matrix4f deslant_transform(Eigen::Matrix4f t_localizer)
{
  tf::Matrix3x3 mat_l;
  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  Eigen::Matrix4f myTransform = Eigen::Matrix4f::Identity();
  // Translation
  myTransform(0, 3) = t_localizer(0, 3); // get x
  myTransform(1, 3) = t_localizer(1, 3); // get y
	myTransform(2, 3) = 2; // Off-setting -2 of tf
  double myRoll, myPitch, myYaw;
  mat_l.getRPY(myRoll, myPitch, myYaw, 1);
  myTransform(0,0) = cos (myYaw);
  myTransform(0,1) = -sin(myYaw);
  myTransform(1,0) = sin (myYaw);
  myTransform(1,1) = cos (myYaw); // Ignoring roll & pitch since we dont want them

	return myTransform;
}
#endif

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

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if(initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    add_new_scan(*transformed_scan_ptr);
    initial_scan_loaded = 1;
  }

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_ptr(new pcl::PointCloud<pcl::PointXYZI>(local_map));
  
  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
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

  ndt.align(*output_cloud, init_guess);
  fitness_score = ndt.getFitnessScore();

  t_localizer = ndt.getFinalTransformation();

  //Do de-slant (setting z, roll, pitch = 0) for transformer here
#ifdef MY_DESLANT_TRANSFORM
  t_localizer = deslant_transform(t_localizer);
#endif

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

  // Update ndt_pose.
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

#ifdef MY_OUTPUT_DATA_CSV // write data to file regardless of adding new scan
    output_csv_file << current_scan_time.sec << "." << current_scan_time.nsec << "," // timestamp
                    << current_pose.x << "," << current_pose.y << "," << current_pose.z << "," // x, y, z w.r.t. global map (global frame)
                    << current_pose.roll << "," << current_pose.pitch << "," << current_pose.yaw << std::endl; // r, p, y w.r.t. global (global frame)
#endif
  
  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift)
  {
#ifdef MY_EXTRACT_SCANPOSE
    static double pcd_count = 1;
    std::ostringstream scan_name;
    scan_name << "scan" << pcd_count << ".pcd";
    std::string filename = work_directory + scan_name.str();
    pcd_count++;
    pcl::io::savePCDFileBinary(filename, *scan_ptr);

    // outputing into csv
    tf_map_csv_file << filename << "," << localizer_pose.x << "," << localizer_pose.y << "," << localizer_pose.z
                                << "," << localizer_pose.roll << "," << localizer_pose.pitch << "," << localizer_pose.yaw
                                << std::endl;
#endif // MY_EXTRACT_SCANPOSE
	  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
    add_new_scan(*transformed_scan_ptr);
    k++;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;
    isMapUpdate = true;
  }
  // The rest of the code
	transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

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

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*local_map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Sequence number: " << input->header.seq << "\n";
  std::cout << "Number of scan points: " << scan_ptr->size() << " points.\n";
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points.\n";
  std::cout << "Local map: " << local_map.points.size() << " points.\n";
  std::cout << "NDT has converged: " << ndt.hasConverged() << "\n";
  std::cout << "Fitness score: " << fitness_score << "\n";
  std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << "\n";
  std::cout << "(x,y,z,roll,pitch,yaw):" << "\n";
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")\n";
  std::cout << "Transformation Matrix:\n";
  std::cout << t_localizer << "\n";
  std::cout << "shift: " << shift << "\n";
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

static void map_maintenance_callback(pose local_pose)
{
  // Get local_key
  local_key = {int(floor(local_pose.x / TILE_WIDTH)),  // .x
               int(floor(local_pose.y / TILE_WIDTH))}; // .y
  // local_key.x = int(floor(local_pose.x / TILE_WIDTH));
  // local_key.y = int(floor(local_pose.y / TILE_WIDTH));

  // Only need to update local_map through world_map only if local_key changes
  if(local_key != previous_key)
  {
    std::lock_guard<std::mutex> lck(mtx);
    // Get local_map, a 3x3 tile map with the center being the local_key
    local_map.clear();
    Key tmp_key;
    for(int x = local_key.x - 1; x <= local_key.x + 1; x++)
      for(int y = local_key.y - 1; y <= local_key.y + 1; y++)
      {
        tmp_key.x = x;
        tmp_key.y = y;
        local_map += world_map[tmp_key];
      }
  }

  // Update key
  previous_key = local_key;
}

void mySigintHandler(int sig) // Publish the map/final_submap if node is terminated
{
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
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

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

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

  diff = 0.0;
  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  ros::init(argc, argv, "ndt_mapping", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);
  ros::NodeHandle private_nh("~");

  // setting parameters
  std::string _bag_file;
  float _start_time, _play_duration;

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
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")\n" << std::endl;

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

  local_map.header.frame_id = "map";

  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000, true);

  work_directory = ros::package::getPath("ndt_mapping") + "/results/new/";

#ifdef MY_OUTPUT_DATA_CSV
  output_csv_file.open(work_directory + csv_file_name);
  output_csv_file << "rostime,x,y,z,roll,pitch,yaw" << std::endl;//write file headers
#endif // MY_OUTPUT_DATA_CSV

#ifdef MY_EXTRACT_SCANPOSE
  tf_map_csv_file.open(work_directory + tf_map_csv_file_name);
  tf_map_csv_file << "filename,x,y,z,roll,pitch,yaw" << std::endl;
#endif // MY_EXTRACT_SCANPOSE

  // Open bagfile
  std::cout << "Loading " << _bag_file << std::endl;
  rosbag::Bag bag(_bag_file, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string("velodyne_points"));
  ros::Time rosbag_start_time = ros::TIME_MAX;
  ros::Time rosbag_stop_time = ros::TIME_MIN;
  if(_play_duration <= 0)
  {
    rosbag::View tmp_view(bag);
    rosbag_start_time = tmp_view.getBeginTime();
    rosbag_stop_time = ros::TIME_MAX;
  }
  else
  {
    rosbag::View tmp_view(bag);
    rosbag_start_time = tmp_view.getBeginTime();
    ros::Duration sim_duration(_play_duration);
    rosbag_stop_time = rosbag_start_time + sim_duration;
    
  }
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics), rosbag_start_time, rosbag_stop_time);
  // rosbag::View view(bag, rosbag::TopicQuery(reading_topics));
  int msg_size = view.size();
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

    // Global callback to call scans process and submap process
    t1 = std::chrono::system_clock::now();
    map_maintenance_callback(current_pose);
    t2 = std::chrono::system_clock::now();
    ndt_mapping_callback(input_cloud);
    t3 = std::chrono::system_clock::now();
    // #pragma omp parallel sections
    // {
    //   #pragma omp section
    //   {
    //     ndt_mapping_callback(input_cloud);
    //   }
    //   #pragma omp section
    //   {
    //     std::cout << "current_pose: (" << current_pose.x << "," << current_pose.y << "," << current_pose.z << ","
    //                                    << current_pose.roll << "," << current_pose.pitch << "," << current_pose.yaw << std::endl;
    //     //using current_pose as local_pose to get local_map
    //     map_maintenance_callback(current_pose);
    //   }
    // }
    msg_pos++;
    std::cout << "Number of key scans: " << k << "\n";
    std::cout << "Processed: " << msg_pos << "/" << msg_size << "\n";
    std::cout << "Get local map took: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0 << "ns.\n";
    std::cout << "NDT Mapping took: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0 << "ms."<< std::endl;
  }
  bag.close();

  ros::spin();

  return 0;
}
