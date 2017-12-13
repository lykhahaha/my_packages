// Basic libs
#include <chrono>
#include <fstream>
#include <iostream>
// #include <mutex>
// #include <omp.h>
#include <sstream>
#include <string>
// #include <unordered_map>
#include <vector>

// Libraries for system commands
// #include <cstdlib>
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
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float32.h>
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

// #ifdef USE_FAST_PCL
// #include <fast_pcl/registration/ndt.h>
// #include <fast_pcl/filters/voxel_grid.h>
// #else
// #include <pcl/registration/ndt.h>
// #include <pcl/filters/voxel_grid.h>
// #endif

// #include <lidar_pcl/lidar_pcl.h>
#include <lidar_pcl/struct_types.h>
#include <lidar_pcl/ndt_corrected_lidar_mapping.h>

#define EXTRACT_POSE // output pose values to csv file

#ifdef EXTRACT_POSE
std::ofstream csv_stream;
std::string csv_filename = "map_pose.csv";
#endif // EXTRACT_POSE

// global variables
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

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
std::time_t process_begin = std::time(NULL);
std::tm* pnow = std::localtime(&process_begin);

lidar_pcl::NDTCorrectedLidarMapping<pcl::PointXYZI> ndt;

void mySigintHandler(int sig) // Publish the map/final_submap if node is terminated
{
  char buffer[100];
  std::strftime(buffer, 100, "%Y%b%d_%H%M", pnow);
  std::string filename = work_directory + "map_" + std::string(buffer) + ".pcd";
  std::cout << "-----------------------------------------------------------------\n";
  std::cout << "Writing the last map to pcd file before shutting down node..." << std::endl;

  pcl::PointCloud<pcl::PointXYZI> last_map;
  for(auto& item: ndt.worldMap()) 
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
  config_stream << "Tile-map type used. Size of each tile: " 
                << "35x35" << std::endl;
  config_stream << "Size of local map: 5 tiles x 5 tiles." << std::endl;
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

  ndt.setTFCalibration(_tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw);
  ndt.setNDTTransformationEpsilon(trans_eps);
  ndt.setNDTStepSize(step_size);
  ndt.setNDTResolution(ndt_res);
  ndt.setNDTMaximumIterations(max_iter);
  ndt.setMinAddScanShift(min_add_scan_shift);
  ndt.setMinAddScanYawDiff(min_add_scan_yaw_diff);
  ndt.setVoxelLeafSize(voxel_leaf_size);

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

  	// Broadcast TF and publish MSGS
	  tf::Transform transform;
	  tf::Quaternion q;
	  Pose vehicle_pose = ndt.vehiclePose();
	  Pose lidar_pose = ndt.ndtPose();
	  transform.setOrigin(tf::Vector3(vehicle_pose.x, vehicle_pose.y, vehicle_pose.z));
	  q.setRPY(vehicle_pose.roll, vehicle_pose.pitch, vehicle_pose.yaw);
	  transform.setRotation(q);

	  tf::TransformBroadcaster br;
	  br.sendTransform(tf::StampedTransform(transform, input_cloud->header.stamp, "map", "base_link"));

    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(ndt.localMap(), *map_msg_ptr);
    ndt_map_pub.publish(*map_msg_ptr);

    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(ndt.transformedScan(), *scan_msg_ptr);
    current_scan_pub.publish(*scan_msg_ptr);

#ifdef EXTRACT_POSE
    // outputing into csv
    csv_stream << ndt.scanNumber() << "," << input_cloud->header.seq << "," 
    					 << input_cloud->header.stamp.sec << "," << input_cloud->header.stamp.nsec << ","
               << lidar_pose.x << "," << lidar_pose.y << "," << lidar_pose.z << ","
               << lidar_pose.roll << "," << lidar_pose.pitch << "," << lidar_pose.yaw
               << std::endl;
#endif // EXTRACT_POSE

    std::cout << "-----------------------------------------------------------------\n";
    std::cout << "Sequence number: " << input_cloud->header.seq << "\n";
    std::cout << "Added scan number: " << ndt.scanNumber() << "\n";
    std::cout << "Number of scan points: " << scan.size() << " points.\n";
    std::cout << "Number of filtered scan points: " << ndt.transformedScan().size() << " points.\n";
    std::cout << "Local map: " << ndt.localMap().size() << " points.\n";
    std::cout << "NDT has converged: " << ndt.NDTConvergence() << "\n";
    std::cout << "Fitness score: " << ndt.fitnessScore() << "\n";
    std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << "\n";
    std::cout << "Vehicle: " << vehicle_pose << "\n";
    std::cout << "-----------------------------------------------------------------" << std::endl;

    msg_pos++;
    std::cout << "---Number of key scans: " << ndt.scanNumber() << "\n";
    std::cout << "---Processed: " << msg_pos << "/" << msg_size << "\n";
    std::cout << "---NDT Mapping took: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1.0 << "ms.\n";
    std::cout << "-----------------------------------------------------------------" << std::endl;
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  mySigintHandler(0);

  return 0;
}
