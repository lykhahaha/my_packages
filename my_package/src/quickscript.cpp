// Standard libs
#include <chrono>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <math.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// System command libs
#include <sys/types.h>
#include <sys/stat.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL libs & 3rd party
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

// Custom libs
#include <lidar_pcl/data_types.h>
#include <lidar_pcl/motion_undistortion.h>

// #define OUTPUT_POSE
#define ICP_POINT2PLANE

// global variables
static Pose previous_pose, guess_pose, current_pose, icp_pose, added_pose, localizer_pose;
static Eigen::Affine3d current_pose_tf, previous_pose_tf, relative_pose_tf;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;
static ros::Publisher icp_map_pub, current_scan_pub;

static double diff, diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw; // current_pose - previous_pose

// static double current_velocity_x = 0.0;
// static double current_velocity_y = 0.0;
// static double current_velocity_z = 0.0;

static std::unordered_map<Key, pcl::PointCloud<pcl::PointXYZI>> world_map;
static pcl::PointCloud<pcl::PointXYZI> local_map;
static Key local_key, previous_key;
static const double TILE_WIDTH = 35;

#ifdef ICP_POINT2PLANE
static pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
#else
static pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
#endif
static pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;

// Default values for ICP
static int maximum_iterations = 100;
static double transformation_epsilon = 0.01;
static double max_correspondence_distance = 1.0;
static double euclidean_fitness_epsilon = 0.1;
static double ransac_outlier_rejection_threshold = 1.0;

static double voxel_leaf_size = 1.0;
static double min_scan_range = 2.0;
static double min_add_scan_shift = 1.0;
static double min_add_scan_yaw_diff = 0.005;

static float _start_time = 0; // 0 means start playing bag from beginnning
static float _play_duration = -1; // negative means play everything
static std::string _bag_file;
static std::string _output_directory;
static std::string _namespace;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static int add_scan_number = 0; // added frame count
static int initial_scan_loaded = 0;
static bool isMapUpdate = true;
static double fitness_score;
static bool has_converged;
// static int final_num_iteration;

#ifdef OUTPUT_POSE
std::ofstream csv_stream;
std::string csv_filename = "map_pose.csv";
#endif

// File name get from time
std::time_t process_begin = std::time(NULL);
std::tm* pnow = std::localtime(&process_begin);

static void addNewScan(const pcl::PointCloud<pcl::PointXYZI> new_scan)
{
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = new_scan.begin(); item < new_scan.end(); item++)
  {
    // Get 2D point
    Key key{int(floor(item->x / TILE_WIDTH)),
            int(floor(item->y / TILE_WIDTH))};
    // key.x = int(floor(item->x / TILE_WIDTH));
    // key.y = int(floor(item->y / TILE_WIDTH));

    world_map[key].push_back(*item);
  }

  local_map += new_scan;
}

static void addNormalComponent(pcl::PointCloud<pcl::PointXYZI>& pcloud,
                               pcl::PointCloud<pcl::PointXYZINormal>& pcloud_with_normal)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcloud));
  pcl::PointCloud<pcl::Normal>::Ptr pcloud_normals_ptr(new pcl::PointCloud<pcl::Normal>());
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree_ptr(new pcl::search::KdTree<pcl::PointXYZI>());

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(pcloud_ptr);
  normal_estimator.setSearchMethod(search_tree_ptr);
  // normal_estimator.setRadiusSearch(0.1);
  normal_estimator.setKSearch(3);
  normal_estimator.compute(*pcloud_normals_ptr);
  
  // Output
  pcloud_with_normal.clear();  
  pcl::concatenateFields(pcloud, *pcloud_normals_ptr, pcloud_with_normal);
}

static void icpMappingCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
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

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (r > min_scan_range)
      scan.push_back(p);
  }

  // lidar_pcl::motionUndistort(scan, relative_pose_tf);
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if(initial_scan_loaded == 0)
  {
    add_scan_number++;
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    addNewScan(*transformed_scan_ptr);
    initial_scan_loaded = 1;
    isMapUpdate = true;
   #ifdef OUTPUT_POSE
    csv_stream << add_scan_number << "," << input->header.seq << "," << current_scan_time.sec << "," << current_scan_time.nsec << ","
               << _tf_x << "," << _tf_y << "," << _tf_z << "," 
               << _tf_roll << "," << _tf_pitch << "," << _tf_yaw
               << std::endl;
   #endif // OUTPUT_POSE
    std::cout << "Added initial scan to map" << std::endl;
    return;
  }

  // Apply voxelgrid filter
  if(voxel_leaf_size < 0.001)
    filtered_scan_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(*scan_ptr));
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
  }

  // Settin point cloud to be aligned
  pcl::PointCloud<pcl::PointXYZINormal> filtered_scan_with_normal;
  addNormalComponent(*filtered_scan_ptr, filtered_scan_with_normal);
  std::vector<int> tmp_idx;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_scan_ptr_2(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::removeNaNNormalsFromPointCloud(filtered_scan_with_normal, *filtered_scan_ptr_2, tmp_idx);
  icp.setInputSource(filtered_scan_ptr_2);

  // Set map as target point cloud
  pcl::PointCloud<pcl::PointXYZI> local_map_valid;
  tmp_idx.clear();
  pcl::removeNaNFromPointCloud(local_map, local_map_valid, tmp_idx);

  pcl::PointCloud<pcl::PointXYZINormal> local_map_with_normal;
  addNormalComponent(local_map_valid, local_map_with_normal);

  tmp_idx.clear();
  pcl::PointCloud<pcl::PointXYZINormal> local_map_with_normal_valid;
  pcl::removeNaNNormalsFromPointCloud(local_map_with_normal, local_map_with_normal_valid, tmp_idx);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map_ptr(new pcl::PointCloud<pcl::PointXYZINormal>(local_map_with_normal_valid));

  pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
  icp->setMaximumIterations(10);
  icp->setInputSource(filtered_scan_ptr_2); // not cloud_source, but cloud_source_trans!
  icp->setInputTarget(local_map_ptr);
  icp->align(*filtered_scan_ptr_2, Eigen::Matrix4f::Identity());
  std::cout << icp->getFinalTransformation() << std::endl;
}

void mySigintHandler(int sig) // Publish the map/final_submap if node is terminated
{
  // char buffer[100];
  // std::strftime(buffer, 100, "%Y%b%d_%H%M", pnow);
  // std::string filename = _output_directory + "icp_" + std::string(buffer) + ".pcd";

  // // Write a config file
  // char cbuffer[100];
  // std::ofstream config_stream;
  // std::strftime(cbuffer, 100, "%b%d-%H%M", pnow);
  // std::string config_file = "config@" + std::string(cbuffer) + ".txt";
  // config_stream.open(_output_directory + config_file);
  // std::strftime(cbuffer, 100, "%c", pnow);

  // std::time_t process_end = std::time(NULL);
  // double process_duration = difftime(process_end, process_begin); // calculate processing duration
  // int process_hr = int(process_duration / 3600);
  // int process_min = int((process_duration - process_hr * 3600) / 60);
  // double process_sec = process_duration - process_hr * 3600 - process_min * 60;

  // config_stream << "Created @ " << std::string(cbuffer) << std::endl;
  // config_stream << "Map: " << _bag_file << std::endl;
  // config_stream << "Start time: " << _start_time << std::endl;
  // config_stream << "Play duration: " << (_play_duration < 0 ? "all" : std::to_string(_play_duration)) << std::endl;
  // config_stream << "Corrected end scan pose: ..\n" << std::endl;
  // config_stream << "\nMaximum Iteration Number: " << maximum_iterations << std::endl;
  // config_stream << "Transformation Epsilon: " << transformation_epsilon << std::endl;
  // config_stream << "Max Correspondence Dist: " << max_correspondence_distance << std::endl;
  // config_stream << "Euclidean Fitness Epsilon: " << euclidean_fitness_epsilon << std::endl;
  // config_stream << "RANSAC Outlier Rejection Threshold: " << ransac_outlier_rejection_threshold << std::endl;
  // config_stream << "\nLeaf Size: " << voxel_leaf_size << std::endl;
  // config_stream << "Minimum Scan Range: " << min_scan_range << std::endl;
  // config_stream << "Minimum Add Scan Shift: " << min_add_scan_shift << std::endl;
  // config_stream << "Minimum Add Scan Yaw Change: " << min_add_scan_yaw_diff << std::endl;
  // config_stream << "Tile-map type used. Size of each tile: " 
  //               << TILE_WIDTH << "x" << TILE_WIDTH << std::endl;
  // config_stream << "Size of local map: 5 tiles x 5 tiles." << std::endl;
  // config_stream << "Time taken: " << process_hr << " hr "
  //                                 << process_min << " min "
  //                                 << process_sec << " sec" << std::endl;

  // std::cout << "-----------------------------------------------------------------\n";
  // std::cout << "Writing the last map to pcd file before shutting down node..." << std::endl;

  // pcl::PointCloud<pcl::PointXYZI> last_map;
  // for (auto& item: world_map) 
  //   last_map += item.second;

  // last_map.header.frame_id = "map";
  // pcl::io::savePCDFileBinary(filename, last_map);
  // std::cout << "Saved " << last_map.points.size() << " data points to " << filename << ".\n";
  // std::cout << "-----------------------------------------------------------------" << std::endl;
  // std::cout << "Done. Node will now shutdown." << std::endl;
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  ros::init(argc, argv, "icp_mapping", ros::init_options::NoSigintHandler);
  ros::NodeHandle private_nh("~");

  // get setting parameters in the launch files
  private_nh.getParam("bag_file", _bag_file);
  private_nh.getParam("start_time", _start_time);
  private_nh.getParam("play_duration", _play_duration);
  private_nh.getParam("namespace", _namespace);
  private_nh.getParam("output_directory", _output_directory);

  private_nh.getParam("maximum_iterations", maximum_iterations);
  private_nh.getParam("transformation_epsilon", transformation_epsilon);
  private_nh.getParam("max_correspondence_distance", max_correspondence_distance);
  private_nh.getParam("euclidean_fitness_epsilon", euclidean_fitness_epsilon);
  private_nh.getParam("ransac_outlier_rejection_threshold", ransac_outlier_rejection_threshold);

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

  std::cout << "\nICP Mapping Parameters: " << std::endl;
  std::cout << "bag_file: " << _bag_file << std::endl;
  std::cout << "start_time: " << _start_time << std::endl;
  std::cout << "play_duration: " << _play_duration << std::endl;
  std::cout << "namespace: " << (_namespace.size() > 0 ? _namespace : "N/A") << std::endl;
  std::cout << "output directory: " << _output_directory << std::endl;
  std::cout << "maximum_iterations: " << maximum_iterations << std::endl;
  std::cout << "transformation_epsilon: " << transformation_epsilon << std::endl;
  std::cout << "max_correspondence_distance: " << max_correspondence_distance << std::endl;
  std::cout << "euclidean_fitness_epsilon: " << euclidean_fitness_epsilon << std::endl;
  std::cout << "ransac_outlier_rejection_threshold: " << ransac_outlier_rejection_threshold << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
  std::cout << "min_add_scan_yaw_diff: " << min_add_scan_yaw_diff << std::endl;
  std::cout << "(tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);

  if(_namespace.size() > 0)
  {
    icp_map_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/local_map", 100, true);
    current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>(_namespace + "/current_scan", 100, true);
  }
  else
  {
    icp_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 100, true);
    current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 100, true);
  }

  try
  {
    mkdir(_output_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }
  catch(...)
  {
    std::cout << "ERROR: Failed to access directory:" << _output_directory << std::endl;
    return -1;
  }

  icp.setMaximumIterations(maximum_iterations);
  icp.setTransformationEpsilon(transformation_epsilon);
  icp.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
  icp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);

  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

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

 #ifdef OUTPUT_POSE
  csv_stream.open(_output_directory + csv_filename);
  csv_stream << "key,sequence,sec,nsec,x,y,z,roll,pitch,yaw" << std::endl;
 #endif

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

    // Global callback to call scans process and submap process
    t1 = std::chrono::system_clock::now();
    // mapMaintenanceCallback(current_pose);
    t2 = std::chrono::system_clock::now();
    icpMappingCallback(input_cloud);
    t3 = std::chrono::system_clock::now();
    // #pragma omp parallel sections
    // {
    //   #pragma omp section
    //   {
    //     icpMappingCallback(input_cloud);
    //   }
    //   #pragma omp section
    //   {
    //     std::cout << "current_pose: (" << current_pose.x << "," << current_pose.y << "," << current_pose.z << ","
    //                                    << current_pose.roll << "," << current_pose.pitch << "," << current_pose.yaw << std::endl;
    //     //using current_pose as local_pose to get local_map
    //     mapMaintenanceCallback(current_pose);
    //   }
    // }
    msg_pos++;
    std::cout << "---Number of key scans: " << add_scan_number << "\n";
    std::cout << "---Processed: " << msg_pos << "/" << msg_size << "\n";
    std::cout << "---Getting local map took: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0 << "ns.\n";
    std::cout << "---ICP Mapping took: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0 << "ms.\n";
    std::cout << "###############################################" << std::endl;
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  mySigintHandler(0);

  return 0;
}