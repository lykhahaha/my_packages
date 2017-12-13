#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstddef>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#define TILE_WIDTH 35

// #define PUBLISH_OUTPUT
// #define WRITE_CORRECTED_SCAN_TO_BAG

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

static std::unordered_map<Key, pcl::PointCloud<pcl::PointXYZI>> world_map;
static pcl::PointCloud<pcl::PointXYZI> local_map;
static Key local_key, previous_key;

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

void correctLIDARscan(pcl::PointCloud<pcl::PointXYZI>& scan, Eigen::Affine3d relative_tf, double scan_interval)
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

  scan.clear();
  pose crnt_pose = {0, 0, 0, 0, 0, 0};
  velocity vel;
  pcl::getTranslationAndEulerAngles(relative_tf, vel.x, vel.y, vel.z, vel.roll, vel.pitch, vel.yaw);
  vel.x = vel.x / scan_interval;
  vel.y = vel.y / scan_interval;
  vel.z = vel.z / scan_interval;
  vel.roll = vel.roll / scan_interval;
  vel.pitch = vel.pitch / scan_interval;
  vel.yaw = vel.yaw / scan_interval;
  for(int i = 0, npackets = scan_packets_vector.size(); i < npackets; i++)
  {
    double offset_time = scan_interval * i / npackets;
    pose this_packet_pose = {crnt_pose.x - vel.x * offset_time,
                             crnt_pose.y - vel.y * offset_time,
                             crnt_pose.z - vel.z * offset_time,
                             crnt_pose.roll - vel.roll * offset_time,
                             crnt_pose.pitch - vel.pitch * offset_time,
                             crnt_pose.yaw - vel.yaw * offset_time}; 

    Eigen::Affine3d transform;
    pcl::getTransformation(this_packet_pose.x, 
                           this_packet_pose.y, 
                           this_packet_pose.z, 
                           this_packet_pose.roll, 
                           this_packet_pose.pitch, 
                           this_packet_pose.yaw, transform);
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

int main(int argc, char** argv)
{
  // Initiate and get csv file
  ros::init(argc, argv, "submap_merger");
 #ifdef PUBLISH_OUTPUT
  ros::NodeHandle nh;
  ros::Rate rosrate(5);
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 100, true);
  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 100, true);
 #endif

  if(argc < 3)
  {
    // std::cout << "Please indicate the bag file corresponding to the map_pose.csv." << std::endl;
    std::cout << "ERROR: Missing user inputs." << std::endl;
    std::cout << "!! rosrun my_package submap_merger \"posefile.csv\" \"bagfile.bag\" [min_scan_range]" << std::endl;
    return -1;
  }
  std::string bagfile = argv[2];
  std::string bagtopic = "/points_raw";
  std::cout << "Reading bag: " << bagfile << " [topic: " << bagtopic << "]... " << std::flush;
  rosbag::Bag bag(bagfile, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string(bagtopic));
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics));
  std::cout << "Done." << std::endl;

  double min_scan_range = 0;
  if(argc == 4)
  {
    min_scan_range = std::stod(argv[3]);
    std::cout << "Using min_scan_range = " << min_scan_range << std::endl;
  }

  std::string csvfile = argv[1];
  std::cout << "Reading " << csvfile << " in the current directory.\n";
  std::cout << "Please ensure that the format of the csv file is:" << std::endl;
  std::cout << "\t{key, sequence, sec, nsec, x, y, z, roll, pitch, yaw}" << std::endl;
  std::ifstream csv_stream(csvfile);

 #ifdef WRITE_CORRECTED_SCAN_TO_BAG
  rosbag::Bag corrected_bag;
  corrected_bag.open("corrected_scan.bag", rosbag::bagmode::Write);
 #endif // WRITE_CORRECTED_SCAN_TO_BAG

  // Place-holder for variables
  pcl::PointCloud<pcl::PointXYZI> map;
  std::string line, key_str, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  unsigned int seq, key;
  double x, y, z, roll, pitch, yaw;
  Eigen::Affine3d prev_transform;
  ros::Time prev_time;
  bool isFirstScan = true;
  bool isGetLine = false;

  getline(csv_stream, line); // to skip header line
  std::cout << "Finished. Starting merging submaps..." << std::endl;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(0.3, 0.3, 0.3);
  foreach(rosbag::MessageInstance const message, view)
  {
    // get new data and process values
    if(!isGetLine)
    {  
      if(!getline(csv_stream, line))
      {
        break; // end of csv
      }
      isGetLine = true;
    }

    std::cout << line << std::endl;
    std::stringstream line_stream(line);

    getline(line_stream, key_str, ',');
    key = std::stod(key_str);

    getline(line_stream, seq_str, ',');
    seq = std::stod(seq_str);
    getline(line_stream, sec_str, ',');
    getline(line_stream, nsec_str, ',');
    ros::Time crnt_time(std::stod(sec_str), std::stod(nsec_str));
    getline(line_stream, x_str, ',');
    x = std::stod(x_str);
    getline(line_stream, y_str, ',');
    y = std::stod(y_str);
    getline(line_stream, z_str, ',');
    z = std::stod(z_str);
    getline(line_stream, roll_str, ',');
    roll = std::stod(roll_str);
    getline(line_stream, pitch_str, ',');
    pitch = std::stod(pitch_str);
    getline(line_stream, yaw_str);
    yaw = std::stod(yaw_str);

    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL) // no data?
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }
    else // check whether the sequence match
    {
      if(input_cloud->header.seq != seq)
      {
        std::cout << "Info: input_cloud->header.seq != seq (" << input_cloud->header.seq << " != " << seq << ")" << std::endl;
        continue;
      }

      if(input_cloud->header.stamp != crnt_time)
      {
        std::cout << "Error: input_cloud->header.stamp != crnt_time" << std::endl;
        std::cout << "(" << input_cloud->header.stamp << " != " << crnt_time << ")" << std::endl;
        return(-1);
      }
    }

    // Create transformation matrix
    Eigen::Affine3d crnt_transform, rel_transform;
    pcl::getTransformation(x, y, z, roll, pitch, yaw, crnt_transform);
    double interval;
    if(isFirstScan)
    {
      prev_transform = crnt_transform;
      interval = 0.1; // dummy value, just need to be != 0
      rel_transform = prev_transform.inverse() * crnt_transform;
      isFirstScan = false;
    }
    else
    {
      interval = (crnt_time - prev_time).toSec();
      rel_transform = prev_transform.inverse() * crnt_transform;
    }
#ifdef WRITE_CORRECTED_SCAN_TO_BAG
    // Correct point cloud first
    pcl::PointCloud<pcl::PointXYZI> src, fsrc, dst;
    pcl::fromROSMsg(*input_cloud, src);
    correctLIDARscan(src, rel_transform, interval);

    if(argc == 4)
    {
      // Filter pointcloud
      pcl::PointXYZI p;
      for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src.begin(); item != src.end(); item++)
      {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if(r > min_scan_range)
        {
          fsrc.push_back(p);
        }
      }
    }
    else fsrc = src;

    // Add to bag regardless of key
    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(fsrc, *scan_msg_ptr);
    scan_msg_ptr->header.seq = input_cloud->header.seq;
    scan_msg_ptr->header.stamp = input_cloud->header.stamp;
    scan_msg_ptr->header.frame_id = input_cloud->header.frame_id;
    corrected_bag.write("/velodyne_points", 
                        ros::Time(input_cloud->header.stamp.sec, input_cloud->header.stamp.nsec), 
                        *scan_msg_ptr);

    // Skip unadded scan (has key = 0)
    if(key == 0)
    {
      std::cout << "Skipping unadded scan." << std::endl;
      prev_transform = crnt_transform;
      prev_time = crnt_time;
      isGetLine = false;
      continue;
    }
#else
    // Skip unadded scan (has key = 0)
    if(key == 0)
    {
     #ifdef PUBLISH_OUTPUT
      // Correct point cloud first
      pcl::PointCloud<pcl::PointXYZI> src, fsrc, dst;
      pcl::fromROSMsg(*input_cloud, src);
      correctLIDARscan(src, rel_transform, interval);

      if(argc == 4)
      {
        // Filter pointcloud
        pcl::PointXYZI p;
        for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src.begin(); item != src.end(); item++)
        {
          p.x = (double)item->x;
          p.y = (double)item->y;
          p.z = (double)item->z;
          p.intensity = (double)item->intensity;
          if((sqrt(pow(p.x, 2.0) + pow(p.y, 2.0))) > min_scan_range)
          {
            fsrc.push_back(p);
          }
        }
      }
      else fsrc = src;

      // Transform the corrected pointcloud
      pcl::transformPointCloud(fsrc, dst, crnt_transform);

      sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(dst, *scan_msg_ptr);
      scan_msg_ptr->header.frame_id = "map";
      scan_pub.publish(scan_msg_ptr);
      rosrate.sleep();
     #endif // PUBLISH_OUTPUT

      std::cout << "Skipping unadded scan." << std::endl;
      prev_transform = crnt_transform;
      prev_time = crnt_time;
      isGetLine = false;
      continue;
    }

    // Correct point cloud first
    pcl::PointCloud<pcl::PointXYZI> src, fsrc, dst;
    pcl::fromROSMsg(*input_cloud, src);
    correctLIDARscan(src, rel_transform, interval);

    if(argc == 4)
    {
      // Filter pointcloud
      pcl::PointXYZI p;
      for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = src.begin(); item != src.end(); item++)
      {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;
        if((sqrt(pow(p.x, 2.0) + pow(p.y, 2.0))) > min_scan_range)
        {
          fsrc.push_back(p);
        }
      }
    }
    else fsrc = src;
    // pcl::io::savePCDFileBinary(key_str + ".pcd", fsrc);
#endif // WRITE_CORRECTED_SCAN_TO_BAG

    // Transform the corrected pointcloud
    pcl::transformPointCloud(fsrc, dst, crnt_transform);

    // Add to map
    // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr dst_ptr(new pcl::PointCloud<pcl::PointXYZI>(dst));
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    // voxel_grid_filter.setLeafSize(0.4, 0.4, 0.4);
    // voxel_grid_filter.setInputCloud(dst_ptr);
    // voxel_grid_filter.filter(*filtered_scan_ptr);
    map += dst;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    
   #ifdef PUBLISH_OUTPUT
    static int map_count = 0;
    static int map_loop = 5;
    sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(dst, *scan_msg_ptr);
    scan_msg_ptr->header.frame_id = "map";
    scan_pub.publish(scan_msg_ptr);

    // static int count = 0;
    if(map_count == map_loop)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
      voxel_grid_filter.setInputCloud(map_ptr);
      voxel_grid_filter.filter(*filtered_scan_ptr);
      sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(map, *map_msg_ptr);
      map_msg_ptr->header.frame_id = "map";
      map_pub.publish(map_msg_ptr);
      map_count = -1;
    }
    map_count++;
   #endif

    // Show output
    std::cout << "Number: " << key << "\n";
    std::cout << "Sequence: " << seq << "\n";
    std::cout << "Number of scan points: " << dst.size() << "\n";
    std::cout << "Number of map points: " << map.size() << "\n";
    std::cout << "(" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ")\n";
    // std::cout << "Transformation Matrix: \n";
    // std::cout << crnt_transform.matrix() << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    if(false) // to use wavelab mapping here
    {
      static int num = 1;
      char numStrScan[30];
      char numStrPose[30];
      sprintf(numStrScan, "test_data/scans/%04d.ply", num);
      sprintf(numStrPose, "test_data/poses/%04d.csv", num);
      pcl::io::savePLYFileBinary(std::string(numStrScan), src);
      std::ofstream out_stream;
      out_stream.open(std::string(numStrPose));
      out_stream << crnt_transform.matrix() << std::endl;
      num++;
    }

    // Update prev values
    prev_transform = crnt_transform;
    prev_time = crnt_time;
    isGetLine = false;
   #ifdef PUBLISH_OUTPUT
    rosrate.sleep();
   #endif
  }
  bag.close();
  std::cout << "Finished processing bag file. Saving to pcd..." << std::endl;

  std::string out_filename = "merged_map";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", map);
  std::cout << "Finished. Saved " << map.size() << " pointcloud to " << out_filename << ".pcd" << std::endl;
  return 0;
}