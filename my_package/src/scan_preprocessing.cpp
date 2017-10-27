#define PCL_NO_PRECOMPILE // to create a custom pcl point type
// Basic libs
// #include <chrono>
#include <fstream>
#include <iostream>
#include <memory.h>
#include <sstream>
// #include <mutex>
// #include <omp.h>
#include <sstream>
#include <string>
#include <stdlib.h>
// #include <cmath>
// #include <unordered_map>
// #include <vector>

// Libraries for system commands
// #include <cstdlib>
// #include <unistd.h>
// #include <sys/types.h>
// #include <pwd.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

// ROS libs
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// #include <ros/time.h>
// #include <ros/duration.h>
// #include <std_msgs/Bool.h>
// // #include <std_msgs/Float32.h>
// #include <pcl/PCLPointField.h>
// #include <sensor_msgs/PointField.h>
// #include <pcl/PCLPointCloud2.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>

// // PCL & 3rd party libs
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher current_scan_pub;

const int LAYER_NUM = 32;
ros::Rate *r;

struct PointXYZIR
{
  PCL_ADD_POINT4D; // add x,y,z member + padding
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  union
  {
    struct
    {
      float intensity;
      uint16_t ring;
    };
    float data_c[4];
  };

  inline PointXYZIR()
  {
    x = 0.; y = 0.; z = 0.;
    intensity = 0.;
    ring = 0;
  }

  inline PointXYZIR(const PointXYZIR &p)
  {
    x = p.x; y = p.y; z = p.z;
    intensity = p.intensity;
    ring = p.ring;
  }

  inline PointXYZIR(float _x, float _y, float _z, float _intensity, uint16_t _ring)
  {
    x = _x; y = _y; z = _z;
    intensity = _intensity;
    ring = _ring;
  }

  inline PointXYZIR(const uint8_t * source)
  {
    memcpy(&x, source, 4);
    memcpy(&y, source + 4, 4);
    memcpy(&z, source + 8, 4);
    memcpy(&intensity, source + 16, 4);
    memcpy(&ring, source + 20, 4);
  }

  inline float yaw()
  {
    return std::atan2(y, x) * 180 / 3.14159265359; // degree value
  }

  friend std::ostream& operator<<(std::ostream& os, const PointXYZIR& ret) 
  { 
    os << "(" << ret.x << "," << ret.y << "," << ret.z << "," 
       << ret.intensity << "," << ret.ring << ")" ;  
    return os;  
  } 
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,  // here we assume a XYZ + "intensity" + ring (as fields)
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (uint16_t, ring, ring)
)

// typedef struct PointXYZIR PointXYZIR;
void fromPCLPointCloud2(const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointXYZIR>& cloud,
                        const pcl::MsgFieldMap& field_map)
{
  // Copy info fields
  cloud.header   = msg.header;
  cloud.width    = msg.width;
  cloud.height   = msg.height;
  cloud.is_dense = msg.is_dense == 1;
   
  // Copy point data
  uint32_t num_points = msg.width * msg.height;
  cloud.points.resize(num_points);
  uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud.points[0]);
  
  // Get first point data
  PointXYZIR first_point(&msg.data[0]);
  float origin_angle = first_point.yaw();
  bool half_circle_check = false;
  float stop_threshold = 1.0; // in degree
  // std::cout << "ori: " << origin_angle << std::endl;
  // memcpy each group of contiguous fields separately
  for (uint32_t row = 0; row < msg.height; ++row)
  {
    const uint8_t* row_data = &msg.data[row * msg.row_step];
    for (uint32_t col = 0; col < msg.width; ++col)
    {
      const uint8_t* msg_data = row_data + col * msg.point_step;

      // Get current point to determine whether to add to cloud
      PointXYZIR current_point(&msg_data[0]);
      float current_angle = current_point.yaw();
      // std::cout << "cur: " << current_angle << std::endl;
      // assuming CW (-) rotation (checked with all current models)
      if(!half_circle_check)
      {
        if(std::fabs(current_angle - origin_angle) > 1.570796)
          half_circle_check = true;
        BOOST_FOREACH (const pcl::detail::FieldMapping& mapping, field_map)
        {
          memcpy(cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
        }
        cloud_data += sizeof(PointXYZIR);
      }
      else // half_circle_check == true
      {
        if(std::fabs(current_angle - origin_angle) > stop_threshold)
        {
          BOOST_FOREACH (const pcl::detail::FieldMapping& mapping, field_map)
          {
            memcpy(cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
          }
          cloud_data += sizeof(PointXYZIR);
        }
        else
        {
          // 360 degree has been traversed through, stop adding more points to cloud
          // Resize cloud to fit then exit
          cloud.width = col + 1;
          num_points = row * msg.width + (col + 1);
          cloud.points.resize(num_points);
          return; 
        }

      }
    }
  }
}

void fromROSMsg(const sensor_msgs::PointCloud2::ConstPtr& input, pcl::PointCloud<PointXYZIR> &pcl_cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  // pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
  pcl::MsgFieldMap field_map;
  pcl::createMapping<PointXYZIR>(pcl_pc2.fields, field_map);
  fromPCLPointCloud2(pcl_pc2, pcl_cloud, field_map);
}

static void ndt_mapping_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // CSV file
  // std::string csv_file = "/home/zwu/trash_ws/scan" + std::to_string(input->header.seq) + ".csv";
  // std::ofstream csv_stream;
  // csv_stream.open(csv_file);
  // for(int i = 0; i < LAYER_NUM; i++)
  //   csv_stream << i << ",";
  // csv_stream << std::endl;  

  // bool ring_check[LAYER_NUM] = {false};
  // float ring_angle[LAYER_NUM];

  pcl::PointCloud<PointXYZIR> scan;
  fromROSMsg(*input, scan);
  std::cout << "Cloud size: " << input->data.size()/32 << " --> " << scan.size() << std::endl;
  // for(int i = 0, i_max = scan.size(); i < i_max; i+=1)
  // {
  //   if(scan.points[i].ring == 0)
  //     std::cout << scan.points[i] << std::endl;
  // }
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(scan, *map_msg_ptr);
  current_scan_pub.publish(*map_msg_ptr);
  r->sleep();

  // for(int i = 0, i_max = input->data.size(); i < i_max; i += 32)
  // {
  //   PointXYZIR new_point(&input->data[i]);
  //   if(ring_check[new_point.ring] == false)
  //   {
  //     ring_angle[new_point.ring] = new_point.yaw();
  //     ring_check[new_point.ring] = true;
  //   }
  //   else // ring_check[new_point.ring] == true
  //   {
  //     // print all angle values to file
  //     for(int j = 0; j < LAYER_NUM; j++)
  //     {
  //       if(ring_check[j] == true)
  //         csv_stream << ring_angle[j] << ",";
  //       else 
  //         csv_stream << "nan,";
  //     }
  //     csv_stream << std::endl;

  //     // reset arrays
  //     for(int j = 0; j < LAYER_NUM; j++)
  //       ring_check[j] = false;
      
  //     // and put the newpoint's value in
  //     ring_angle[new_point.ring] = new_point.yaw();
  //     ring_check[new_point.ring] = true;
  //   }
  // }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_preprocessing");

  ros::NodeHandle nh;
  r = new ros::Rate(1);
  // ROS node
  current_scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/current_scan", 1, true);

  // Open bagfile
  if(argc < 2)
  {
    std::cout << "Please indicate bag file to process." << std::endl;
    return -1;
  }
  std::cout << "Loading " << argv[1] << std::endl;
  rosbag::Bag bag(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
  reading_topics.push_back(std::string("/points_raw"));
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics));

  // Looping, processing messages in bag file
  // int count = 0;
  foreach(rosbag::MessageInstance const message, view)
  {
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL)
    {
      std::cout << "No input PointCloud2 available. Waiting..." << std::endl;
      continue;
    }
    ndt_mapping_callback(input_cloud);
    // count++;
    // if(count > 1)
    // {
    //   std::cout << "Stopping since more than " << count << " messages are processed." << std::endl; 
    //   return 0;
    // }
  }
  bag.close();
  std::cout << "Finished processing bag file." << std::endl;

  return 0;
}