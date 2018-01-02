#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/time.h>
#include <ros/duration.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

#include <pcl/common/common.h>


int main(int argc, char** argv)
{
  if(argc != 4)
  {
    std::cout << "ERROR: false number of args" << std::endl;
    std::cout << "!!! Usage: rosrun my_package ndt_to_imu INPUT_BAG INPUT_CSV_TRAJECTORY OUTPUT_BAG" << std::endl;
    return(-1);
  }

  // Get input
  std::string f_inbag = argv[1];
  std::string f_incsv = argv[2];
  std::string f_outbag = argv[3];

  // Read input bag
  std::string bag_topic = "/points_raw";
  std::cout << "Reading bag: " << f_inbag << " [topic: " << bag_topic << "]... " << std::flush;
  rosbag::Bag inbag(f_inbag, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string(bag_topic));
  rosbag::View view(inbag, rosbag::TopicQuery(reading_topics));
  std::cout << "Done." << std::endl;

  // Preps for input NDT-localized trajectory
  std::cout << "Reading " << f_incsv << "\n";
  std::cout << "csv file format assumption:" << std::endl;
  std::cout << "\t{key, sequence, sec, nsec, x, y, z, roll, pitch, yaw}" << std::endl;
  std::ifstream csv_stream(f_incsv);

  // Preps for output bag
  rosbag::Bag outbag;
  outbag.open(f_outbag, rosbag::bagmode::Write);

  // Place-holders for variables
  std::string line, key_str, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  ros::Time crnt_time;
  unsigned int seq = 0;
  unsigned int key = 0;
  double x = 0, y = 0, z = 0;
  double roll = 0, pitch = 0, yaw = 0;
  bool isGetLine = false;

  getline(csv_stream, line); // to skip header line in the csv
  
  // Iterate through the msgs in the input bag
  foreach(rosbag::MessageInstance const message, view)
  {
    // get new data and process values
    if(!isGetLine)
    {  
      if(!getline(csv_stream, line))
      {
        std::cout << "Done processing (End of csv)" << std::endl;
        break; // end of csv
      }
      std::stringstream line_stream(line);

      getline(line_stream, key_str, ',');
      key = std::stod(key_str);
      getline(line_stream, seq_str, ',');
      seq = std::stod(seq_str);
      getline(line_stream, sec_str, ',');
      getline(line_stream, nsec_str, ',');
      crnt_time = ros::Time(std::stod(sec_str), std::stod(nsec_str));
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

      isGetLine = true;
    }

    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL) // no data?
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }
    else // check sequence & time match, else skip or end
    {
      if(input_cloud->header.seq < seq)
      {
        std::cout << "INFO: input_cloud->header.seq < seq (" << input_cloud->header.seq << " < " << seq << "). Skipping cloud msg." << std::endl;
        continue;
      }
      else if(input_cloud->header.seq > seq)
      {
        std::cout << "ERROR: input_cloud->header.seq > seq (" << input_cloud->header.seq << " > " << seq << ")." << std::endl;
        std::cout << "Are you giving the correct input bag file?" << std::endl;
        return(-1);
      }

      if(input_cloud->header.stamp != crnt_time)
      {
        std::cout << "ERROR: input_cloud->header.stamp != crnt_time" << std::endl;
        std::cout << "(" << input_cloud->header.stamp << " != " << crnt_time << ")" << std::endl;
        std::cout << "Are you giving the correct input bag file?" << std::endl;
        return(-1);
      }
    }

    // Write point cloud msg -> /velodyne_points
    outbag.write("/velodyne_points", crnt_time, *input_cloud);

    // Convert pose to tf then write msg -> /tf
    geometry_msgs::TransformStamped imu2world_tf;
    imu2world_tf.header.seq = seq;
    imu2world_tf.header.stamp = crnt_time;
    imu2world_tf.header.frame_id = "world";
    imu2world_tf.child_frame_id = "velodyne";

    imu2world_tf.transform.translation.x = x;
    imu2world_tf.transform.translation.y = y;
    imu2world_tf.transform.translation.z = z;
    imu2world_tf.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    tf::tfMessage tf_msg;
    tf_msg.transforms.push_back(imu2world_tf);
    // tf_msg.transforms[0] = imu2world_tf;
    outbag.write("/tf", crnt_time, tf_msg);

    // Update
    isGetLine = false;
  }

  // Finished
  inbag.close();
  outbag.close();
  std::cout << "Finished. Output: " << f_outbag << std::endl;

  return(0);
}