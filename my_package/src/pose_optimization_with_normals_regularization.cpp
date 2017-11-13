#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <boost/foreach.hpp> // to read bag file
#define foreach BOOST_FOREACH

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "my_package/pslPoseGraphL2.h"

using namespace std;
using namespace ros;

#define OUTPUT_POSE // To write x, y, z into a txt file

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

bool optimizeEssentialGraphWithL2(const VectorofPoses &NonCorrectedSim3,
                                  const VectorofNormalVectors &GroundNormalVector3,
                                  const double regularization_strength,
                                  Pose3d endCorrectedPose,
                                  VectorofPoses &CorrectedSim3)
{
  if(NonCorrectedSim3.size() != GroundNormalVector3.size())
  {
    std::cout << "Error: NonCorrectedSim3.size() != GroundNormalVector3.size()\n(";
    std::cout << NonCorrectedSim3.size() << " != " << GroundNormalVector3.size() << std::endl;
    return false;
  }

  VectorofPoses poses;
  poses.resize(NonCorrectedSim3.size());
  poses.clear();
  VectorOfConstraints constraints;
  constraints.clear();

  // Set KeyFrame poses (vertex)
  // We assume the start frame and end frame are the two ends of the loop
  int endID = NonCorrectedSim3.size()-1;
  
  for(size_t i = 0, iend = NonCorrectedSim3.size(); i < iend; i++)
  {
    poses[i] = NonCorrectedSim3[i]; // basically, poses = copy(NonCorrectedSim3)
  }

  // edges (constraint just between two neighboring scans)
  for(size_t i = 0, iend = NonCorrectedSim3.size() - 1; i < iend; i++)
  {
    const Pose3d Swi = poses[i];
    const Pose3d Swj = poses[i+1];
    const Pose3d Sjw = Swj.inverse();
    const Pose3d Sji = Sjw * Swi;

    Constraint3d constraint(i, i+1, Sji, GroundNormalVector3[i], GroundNormalVector3[i+1]);
      
    constraints.push_back(constraint);
  }
  
  // modify the pose of last keyframe to the corrected one
  poses[endID] = endCorrectedPose;

  ceres::LossFunction* loss_function = NULL;
  ceres::LocalParameterization *quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
  ceres::Problem problem;

  for (VectorOfConstraints::const_iterator constraints_iter = constraints.begin(); constraints_iter != constraints.end(); ++constraints_iter) 
  {
    const Constraint3d& constraint = *constraints_iter;
    // const Constraint3d& constraint = constraints[i];
    const Eigen::Matrix<double, 6, 6> sqrt_information = constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(constraint.t_be,
                                                                      sqrt_information,
                                                                      constraint.nvec_a_,
                                                                      constraint.nvec_b_,
                                                                      regularization_strength);

    // Add pose constraints
    problem.AddResidualBlock(cost_function, loss_function,
                             poses[constraint.id_begin].p.data(),
                             poses[constraint.id_begin].q.coeffs().data(),
                             poses[constraint.id_end].p.data(),
                             poses[constraint.id_end].q.coeffs().data());

    problem.SetParameterization(poses[constraint.id_begin].q.coeffs().data(),
                quaternion_local_parameterization);
    problem.SetParameterization(poses[constraint.id_end].q.coeffs().data(),
                quaternion_local_parameterization);
  }

  // set constant pose for start and end scans
  problem.SetParameterBlockConstant(poses[0].p.data());
  problem.SetParameterBlockConstant(poses[0].q.coeffs().data());
  problem.SetParameterBlockConstant(poses[endID].p.data());
  problem.SetParameterBlockConstant(poses[endID].q.coeffs().data());

  // optimize
  ceres::Solver::Options options;
  options.max_num_iterations = 50000;  //can try more iterations if not converge
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  // options.parameter_tolerance = 1e-15;
  // options.function_tolerance = 1e-15;
  // options.min_line_search_step_contraction = 0.99999999999;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << '\n';

  CorrectedSim3.clear();
  for(size_t i = 0, iend = NonCorrectedSim3.size(); i < iend; i++)
    CorrectedSim3.push_back(poses[i]);
  return true;
}

int main(int argc, char** argv)
{
  // Initiate node
  ros::init(argc, argv, "pose_optimization_norm_regu");

  double L2_strength = 1.0;
  if(argc >= 2)
  {
    L2_strength = std::stod(argv[1]);
  }
  std::cout << "lambda = " << L2_strength << std::endl;
  std::string bagfile = "/home/zwu/LIDAR-DATA/3oct-around-pana1.bag";
  std::string bagtopic = "/points_raw";
  std::cout << "Reading bag: " << bagfile << " [topic: " << bagtopic << "]... " << std::flush;
  rosbag::Bag bag(bagfile, rosbag::bagmode::Read);
  std::vector<std::string> reading_topics;
    reading_topics.push_back(std::string(bagtopic));
  rosbag::View view(bag, rosbag::TopicQuery(reading_topics));
  std::cout << "Done." << std::endl;

  // Do processing
  std::string pose_csv = "map_pose.csv";
  std::string normvec_csv = "normal_vector.csv";
  std::cout << "Processing " << pose_csv << " & " << normvec_csv << " in the current directory." << std::endl;
  std::cout << "Please note that first line of every csv is skipped as the header." << std::endl;
  std::ifstream pose_stream(pose_csv);
  std::ifstream normvec_stream(normvec_csv);

  // Place-holder for csv stream variables
  std::string pose_line, normvec_line;
  std::string key_str, seq_str, sec_str, nsec_str, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;
  std::string seqnv_str, secnv_str, nsecnv_str, xnv_str, ynv_str, znv_str;

  // Place-holder for input and output poses
  VectorofPoses non_corrected_sim3, corrected_sim3;
  VectorofNormalVectors vector_of_normal_vectors;
  std::vector< pcl::PointCloud<pcl::PointXYZI> > all_scans;
  std::vector<unsigned int> key_list, seq_list, sec_list, nsec_list;

  getline(pose_stream, pose_line); // to skip header line of csv
  getline(normvec_stream, normvec_line); // to skip header line of csv

  // Get data from bag and csv files
  foreach(rosbag::MessageInstance const message, view)
  {
    if(!getline(pose_stream, pose_line) || !getline(normvec_stream, normvec_line))
    {
      break;
    }

    std::stringstream pose_line_stream(pose_line);
    std::stringstream normvec_line_stream(normvec_line);

    // Get identity data first to validate
    getline(pose_line_stream, key_str, ',');
    unsigned int key = std::stoi(key_str);
    
    getline(pose_line_stream, seq_str, ',');
    unsigned int sequence_pose = std::stoi(seq_str);

    getline(normvec_line_stream, seqnv_str, ',');
    unsigned int sequence_normvec = std::stoi(seqnv_str);
    if(sequence_pose != sequence_normvec)
    {
      std::cout << "ERROR: sequence_pose != sequence_normvec (" << sequence_pose << " != " << sequence_normvec << ")" << std::endl;
      std::cout << "Please ensure that the same bagfile was for the files." << std::endl;
      return(-1);
    }

    getline(pose_line_stream, sec_str, ',');
    getline(pose_line_stream, nsec_str, ',');
    getline(normvec_line_stream, secnv_str, ',');
    getline(normvec_line_stream, nsecnv_str, ',');
    unsigned int sec_pose = std::stoi(sec_str);
    unsigned int nsec_pose = std::stoi(nsec_str);
    unsigned int sec_normvec = std::stoi(secnv_str);
    unsigned int nsec_normvec = std::stoi(nsecnv_str);
    if(sec_pose != sec_normvec)
    {
      std::cout << "ERROR: sec_pose != sec_normvec (" << sec_pose << " != " << sec_normvec << ")" << std::endl;
      std::cout << "Please ensure that the same bagfile was for the files." << std::endl;
      return(-1);
    }

    if(nsec_pose != nsec_normvec)
    {
      std::cout << "ERROR: nsec_pose != nsec_normvec (" << nsec_pose << " != " << nsec_normvec << ")" << std::endl;
      std::cout << "Please ensure that the same bagfile was for the files." << std::endl;
      return(-1);
    }

    if(key == 0)
    {
      // not an added scan to map, skip 
      continue;
    }
    
    // and the rest of the data
    getline(pose_line_stream, x_str, ',');
    double x = std::stod(x_str);
    getline(pose_line_stream, y_str, ',');
    double y = std::stod(y_str);
    getline(pose_line_stream, z_str, ',');
    double z = std::stod(z_str);
    getline(pose_line_stream, roll_str, ',');
    double roll = std::stod(roll_str);
    getline(pose_line_stream, pitch_str, ',');
    double pitch = std::stod(pitch_str);
    getline(pose_line_stream, yaw_str);
    double yaw = std::stod(yaw_str);

    getline(normvec_line_stream, xnv_str, ',');
    getline(normvec_line_stream, ynv_str, ',');
    getline(normvec_line_stream, znv_str);

    // Get translation and rotation -> Pose3d
    Eigen::Vector3d scan_p(x, y, z);
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    Eigen::Quaterniond scan_q(q.w(), q.x(), q.y(), q.z());
    Pose3d current_pose(scan_p, scan_q);
    non_corrected_sim3.push_back(current_pose);

    // Get ground surface normal vector -> Vector3d
    Eigen::Vector3d normvec_local(std::stod(xnv_str), std::stod(ynv_str), std::stod(znv_str));
    vector_of_normal_vectors.push_back(normvec_local);

    // And finally, the pointcloud
    sensor_msgs::PointCloud2::ConstPtr input_cloud = message.instantiate<sensor_msgs::PointCloud2>();
    if(input_cloud == NULL) // no data?
    {
      std::cout << "No input PointCloud available. Waiting..." << std::endl;
      continue;
    }
    else // check whether the sequence match
    {
      if(input_cloud->header.seq != sequence_pose)
      {
        std::cout << "Error: input_cloud->header.seq != sequence_pose (" << input_cloud->header.seq << " != " << sequence_pose << ")" << std::endl;
        return -1;
      }
    }
    
    // From ROS msg to PCL pcloud
    pcl::PointCloud<pcl::PointXYZI> scan;
    pcl::fromROSMsg(*input_cloud, scan);
    all_scans.push_back(scan);

    // Save meta data
    key_list.push_back(key);
    seq_list.push_back(sequence_pose);
    sec_list.push_back(sec_pose);
    nsec_list.push_back(nsec_pose);

    // Show output
    // std::cout << "Importing [key, sequence] [" << key << ", " << seq_str << "] at " << sec_str << "." << nsec_str << std::endl;
    // std::cout << "[n] = [" << normvec_local[0] << "," << normvec_local[1] << "," << normvec_local[2] << "]" << std::endl;
    // std::cout << "---------------------------------------" << std::endl;
  }
  std::cout << "Finished reading data." << std::endl;
  std::cout << "Pose: " << non_corrected_sim3.size() << "\n";
  std::cout << "Vector: " << vector_of_normal_vectors.size() << std::endl;
  std::cout << "Cloud: " << all_scans.size() << std::endl;
  if(non_corrected_sim3.size() != vector_of_normal_vectors.size() || non_corrected_sim3.size() != all_scans.size())
  {
    std::cout << "ERROR: Size of the data do not match!" << std::endl;
    return(-1);
  }

  if(!non_corrected_sim3.size() || !vector_of_normal_vectors.size() || !all_scans.size())
  {
    std::cout << "ERROR: Zero-sized data!" << std::endl;
    return(-1);
  }

  // Ground-truth pose for the final scan
  Eigen::Vector3d final_scan_p(-78.1281, -10.2903, 3.18067);
  tf::Quaternion final_q;
  final_q.setRPY(0.0547467, -0.0623987, 1.61432);
  Eigen::Quaterniond final_scan_q(final_q.w(), final_q.x(), final_q.y(), final_q.z());
  Pose3d end_corrected_pose(final_scan_p, final_scan_q);

  // Pose Graph Optimize!
  optimizeEssentialGraphWithL2(non_corrected_sim3,
                               vector_of_normal_vectors,
                               L2_strength,
                               end_corrected_pose,
                               corrected_sim3);
  
  // Re-map with optimized pose
  #ifdef OUTPUT_POSE  // Also, output pose.txt 
  std::ofstream out_stream;
  std::string out_file = "optimized_pose.csv";
  out_stream.open(out_file);
  out_stream << "key,sequence,sec,nsec,x,y,z,roll,pitch,yaw" << std::endl;
  #endif // OUTPUT_POSE

  std::cout << "Re-mapping... " << std::endl;
  pcl::PointCloud<pcl::PointXYZI> map; // full map holder
  map.header.frame_id = "map";
  Eigen::Affine3d prev_transform, crnt_transform, rel_transform;
  ros::Time prev_time;
  double interval;
  for(int i = 0, i_end = corrected_sim3.size(); i < i_end; i++)
  {
    // Get transform
    double x = corrected_sim3[i].p.x();
    double y = corrected_sim3[i].p.y();
    double z = corrected_sim3[i].p.z();
    tf::Quaternion q(corrected_sim3[i].q.x(),
                     corrected_sim3[i].q.y(),
                     corrected_sim3[i].q.z(),
                     corrected_sim3[i].q.w());
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pcl::getTransformation(x, y, z, roll, pitch, yaw, crnt_transform);

    // Correct scan
    ros::Time crnt_time(sec_list[i], nsec_list[i]);
    
    if(i == 0)
    {
      prev_transform = crnt_transform;
      interval = 0.1; // dummy value
      rel_transform = prev_transform.inverse() * crnt_transform;
    }
    else
    {
      interval = (crnt_time - prev_time).toSec();
      rel_transform = prev_transform.inverse() * crnt_transform;
    }

    pcl::PointCloud<pcl::PointXYZI> src = all_scans[i];
    correctLIDARscan(src, rel_transform, interval);

    // Do transform
    pcl::PointCloud<pcl::PointXYZI> dst;
    pcl::transformPointCloud(src, dst, crnt_transform);

    // Add tf-ed scan to map
    map += dst;

    #ifdef OUTPUT_POSE // to .txt
    out_stream << key_list[i] << "," << seq_list[i] << "," << sec_list[i] << "," << nsec_list[i] << ","
               << x << "," << y << "," << z << "," << roll << "," << pitch << "," << yaw << std::endl;
    #endif // OUTPUT_POSE

    // Update prev values
    prev_transform = crnt_transform;
    prev_time = crnt_time;
  }

  std::cout << "Re-mapping finished. Total map size: " << map.size() << " points." << std::endl;

  // Writing Point Cloud data to PCD file
  std::string out_filename = "optimizedL2_map";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", map);
  std::cout << "Finished. Saved pointcloud to " << out_filename << ".pcd" << std::endl;

  #ifdef OUTPUT_POSE
  std::cout << "optimized_poses.csv written into the current directory." << std::endl;
  #endif

  return(0);
}