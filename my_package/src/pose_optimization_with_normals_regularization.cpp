#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "my_package/pslPoseGraph.h"

using namespace std;
using namespace ros;

#define OUTPUT_POSE // To write x, y, z into a txt file

void optimizeEssentialGraph(const VectorofPoses &NonCorrectedSim3,
                            const VectorofNormalVectors &GroundNormalVector3,
                            Pose3d endCorrectedPose,
                            VectorofPoses &CorrectedSim3)
{
  VectorofPoses poses;
  poses.resize(NonCorrectedSim3.size());
  VectorOfConstraints constraints;
  poses.clear();
  constraints.clear();
  
  // Set KeyFrame poses (vertex)
  // We assume the start frame and end frame are the two ends of the loop
  int endID = NonCorrectedSim3.size()-1;
  
  for(size_t i=0, iend=NonCorrectedSim3.size(); i<iend;i++)
    poses[i] = NonCorrectedSim3[i];

  // edges (constraint just between two neighboring scans)
  for(size_t i=0, iend=NonCorrectedSim3.size()-1; i<iend;i++)
  {
    const Pose3d Swi = poses[i];
    const Pose3d Swj = poses[i+1];
    const Pose3d Sjw = Swj.inverse();
    const Pose3d Sji = Sjw * Swi;

    Constraint3d constraint(i, i+1, Sji);
      
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
    const Eigen::Matrix<double, 6, 6> sqrt_information = constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

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
  options.max_num_iterations = 50;  //can try more iterations if not converge
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << '\n';

  CorrectedSim3.clear();
  for(size_t i = 0, iend = NonCorrectedSim3.size(); i < iend; i++)
    CorrectedSim3.push_back(poses[i]);
}

int main(int argc, char** argv)
{
  // Initiate node
  ros::init(argc, argv, "pose_optimization_norm_regu");

  // ros::NodeHandle nh;
  // ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("optimized_graph", 10, true);

  // Do processing
  // Working dir
  pcl::PointCloud<pcl::PointXYZI> map; // full map holder
  map.header.frame_id = "map";
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

  getline(pose_stream, pose_line); // to skip header line of csv
  getline(normvec_stream, normvec_line); // to skip header line of csv

  // Get data from csv and pcd
  while(getline(pose_stream, pose_line) && getline(normvec_stream, normvec_line))
  {
    std::stringstream pose_line_stream(pose_line);
    std::stringstream normvec_line_stream(normvec_line);

    // Get identity data first to validate
    getline(pose_line_stream, key_str, ',');
    int key = std::stoi(key_str);
    
    getline(pose_line_stream, seq_str, ',');
    int sequence_pose = std::stoi(seq_str);

    getline(normvec_line_stream, seqnv_str, ',');
    int sequence_normvec = std::stoi(seqnv_str);
    if(sequence_pose != sequence_normvec)
    {
      std::cout << "ERROR: sequence_pose != sequence_normvec (" << sequence_pose << " != " << sequence_normvec << ")" << std::endl;
      std::cout << "Please ensure that the same bagfile was for the files." << std::endl;
      return(-1);
    }

    getline(pose_line_stream, sec_str, ',');
    getline(pose_line_stream, nsec_str, ','); // Unused
    getline(normvec_line_stream, secnv_str, ',');
    getline(normvec_line_stream, nsecnv_str, ','); // Unused
    unsigned int sec_pose = std::stoi(sec_str);
    unsigned int sec_normvec = std::stoi(secnv_str);
    if(sec_pose != sec_normvec)
    {
      std::cout << "ERROR: sec_pose != sec_normvec (" << sec_pose << " != " << sec_normvec << ")" << std::endl;
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

    // Get ground surface normal vector in global frame
    Eigen::Vector3d normvec_local(std::stod(xnv_str), std::stod(ynv_str), std::stod(znv_str));
    Eigen::Vector3d normvec_global = scan_q.toRotationMatrix() * normvec_local;
    vector_of_normal_vectors.push_back(normvec_global);

    // Show output
    std::cout << "Importing scan sequence " << seq_str << " at " << sec_str << "." << nsec_str << yaw << std::endl;
    std::cout << "[n] = [" << normvec_global[0] << "," << normvec_global[1] << "," << normvec_global[2] << "]" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
  }
  std::cout << "Finished importing pose data." << std::endl;
  if(non_corrected_sim3.size() != vector_of_normal_vectors.size())
  {
    std::cout << "ERROR: non_corrected_sim3.size() != vector_of_normal_vectors.size() (" 
              << non_corrected_sim3.size() << " != " << vector_of_normal_vectors.size() << ")" << std::endl;
    return(-1);
  }

  // Ground-truth pose for the final scan
  Eigen::Vector3d final_scan_p(-74.2269, 14.2863, 3.40642);
  tf::Quaternion final_q;
  final_q.setRPY(0.0418069, -0.0479844, 1.64459);
  Eigen::Quaterniond final_scan_q(final_q.w(), final_q.x(), final_q.y(), final_q.z());
  Pose3d end_corrected_pose(final_scan_p, final_scan_q);

  // Pose Graph Optimize!
  optimizeEssentialGraph(non_corrected_sim3,
                         vector_of_normal_vectors,
                         end_corrected_pose,
                         corrected_sim3);
  
  // Re-map with optimized pose
  #ifdef OUTPUT_POSE  // Also, output pose.txt 
  std::ofstream outp;
  std::string out_file = "optimized_pose.csv";
  outp.open(out_file);
  outp << "key,seq,sec,nsec,x,y,z,roll,pitch,yaw, nx, ny, nz" << std::endl;
  #endif // OUTPUT_POSE

//   if(all_scans.size() != corrected_sim3.size())
//   {
//     std::cout << "Error: number of pointclouds and poses do not match!" << std::endl;
//     std::cout << "Pointclouds: " << all_scans.size() << std::endl;
//     std::cout << "Poses: " << corrected_sim3.size() << std::endl;
//     return -1;
//   }

//   std::cout << "Re-mapping... " << std::endl;
//   for(size_t i = 0, iend = corrected_sim3.size(); i < iend; i++)
//   {
//     // Get transform
//     double x = corrected_sim3[i].p.x();
//     double y = corrected_sim3[i].p.y();
//     double z = corrected_sim3[i].p.z();
//     tf::Quaternion q(corrected_sim3[i].q.x(),
//                       corrected_sim3[i].q.y(),
//                       corrected_sim3[i].q.z(),
//                       corrected_sim3[i].q.w());
//     double roll, pitch, yaw;
//     tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//     Eigen::Affine3f global_transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

//     // Do transform
//     pcl::PointCloud<pcl::PointXYZI> dst;
//     pcl::transformPointCloud(all_scans[i], dst, global_transform);

//     // Add tf-ed scan to map
//     map += dst;

// #ifdef OUTPUT_POSE // to .txt
//     outp << x << "," << y << "," << z << "," << roll << "," << pitch << "," << yaw << std::endl;
// #endif // OUTPUT_POSE
//   }

//   std::cout << "Re-mapping finished. Total map size: " << map.size() << " points." << std::endl;
//   // Writing Point Cloud data to PCD file
//   std::string out_filename = "optimized_graph";
//   pcl::io::savePCDFileBinary(out_filename + ".pcd", map);
//   std::cout << "Finished. Saved pointcloud to " << out_filename << ".pcd" << std::endl;

// #ifdef OUTPUT_POSE
//   std::cout << "optimized_poses.csv written into the current directory." << std::endl;
// #endif

  return(0);
}