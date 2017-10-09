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

#include "my_package/pslPoseGraph.h"

using namespace std;
using namespace ros;

#define OUTPUT_POSE // To write x, y, z into a txt file

void optimizeEssentialGraph(const VectorofPoses &NonCorrectedSim3, Pose3d endCorrectedPose,
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
  ros::init(argc, argv, "pose_optimization_mapping");

  // ros::NodeHandle nh;
  // ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("optimized_graph", 10, true);

  // Do processing
  // Working dir
  pcl::PointCloud<pcl::PointXYZI> map; // full map holder
  map.header.frame_id = "map";
	std::string csv = "map_pose.csv";
  std::cout << "Processing " << csv << " in the current directory" << std::endl;
	std::ifstream csv_stream(csv);

  // Place-holder for csv stream variables
	std::string line, pcd_filename, x_str, y_str, z_str, roll_str, pitch_str, yaw_str;

	// Place-holder for input and output poses
	VectorofPoses non_corrected_sim3, corrected_sim3;

	// Get data from csv and pcd
#ifdef OUTPUT_POSE
  std::ofstream inp;
  std::string in_file = "initial_pose@9oct-0854.csv";
  inp.open(in_file);
#endif // OUTPUT_POSE
	std::vector< pcl::PointCloud<pcl::PointXYZI> > all_scans;
  while(getline(csv_stream, line))
  {
  	pcl::PointCloud<pcl::PointXYZI> current_scan;
  	std::stringstream line_stream(line);

    // Get pcd file
    getline(line_stream, pcd_filename, ',');
    std::cout << "Loading file: "<<  pcd_filename << std::endl;
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, current_scan) == -1)
    {
      std::cout << "Couldn't read " << pcd_filename << "." << std::endl;
      return -1;
    }
    std::cout << "Loaded " << current_scan.size() << " data points from " << pcd_filename << std::endl;
    all_scans.push_back(current_scan);
    
    // and the rest of the data
    getline(line_stream, x_str, ',');
    double x = std::stod(x_str);
    getline(line_stream, y_str, ',');
    double y = std::stod(y_str);
    getline(line_stream, z_str, ',');
    double z = std::stod(z_str);
    getline(line_stream, roll_str, ',');
    double roll = std::stod(roll_str);
    getline(line_stream, pitch_str, ',');
    double pitch = std::stod(pitch_str);
    getline(line_stream, yaw_str);
    double yaw = std::stod(yaw_str);

    // Get translation and rotation
    Eigen::Vector3d scan_p(x, y, z);
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    Eigen::Quaterniond scan_q(q.w(), q.x(), q.y(), q.z());
    Pose3d current_pose(scan_p, scan_q);
    non_corrected_sim3.push_back(current_pose);

    // Show output
    std::cout << "Number of points in scan: " << current_scan.size() << std::endl;
    std::cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
    std::cout << "---------------------------------------" << std::endl;

#ifdef OUTPUT_POSE
    inp << x << "," << y << "," << z << std::endl;
#endif // OUTPUT_POSE
	}

	// Ground-truth pose for the final scan
	Eigen::Vector3d final_scan_p(-76.582, -0.7804, 3.58456);
	tf::Quaternion final_q;
	final_q.setRPY(0.0599305, -0.0247047, 1.6297);
	Eigen::Quaterniond final_scan_q(final_q.w(), final_q.x(), final_q.y(), final_q.z());
  Pose3d end_corrected_pose(final_scan_p, final_scan_q);

  // Pose Graph Optimize!
  optimizeEssentialGraph(non_corrected_sim3, end_corrected_pose, corrected_sim3);
  
  // Re-map with optimized pose
#ifdef OUTPUT_POSE  // Also, output pose.txt 
  std::ofstream outp;
  std::string out_file = "optimized_pose@9oct-0854.csv";
  outp.open(out_file);
  outp << 'x' << ',' << 'y' << ',' << 'z' << std::endl;
#endif // OUTPUT_POSE

  if(all_scans.size() != corrected_sim3.size())
  {
  	std::cout << "Error: number of pointclouds and poses do not match!" << std::endl;
  	std::cout << "Pointclouds: " << all_scans.size() << std::endl;
  	std::cout << "Poses: " << corrected_sim3.size() << std::endl;
  	return -1;
  }

  std::cout << "Re-mapping... " << std::endl;
  for(size_t i = 0, iend = corrected_sim3.size(); i < iend; i++)
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
  	Eigen::Affine3f global_transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);

  	// Do transform
  	pcl::PointCloud<pcl::PointXYZI> dst;
  	pcl::transformPointCloud(all_scans[i], dst, global_transform);

  	// Add tf-ed scan to map
  	map += dst;

#ifdef OUTPUT_POSE // to .txt
    outp << x << "," << y << "," << z << std::endl;
#endif // OUTPUT_POSE
  }

  std::cout << "Re-mapping finished. Total map size: " << map.size() << " points." << std::endl;
  // Writing Point Cloud data to PCD file
  std::string out_filename = "optimized_graph";
  pcl::io::savePCDFileBinary(out_filename + ".pcd", map);
  std::cout << "Finished. Saved pointcloud to " << out_filename << ".pcd" << std::endl;
#ifdef OUTPUT_POSE
  std::cout << "initial_pose.csv and optimized_poses.csv written into the current directory." << std::endl;
#endif
  // Publish to topic to view in rviz
  // sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  // pcl::toROSMsg(map, *map_msg_ptr);
  // map_pub.publish(*map_msg_ptr);

  // ros::spin();
  return 0;
}
