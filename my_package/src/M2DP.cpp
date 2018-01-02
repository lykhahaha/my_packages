// Reference: https://github.com/LiHeUA/M2DP/blob/master/M2DP.m
#include <iostream>
#include <string>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    std::cout << "Error: missing input file argument." << std::endl;
    return(-1);
  }

  // Get input
  std::string f_pcd = argv[1];
  pcl::PointCloud<pcl::PointXYZI> input_cloud;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(f_pcd, input_cloud) == -1)
  {
    std::cout << "Couldn't read pcd " << f_pcd << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << input_cloud.size() << " data points from " << f_pcd << std::endl;

  // Do PCA, demean and re-align x/y-axis
  pcl::PCA<pcl::PointXYZI> pca(false);
  pca.setInputCloud(input_cloud.makeShared());

  Eigen::MatrixXd processed_cloud;
  pcl::demeanPointCloud(input_cloud, pca.getMean().cast<double>(), processed_cloud);
  
  Eigen::MatrixXd X_values = pca.getEigenVectors().cast<double>().col(0).transpose() * processed_cloud.topRows(3);
  Eigen::MatrixXd Y_values = pca.getEigenVectors().cast<double>().col(1).transpose() * processed_cloud.topRows(3);
  Eigen::MatrixXd Z_values = pca.getEigenVectors().cast<double>().col(2).transpose() * processed_cloud.topRows(3);
  processed_cloud.row(0) = X_values;
  processed_cloud.row(1) = Y_values;
  processed_cloud.row(2) = Z_values;


  // Some test
  pcl::PointCloud<pcl::PointXYZI> new_cloud;
  for(int i = 0, i_end = processed_cloud.cols(); i != i_end; i++)
  {
    pcl::PointXYZI new_point;
    new_point.x = processed_cloud(0, i);
    new_point.y = processed_cloud(1, i);
    new_point.z = processed_cloud(2, i);
    new_point.intensity = processed_cloud(3, i);
    new_cloud.push_back(new_point);
  }
  
  pca.setInputCloud(new_cloud.makeShared());
  std::cout << pca.getEigenVectors() << std::endl;
  Eigen::Vector4d cloud_centroid; // (x, y, z, 1)
  if(!pcl::compute3DCentroid(new_cloud, cloud_centroid))
  {
    std::cout << "Failed to compute cloud centroid." << std::endl;
    return(-1);
  }
  std::cout << cloud_centroid << std::endl;
  // pcl::io::savePCDFileBinary("processed_" + f_pcd, processed_cloud);
  return(0);
}