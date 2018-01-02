#include <iostream>
#include <string>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    std::cout << "Error: missing input file argument." << std::endl;
    return(-1);
  }

  // Get input
  std::string f_pcd = argv[1];
  pcl::PointCloud<pcl::PointXYZI> input_cloud, processed_cloud;
  if(pcl::io::loadPCDFile<pcl::PointXYZI>(f_pcd, input_cloud) == -1)
  {
    std::cout << "Couldn't read pcd " << f_pcd << "." << std::endl;
    return(-1);
  }
  std::cout << "Loaded " << input_cloud.size() << " data points from " << f_pcd << std::endl;

  // Demean cloud
  Eigen::Vector4d cloud_centroid; // (x, y, z, 1)
  if(!pcl::compute3DCentroid(input_cloud, cloud_centroid))
  {
    std::cout << "Failed to compute cloud centroid." << std::endl;
    return(-1);
  }
  pcl::demeanPointCloud(input_cloud, cloud_centroid, processed_cloud);

  // Do PCA and re-alignment
  pcl::PCA<pcl::PointXYZI> pca(false);
  pca.setInputCloud(processed_cloud.makeShared());
  
  
  std::cout << pca.getEigenVectors() << std::endl;


  return(0);
}