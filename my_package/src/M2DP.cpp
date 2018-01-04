// Reference: https://github.com/LiHeUA/M2DP/blob/master/M2DP.m
#include <iostream>
#include <string>
#include <cmath>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
// #include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/SVD>

const double pi = 3.14159265358979323846;

double remapped_atan2(double y, double x)
{
  double angle = std::atan2(y, x);
  return (angle > 0 ? angle : angle + 2 * pi);
}



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

  // Key parameters
  const int numP = 4; // number of azimuth angles
  const int numQ = 16; // number of elevation angles
  const int numT = 16; // number of bins in theta (t)
  const int numR = 8; // number of bins in rho (l), or number of cocentric circles

  // Do PCA, demean and re-align x/y-axis
  pcl::PCA<pcl::PointXYZI> pca(false);
  pca.setInputCloud(input_cloud.makeShared());
  Eigen::MatrixXd processed_cloud;
  pcl::demeanPointCloud(input_cloud, pca.getMean().cast<double>(), processed_cloud);
  processed_cloud.transposeInPlace();
  Eigen::MatrixXd X_values = pca.getEigenVectors().cast<double>().col(0).transpose() * processed_cloud.leftCols(3).transpose();
  Eigen::MatrixXd Y_values = pca.getEigenVectors().cast<double>().col(1).transpose() * processed_cloud.leftCols(3).transpose();
  Eigen::MatrixXd Z_values = pca.getEigenVectors().cast<double>().col(2).transpose() * processed_cloud.leftCols(3).transpose();
  processed_cloud.col(0) = X_values.transpose();
  processed_cloud.col(1) = Y_values.transpose();
  processed_cloud.col(2) = Z_values.transpose();

  // Get distance to farthest point
  // Eigen::MatrixXd rho2 = processed_cloud.array().square().rowwise  ().sum();
  // double maxRho = sqrt(rho2.maxCoeff());
  double max_rho = sqrt(processed_cloud.array().square().rowwise().sum().maxCoeff());
  
  // Generate signature matrix A
  Eigen::ArrayXd azimuth_list = Eigen::ArrayXd::LinSpaced(numP, -pi/2, pi/2);
  Eigen::ArrayXd elevation_list = Eigen::ArrayXd::LinSpaced(numQ, 0, pi/2);
  const double delta_theta = 2 * pi / numT; // angle of one theta-bin
  const double base_radius = max_rho / numR; // radius of the smallest circle
  Eigen::MatrixXd signature_matrix = Eigen::MatrixXd::Zero(numP * numQ, numT * numR);
  for(int i = 0, i_end = azimuth_list.size(); i < i_end; i++)
  {
    double azimuth = azimuth_list[i];
    for(int j = 0, j_end = elevation_list.size(); j < j_end; j++)
    {
      double elevation = elevation_list[j];

      // Normal vector of selected plane
      Eigen::Vector3d p_norm_vec(cos(elevation) * cos(azimuth),
                                 cos(elevation) * sin(azimuth),
                                 sin(elevation));

      // Projection of x-axis onto this plane, the the corresponding y-axis
      Eigen::Vector3d o_x = Eigen::Vector3d::UnitX();
      Eigen::Vector3d p_x = (o_x - (o_x.transpose().dot(p_norm_vec)) * p_norm_vec).normalized();
      Eigen::Vector3d p_y = p_norm_vec.cross(p_x);

      // Point-projections onto plane
      Eigen::MatrixXd p_data_xy(input_cloud.size(), 2);
      p_data_xy.col(0) = processed_cloud.leftCols(3) * p_x.matrix(); // x
      p_data_xy.col(1) = processed_cloud.leftCols(3) * p_y.matrix(); // y

      // Map data into polar coordinates
      Eigen::MatrixXd p_data_pol(input_cloud.size(), 2);
      p_data_pol.col(0) = p_data_xy.col(1).binaryExpr(p_data_xy.col(0), std::ptr_fun(remapped_atan2)); // theta
      p_data_pol.col(1) = p_data_xy.array().square().rowwise().sum().array().sqrt(); // rho

      // Count points in bins
      Eigen::VectorXd hist_bin = Eigen::VectorXd::Zero(numT * numR);
      for(int n = 0, n_end = input_cloud.size(); n < n_end; n++)
      {
        int idx_theta = std::ceil(p_data_pol(n, 0) / delta_theta) - 1;
        int idx_rho = std::ceil(std::sqrt(p_data_pol(n, 1) / base_radius)) - 1;

        hist_bin[(idx_rho >= 0 ? idx_rho : 0) * numT + (idx_theta >= 0 ? idx_theta : 0)]++;
      }
      hist_bin /= input_cloud.size();
      std::cout << hist_bin << std::endl;
      
      // Add this bin to signature matrix
      signature_matrix.row(i * numQ + j) = hist_bin.transpose();
    }
  }

  // run SVD on signature_matrix and use[u1, v1] as the final output
  Eigen::BDCSVD<Eigen::MatrixXd> svd(signature_matrix);

  // // Some test
  // pcl::PointCloud<pcl::PointXYZI> new_cloud;
  // for(int i = 0, i_end = processed_cloud.cols(); i != i_end; i++)
  // {
  //   pcl::PointXYZI new_point;
  //   new_point.x = processed_cloud(0, i);
  //   new_point.y = processed_cloud(1, i);
  //   new_point.z = processed_cloud(2, i);
  //   new_point.intensity = processed_cloud(3, i);
  //   new_cloud.push_back(new_point);
  // }

  // pca.setInputCloud(new_cloud.makeShared());
  // // std::cout << pca.getEigenVectors() << std::endl;
  // Eigen::Vector4d cloud_centroid; // (x, y, z, 1)
  // if(!pcl::compute3DCentroid(new_cloud, cloud_centroid))
  // {
  //   std::cout << "Failed to compute cloud centroid." << std::endl;
  //   return(-1);
  // }
  // // std::cout << cloud_centroid << std::endl;
  // // pcl::io::savePCDFileBinary("processed_" + f_pcd, processed_cloud);
  return(0);
}