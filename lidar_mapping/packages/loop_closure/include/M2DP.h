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

#include <Eigen/Dense>
#include <Eigen/SVD>

const double pi = 3.14159265358979323846;

double cart2pol_theta(double y, double x)
{
  return std::atan2(y, x); // range: (-pi, pi]
}

double cart2pol_rho(double y, double x)
{
  return sqrt(x * x + y * y);
}

float re_invert(float x)
{
  return -x;
}

bool getM2DP(const pcl::PointCloud<pcl::PointXYZI> input_cloud, Eigen::VectorXd &m2dp)
{
  if (input_cloud.size() == 0)
  {
    std::cout << "ERROR: Empty pointcloud!" << std::endl;
    return (false);
  }

  // Key parameters
  const int numP = 4;  // number of azimuth angles
  const int numQ = 16; // number of elevation angles
  const int numT = 16; // number of bins in theta (t)
  const int numR = 8;  // number of bins in rho (l), or number of cocentric circles

  // Do PCA, demean and re-align x/y-axis
  pcl::PCA<pcl::PointXYZI> pca(false);
  pca.setInputCloud(input_cloud.makeShared());

  Eigen::MatrixXd processed_cloud;
  pcl::demeanPointCloud(input_cloud, pca.getMean().cast<double>(), processed_cloud);

  processed_cloud.transposeInPlace();
  Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
  if (eigenVectors.col(0).array().sum() < 0)
    eigenVectors.col(0) = eigenVectors.col(0).unaryExpr(&re_invert);
  if (eigenVectors.col(1).array().sum() < 0)
    eigenVectors.col(1) = eigenVectors.col(1).unaryExpr(&re_invert);
  if (eigenVectors.col(2).array().sum() < 0)
    eigenVectors.col(2) = eigenVectors.col(2).unaryExpr(&re_invert);
  Eigen::MatrixXd X_values = eigenVectors.cast<double>().col(0).transpose() * processed_cloud.leftCols(3).transpose();
  Eigen::MatrixXd Y_values = eigenVectors.cast<double>().col(1).transpose() * processed_cloud.leftCols(3).transpose();
  Eigen::MatrixXd Z_values = eigenVectors.cast<double>().col(2).transpose() * processed_cloud.leftCols(3).transpose();
  processed_cloud.col(0) = X_values.transpose();
  processed_cloud.col(1) = Y_values.transpose();
  processed_cloud.col(2) = Z_values.transpose();

  // Get distance to farthest point
  double max_rho = sqrt(processed_cloud.array().square().rowwise().sum().maxCoeff());

  // Generate signature matrix A
  Eigen::ArrayXd azimuth_list = Eigen::ArrayXd::LinSpaced(numP, -pi / 2, pi / 2);
  Eigen::ArrayXd elevation_list = Eigen::ArrayXd::LinSpaced(numQ, 0, pi / 2);
  const double delta_theta = 2 * pi / numT;                    // angle of one theta-bin
  const double base_radius = (max_rho + 1e-4) / (numR * numR); // radius of the smallest circle
  Eigen::MatrixXd signature_matrix = Eigen::MatrixXd::Zero(numP * numQ, numT * numR);
  for (int i = 0, i_end = azimuth_list.size(); i < i_end; i++)
  {
    double azimuth = azimuth_list[i];
    for (int j = 0, j_end = elevation_list.size(); j < j_end; j++)
    {
      double elevation = elevation_list[j];

      // Normal vector of selected plane
      Eigen::Vector3d p_norm_vec(cos(elevation) * cos(azimuth),
                                 cos(elevation) * sin(azimuth),
                                 sin(elevation));

      // Projection of x-axis onto this plane, the the corresponding y-axis
      Eigen::Vector3d o_x = Eigen::Vector3d::UnitX();
      Eigen::Vector3d p_x = o_x - (o_x.transpose() * p_norm_vec) * p_norm_vec;
      Eigen::Vector3d p_y = p_norm_vec.cross(p_x);

      // Point-projections onto plane
      Eigen::MatrixXd p_data_xy(input_cloud.size(), 2);
      p_data_xy.col(0) = processed_cloud.leftCols(3) * p_x.matrix(); // x
      p_data_xy.col(1) = processed_cloud.leftCols(3) * p_y.matrix(); // y

      // Map data into polar coordinates
      Eigen::MatrixXd p_data_pol(input_cloud.size(), 2);
      p_data_pol.col(0) = p_data_xy.col(1).binaryExpr(p_data_xy.col(0), std::ptr_fun(cart2pol_theta)); // theta
      p_data_pol.col(1) = p_data_xy.col(1).binaryExpr(p_data_xy.col(0), std::ptr_fun(cart2pol_rho));

      // Count points in bins
      Eigen::VectorXd hist_bin = Eigen::VectorXd::Zero(numT * numR);
      for (int n = 0, n_end = input_cloud.size(); n < n_end; n++)
      {
        int idx_theta = std::ceil((p_data_pol(n, 0) + pi) / delta_theta) - 1; // ceil() instead of floor() to include theta=pi at bin 0
        int idx_rho = std::floor(std::sqrt(p_data_pol(n, 1) / base_radius));

        hist_bin[idx_rho * numT + (idx_theta >= 0 ? idx_theta : 0)]++;
      }
      hist_bin /= input_cloud.size();

      // Add this bin to signature matrix
      signature_matrix.row(i * numQ + j) = hist_bin.transpose();
    }
  }

  // run SVD on signature_matrix and use[u1, v1] as the final output
  Eigen::BDCSVD<Eigen::MatrixXd> svd(signature_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  m2dp = Eigen::VectorXd(numP * numQ + numT * numR); // shape: (192, 1)
  m2dp << svd.matrixU().col(0), svd.matrixV().col(0);
  return (true);
}

bool getM2DP(const pcl::PointCloud<pcl::PointXYZI> input_cloud, Eigen::MatrixXd &m2dp)
{
  Eigen::VectorXd m2dp_vec;
  bool ret = getM2DP(input_cloud, m2dp_vec);
  m2dp = m2dp_vec.matrix();
  return ret;
}