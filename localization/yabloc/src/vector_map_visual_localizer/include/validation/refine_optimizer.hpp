#pragma once
#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace validation
{
struct RefineConfig
{
  RefineConfig(rclcpp::Node * node);

  RefineConfig(bool verbose = false, int max_iteration = 50, double euler_bound = 0.1)
  : verbose_(verbose), max_iteration_(max_iteration), euler_bound_(euler_bound)
  {
    long_bound_ = 0.1;
    late_bound_ = 1.0;
    height_bound_ = 0.1;
  }
  bool verbose_;
  int max_iteration_;
  double euler_bound_;
  double long_bound_;
  double late_bound_;
  double height_bound_;
};

Sophus::SE3f refinePose(
  const Sophus::SE3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Sophus::SE3f & pose, pcl::PointCloud<pcl::PointXYZ> & samples, const RefineConfig & config,
  std::string * summary_text);

}  // namespace validation