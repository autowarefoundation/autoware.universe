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
  }
  bool verbose_;
  int max_iteration_;
  double euler_bound_;
};

Sophus::SE3f refinePose(
  const Sophus::SE3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Sophus::SE3f & pose, pcl::PointCloud<pcl::PointNormal> & linesegments,
  const RefineConfig & config, std::string * summary_text);

}  // namespace validation