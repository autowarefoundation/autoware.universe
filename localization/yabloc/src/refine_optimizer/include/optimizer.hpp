#pragma once
#include "config.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace validation
{

Sophus::SE3f refinePose(
  const Sophus::SE3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Sophus::SE3f & pose, pcl::PointCloud<pcl::PointXYZ> & samples, const RefineConfig & config,
  std::string * summary_text);

}  // namespace validation