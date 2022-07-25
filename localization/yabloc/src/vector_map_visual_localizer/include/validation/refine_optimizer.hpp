#pragma once
#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace validation
{
Eigen::Affine3f refinePose(
  const Eigen::Affine3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Eigen::Affine3f & pose, pcl::PointCloud<pcl::PointNormal> & linesegments);

}  // namespace validation