#pragma once
#include <eigen3/Eigen/Geometry>
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace vml_common
{
Eigen::Affine3f pose2Affine(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose affine2Pose(const Eigen::Affine3f & affine);

Sophus::SE3f pose2Se3(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose se32Pose(const Sophus::SE3f & se3f);
}  // namespace vml_common