#pragma once
#include <eigen3/Eigen/Geometry>
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace pcdless::common
{
Eigen::Affine3f pose_to_affine(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose affine_to_pose(const Eigen::Affine3f & affine);

Sophus::SE3f pose_to_se3(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose se3_to_pose(const Sophus::SE3f & se3f);
}  // namespace pcdless::common