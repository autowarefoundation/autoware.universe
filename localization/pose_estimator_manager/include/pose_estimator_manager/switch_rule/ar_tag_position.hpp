#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace multi_pose_estimator
{
// This class finds the distance to the nearest landmark for AR tag based localization.
class ArTagPosition
{
public:
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  ArTagPosition(rclcpp::Node * node);

  double distance_to_nearest_ar_tag_around_ego(const geometry_msgs::msg::Point & ego_point) const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  rclcpp::Logger logger_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::optional<TransformStamped> get_transform(
    const std::string & target_frame, const std::string & source_frame) const;
};
}  // namespace multi_pose_estimator