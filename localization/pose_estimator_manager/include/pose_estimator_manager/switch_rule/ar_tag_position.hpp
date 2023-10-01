#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace multi_pose_estimator
{
class ArTagPosition
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  ArTagPosition(rclcpp::Node * node);

  double distance_to_nearest_ar_tag_around_ego(const geometry_msgs::msg::Point & ego_point) const;

  std::string debug_string() const;

  MarkerArray debug_marker_array() const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  rclcpp::Logger logger_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::optional<ArTagPosition::TransformStamped> get_transform(
    const std::string & target_frame, const std::string & source_frame) const;
};
}  // namespace multi_pose_estimator