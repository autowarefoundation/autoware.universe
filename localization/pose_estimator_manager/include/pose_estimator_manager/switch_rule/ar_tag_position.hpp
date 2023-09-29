#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace multi_pose_estimator
{
class ArTagPosition
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  ArTagPosition(rclcpp::Node * node);

  bool exist_ar_tag_around_ego(const geometry_msgs::msg::Point & point) const;

  std::string debug_string() const;

  MarkerArray debug_marker_array() const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  rclcpp::Logger logger_;
};
}  // namespace multi_pose_estimator