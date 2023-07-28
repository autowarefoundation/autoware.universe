#pragma once

#include "pose_estimator_manager/pose_estimator_name.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <unordered_map>

namespace multi_pose_estimator
{

class BaseSwitchRule
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  BaseSwitchRule(rclcpp::Node & node)
  : logger_ptr_(std::make_shared<rclcpp::Logger>(node.get_logger()))
  {
  }

  virtual ~BaseSwitchRule() = default;
  virtual std::unordered_map<PoseEstimatorName, bool> update() = 0;
  virtual std::vector<PoseEstimatorName> supporting_pose_estimators() = 0;

  virtual std::string debug_string() { return std::string{}; }
  virtual MarkerArray debug_marker_array() { return MarkerArray{}; }

protected:
  rclcpp::Logger get_logger() { return *logger_ptr_; }
  std::shared_ptr<rclcpp::Logger> logger_ptr_{nullptr};
};

}  // namespace multi_pose_estimator