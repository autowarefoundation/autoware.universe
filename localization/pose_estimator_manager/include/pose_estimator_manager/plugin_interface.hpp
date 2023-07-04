#pragma once

#include "pose_estimator_manager/pose_estimator_name.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <unordered_map>

namespace multi_pose_estimator
{

class PluginInterface
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  virtual ~PluginInterface() = default;
  virtual void init(rclcpp::Node & node) = 0;
  virtual std::unordered_map<PoseEstimatorName, bool> update() = 0;
  virtual std::vector<PoseEstimatorName> supporting_pose_estimators() = 0;

  virtual std::string debug_string() { return std::string{}; }
  virtual MarkerArray debug_marker_array() { return MarkerArray{}; }

protected:
  std::shared_ptr<rclcpp::Logger> logger_ptr_{nullptr};
};

}  // namespace multi_pose_estimator