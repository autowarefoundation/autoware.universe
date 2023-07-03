#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace multi_pose_estimator
{

class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void init(rclcpp::Node & node) = 0;
  virtual void best_estimator() = 0;
  virtual const char * get_module_name() = 0;
};

}  // namespace multi_pose_estimator