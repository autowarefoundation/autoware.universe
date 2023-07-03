#pragma once

#include <pose_estimator_manager/plugin_interface.hpp>
#include <rclcpp/logger.hpp>

namespace multi_pose_estimator
{

class SimpleSwitchRule : public PluginInterface
{
public:
  void init(rclcpp::Node & node) override;
  bool ndt_is_best() override;
  const char * get_module_name() override;

  rclcpp::Logger logger_{rclcpp::get_logger("simple_switch_rule")};
};
}  // namespace multi_pose_estimator