#include "simple_switch_rule.hpp"

namespace multi_pose_estimator
{

void SimpleSwitchRule::init(rclcpp::Node &)
{
  RCLCPP_INFO_STREAM(logger_, "pluginlib is initialized");
}

bool SimpleSwitchRule::ndt_is_best()
{
  RCLCPP_INFO_STREAM(logger_, "pluginlib try to choice the best pose_estimator");
  return true;
}

const char * SimpleSwitchRule::get_module_name()
{
  return "SimpleSwitchRule";
}

}  // namespace multi_pose_estimator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  multi_pose_estimator::SimpleSwitchRule, multi_pose_estimator::PluginInterface)