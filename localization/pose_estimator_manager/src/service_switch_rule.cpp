#include "pose_estimator_manager/service_switch_rule.hpp"

namespace multi_pose_estimator
{

void ServiceSwitchRule::init(rclcpp::Node &)
{
  RCLCPP_INFO_STREAM(logger_, "pluginlib is initialized");
}

void ServiceSwitchRule::best_estimator()
{
  RCLCPP_INFO_STREAM(logger_, "pluginlib try to choice the best pose_estimator");
}

const char * ServiceSwitchRule::get_module_name()
{
  return "ServiceSwitchRule";
}

}  // namespace multi_pose_estimator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  multi_pose_estimator::ServiceSwitchRule, multi_pose_estimator::PluginInterface)