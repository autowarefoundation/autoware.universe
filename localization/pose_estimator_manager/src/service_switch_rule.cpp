#include "pose_estimator_manager/service_switch_rule.hpp"

namespace multi_pose_estimator
{

void ServiceSwitchRule::init(rclcpp::Node & node)
{
  RCLCPP_INFO_STREAM(logger_, "pluginlib is initialized");

  using std::placeholders::_1;
  using std::placeholders::_2;

  auto on_service = std::bind(&ServiceSwitchRule::on_service, this, _1, _2);
  switcher_set_server_ = node.create_service<SetBool>("~/switch_ndt_srv", on_service);
}

bool ServiceSwitchRule::ndt_is_best()
{
  RCLCPP_INFO_STREAM(logger_, "pluginlib try to choice the best pose_estimator");
  return use_ndt_;
}

const char * ServiceSwitchRule::get_module_name()
{
  return "ServiceSwitchRule";
}

void ServiceSwitchRule::on_service(
  SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response)
{
  use_ndt_ = request->data;
  response->success = true;
}

}  // namespace multi_pose_estimator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  multi_pose_estimator::ServiceSwitchRule, multi_pose_estimator::PluginInterface)