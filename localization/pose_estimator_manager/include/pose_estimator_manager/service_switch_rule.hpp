#pragma once

#include <pose_estimator_manager/plugin_interface.hpp>
#include <rclcpp/logger.hpp>

#include <std_srvs/srv/set_bool.hpp>

namespace multi_pose_estimator
{

class ServiceSwitchRule : public PluginInterface
{
public:
  using SetBool = std_srvs::srv::SetBool;

  void init(rclcpp::Node & node) override;
  bool ndt_is_best() override;
  const char * get_module_name() override;

  rclcpp::Logger logger_{rclcpp::get_logger("service_switch_rule")};

protected:
  bool use_ndt_{false};

  rclcpp::Service<SetBool>::SharedPtr switcher_set_server_;

  void on_service(SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response);
};
}  // namespace multi_pose_estimator