#ifndef POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_
#define POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_

#include "pose_estimator_manager/manager_client.hpp"
#include "pose_estimator_manager/plugin_interface.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace multi_pose_estimator
{
class PoseEstimatorManager : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;
  using String = std_msgs::msg::String;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  PoseEstimatorManager();

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_array_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_string_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unordered_map<PoseEstimatorName, ManagerClient::SharedPtr> clients_;

  pluginlib::ClassLoader<PluginInterface> plugin_loader_;
  std::shared_ptr<PluginInterface> switch_rule_plugin_;

  void toggle_all(bool enabled);
  void toggle_each(const std::unordered_map<PoseEstimatorName, bool> & toggle_list);

  void on_timer();

  void load_switch_rule_plugin(rclcpp::Node & node, const std::string & name);
};
}  // namespace multi_pose_estimator

#endif /* POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_ */