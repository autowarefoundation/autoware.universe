#include "pose_estimator_manager/pose_estimator_manager.hpp"
#include "pose_estimator_manager/pose_estimator_name.hpp"

#include <sstream>
namespace multi_pose_estimator
{

PoseEstimatorManager::PoseEstimatorManager()
: Node("pose_estimator_manager"),
  plugin_loader_("pose_estimator_manager", "multi_pose_estimator::PluginInterface")
{
  // Publisher
  pub_debug_string_ = create_publisher<String>("~/debug/string", 10);
  pub_debug_marker_array_ = create_publisher<MarkerArray>("~/debug/marker_array", 10);

  // Load plugin
  RCLCPP_INFO_STREAM(get_logger(), "load plugin");
  load_switch_rule_plugin(*this, declare_parameter<std::string>("estimator_switch_plugin"));

  // Service client
  RCLCPP_INFO_STREAM(get_logger(), "define service client");
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  for (auto pose_estimator_name : switch_rule_plugin_->supporting_pose_estimators()) {
    switch (pose_estimator_name) {
      case PoseEstimatorName::NDT:
        clients_.emplace(
          PoseEstimatorName::NDT,
          std::make_shared<ManagerClient>(this, "~/ndt_enable_srv", service_callback_group_));
        break;
      case PoseEstimatorName::YABLOC:
        clients_.emplace(
          PoseEstimatorName::YABLOC,
          std::make_shared<ManagerClient>(this, "~/yabloc_enable_srv", service_callback_group_));
        break;
      case PoseEstimatorName::ARUCO:
        clients_.emplace(
          PoseEstimatorName::ARUCO,
          std::make_shared<ManagerClient>(this, "~/aruco_enable_srv", service_callback_group_));
        break;
      default:
        RCLCPP_WARN_STREAM(get_logger(), "invalid pose_estimator is specified");
    }
  }

  // Timer callback
  auto on_timer = std::bind(&PoseEstimatorManager::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer));

  RCLCPP_INFO_STREAM(get_logger(), "toggle all");
  toggle_all(true);
}

void PoseEstimatorManager::load_switch_rule_plugin(rclcpp::Node & node, const std::string & name)
{
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(node);
    switch_rule_plugin_ = plugin;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "The switch rule plugin '" << name << "' is not available.");
    exit(EXIT_FAILURE);
  }
}

void PoseEstimatorManager::toggle_each(
  const std::unordered_map<PoseEstimatorName, bool> & toggle_list)
{
  for (auto c : clients_) {
    if (toggle_list.at(c.first)) {
      c.second->enable();
    } else {
      c.second->disable();
    }
  }
}

void PoseEstimatorManager::toggle_all(bool enabled)
{
  std::unordered_map<PoseEstimatorName, bool> toggle_list;
  for (auto c : clients_) {
    toggle_list.emplace(c.first, enabled);
  }

  toggle_each(toggle_list);
}

void PoseEstimatorManager::on_timer()
{
  RCLCPP_INFO_STREAM(get_logger(), "on_timer");

  if (switch_rule_plugin_) {
    auto toggle_list = switch_rule_plugin_->update();
    toggle_each(toggle_list);

    {
      String msg;
      msg.data = switch_rule_plugin_->debug_string();
      pub_debug_string_->publish(msg);
    }
    {
      MarkerArray msg = switch_rule_plugin_->debug_marker_array();
      pub_debug_marker_array_->publish(msg);
    }

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), "swtich_rule is not activated. Therefore, enable all pose_estimators");
    toggle_all(true);
  }
}

}  // namespace multi_pose_estimator