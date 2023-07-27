#include "pose_estimator_manager/pose_estimator_manager.hpp"
#include "pose_estimator_manager/pose_estimator_name.hpp"
#include "pose_estimator_manager/sub_manager/sub_manager_eagleye.hpp"
#include "pose_estimator_manager/sub_manager/sub_manager_ndt.hpp"
#include "pose_estimator_manager/sub_manager/sub_manager_yabloc.hpp"
#include "pose_estimator_manager/switch_rule/pcd_occupancy_rule.hpp"

#include <sstream>

namespace multi_pose_estimator
{

static std::vector<PoseEstimatorName> parse_estimator_name_args(
  const std::vector<std::string> & arg)
{
  std::vector<PoseEstimatorName> running_estimator_list;
  for (const auto & estimator_name : arg) {
    // TODO: Use magic_enum or fuzzy interpretation
    if (estimator_name == "ndt") {
      running_estimator_list.push_back(PoseEstimatorName::NDT);
    } else if (estimator_name == "yabloc") {
      running_estimator_list.push_back(PoseEstimatorName::YABLOC);
    } else if (estimator_name == "eagleye") {
      running_estimator_list.push_back(PoseEstimatorName::EAGLEYE);
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("pose_estimator_manager"),
        "invalid pose_estimator_name is spciefied: " << estimator_name);
    }
  }
  return running_estimator_list;
}

PoseEstimatorManager::PoseEstimatorManager()
: Node("pose_estimator_manager"),
  running_estimator_list_(
    parse_estimator_name_args(declare_parameter<std::vector<std::string>>("pose_sources")))
{
  // Publisher
  pub_debug_string_ = create_publisher<String>("~/debug/string", 10);
  pub_debug_marker_array_ = create_publisher<MarkerArray>("~/debug/marker_array", 10);

  // sub-managers
  for (auto pose_estimator_name : running_estimator_list_) {
    // TODO: Use magic enum
    switch (pose_estimator_name) {
      case PoseEstimatorName::NDT:
        sub_managers_.emplace(PoseEstimatorName::NDT, std::make_shared<SubManagerNdt>(this));
        break;
      case PoseEstimatorName::YABLOC:
        sub_managers_.emplace(PoseEstimatorName::YABLOC, std::make_shared<SubManagerYabLoc>(this));
        break;
      case PoseEstimatorName::EAGLEYE:
        sub_managers_.emplace(
          PoseEstimatorName::EAGLEYE, std::make_shared<SubManagerEagleye>(this));
        break;
      default:
        RCLCPP_WARN_STREAM(get_logger(), "invalid pose_estimator is specified");
    }
  }

  // Load switching rule
  load_switch_rule();

  // Timer callback
  auto on_timer = std::bind(&PoseEstimatorManager::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer));

  // Enable all pose estimators at the first
  toggle_all(true);
}

void PoseEstimatorManager::load_switch_rule()
{
  // NOTE: In the future, some rule will be laid below
  RCLCPP_INFO_STREAM(get_logger(), "load default switching rule");
  switch_rule_ = std::make_shared<PcdOccupancyRule>(*this);
}

void PoseEstimatorManager::toggle_each(
  const std::unordered_map<PoseEstimatorName, bool> & toggle_list)
{
  for (auto s : sub_managers_) {
    if (toggle_list.at(s.first)) {
      s.second->enable();
    } else {
      s.second->disable();
    }
  }
}

void PoseEstimatorManager::toggle_all(bool enabled)
{
  RCLCPP_INFO_STREAM(get_logger(), (enabled ? "Enable" : "Disable") << " all pose estimators");

  std::unordered_map<PoseEstimatorName, bool> toggle_list;
  for (auto s : sub_managers_) {
    toggle_list.emplace(s.first, enabled);
  }
  toggle_each(toggle_list);
}

void PoseEstimatorManager::on_timer()
{
  RCLCPP_INFO_STREAM(get_logger(), "on_timer");

  if (switch_rule_) {
    auto toggle_list = switch_rule_->update();
    toggle_each(toggle_list);

    {
      String msg;
      msg.data = switch_rule_->debug_string();
      pub_debug_string_->publish(msg);
    }
    {
      MarkerArray msg = switch_rule_->debug_marker_array();
      pub_debug_marker_array_->publish(msg);
    }

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), "swtich_rule is not activated. Therefore, enable all pose_estimators");
    toggle_all(true);
  }
}

}  // namespace multi_pose_estimator