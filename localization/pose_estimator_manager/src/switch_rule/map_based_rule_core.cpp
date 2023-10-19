#include "pose_estimator_manager/switch_rule/grid_info.hpp"
#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace multi_pose_estimator
{
std::unordered_map<PoseEstimatorName, bool> MapBasedRule::update()
{
  // (1) If the localization state is not 'INITIALIZED'
  if (initialization_state_.state != InitializationState::INITIALIZED) {
    debug_string_msg_ = "enable All\nlocalization is not initialized";
    RCLCPP_WARN_STREAM(
      get_logger(), "Enable all estimators because localization component is not initialized");
    return {
      {PoseEstimatorName::NDT, true},
      {PoseEstimatorName::YABLOC, true},
      {PoseEstimatorName::EAGLEYE, true},
      {PoseEstimatorName::ARTAG, true},
    };
  }

  // (2) If no pose are published, enable all;
  if (!latest_pose_.has_value()) {
    debug_string_msg_ = "enable All\nestimated pose has not been published yet";
    RCLCPP_WARN_STREAM(
      get_logger(), "Unable to determine which estimation to use, due to lack of latest position");
    return {
      {PoseEstimatorName::NDT, true},
      {PoseEstimatorName::YABLOC, true},
      {PoseEstimatorName::EAGLEYE, true},
      {PoseEstimatorName::ARTAG, true},
    };
  }

  // (3) If eagleye is vailable, enable eagleye
  if (eagleye_is_available()) {
    debug_string_msg_ = "enable Eagleye\nthe vehicle is within eagleye_area";
    RCLCPP_WARN_STREAM(get_logger(), "Enable eagleye");
    return {
      {PoseEstimatorName::NDT, false},
      {PoseEstimatorName::YABLOC, false},
      {PoseEstimatorName::EAGLEYE, true},
      {PoseEstimatorName::ARTAG, false},
    };
  }

  // (4) If AR marker exists around ego position, enable ARTAG
  if (artag_is_available()) {
    debug_string_msg_ = "enable ARTAG\nlandmark exists around the ego";
    RCLCPP_WARN_STREAM(get_logger(), "Enable ARTAG");
    return {
      {PoseEstimatorName::NDT, false},
      {PoseEstimatorName::YABLOC, false},
      {PoseEstimatorName::EAGLEYE, false},
      {PoseEstimatorName::ARTAG, true},
    };
  }

  // (5) If yabloc is disabled, enable NDT
  if (!yabloc_is_available()) {
    debug_string_msg_ = "enable NDT\nonly NDT is available";
    RCLCPP_WARN_STREAM(get_logger(), "Enable NDT");
    return {
      {PoseEstimatorName::NDT, true},
      {PoseEstimatorName::YABLOC, false},
      {PoseEstimatorName::EAGLEYE, false},
      {PoseEstimatorName::ARTAG, false},
    };
  }

  // (6) If ndt is disabled, enable YabLoc
  if (!ndt_is_available()) {
    debug_string_msg_ = "enable YabLoc\nonly yabloc is available";
    RCLCPP_WARN_STREAM(get_logger(), "Enable YABLOC");
    return {
      {PoseEstimatorName::NDT, false},
      {PoseEstimatorName::YABLOC, true},
      {PoseEstimatorName::EAGLEYE, false},
      {PoseEstimatorName::ARTAG, false},
    };
  }

  // (7) If PCD is enough occupied, enable NDT
  if (ndt_is_more_suitable_than_yabloc(&debug_string_msg_)) {
    RCLCPP_WARN_STREAM(get_logger(), "Enable YABLOC");
    return {
      {PoseEstimatorName::NDT, false},
      {PoseEstimatorName::YABLOC, true},
      {PoseEstimatorName::EAGLEYE, false},
    };
  }

  // (8) Enable YabLoc
  RCLCPP_WARN_STREAM(get_logger(), "Enable YabLoc");
  return {
    {PoseEstimatorName::NDT, false},
    {PoseEstimatorName::YABLOC, true},
    {PoseEstimatorName::EAGLEYE, false},
    {PoseEstimatorName::ARTAG, false}};
}

}  // namespace multi_pose_estimator