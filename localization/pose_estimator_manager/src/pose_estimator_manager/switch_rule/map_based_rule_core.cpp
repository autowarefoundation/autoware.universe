// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pose_estimator_manager/rule_helper/grid_info.hpp"
#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

namespace pose_estimator_manager::switch_rule
{
std::unordered_map<PoseEstimatorName, bool> MapBasedRule::update()
{
  // (1) If the localization state is not 'INITIALIZED'
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  if (shared_data_->initialization_state()->state != InitializationState::INITIALIZED) {
    debug_string_ = "enable All\nlocalization is not initialized";
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, true},
      {PoseEstimatorName::artag, true},
    };
  }

  // (2) If no pose are published, enable all;
  if (!shared_data_->localization_pose_cov.has_value()) {
    debug_string_ =
      "enable All\nestimated pose has not been published yet, so unable to determine which to use";
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, true},
      {PoseEstimatorName::artag, true},
    };
  }

  // (3) If eagleye is vailable, enable eagleye
  if (eagleye_is_available()) {
    debug_string_ = "enable Eagleye\nthe vehicle is within eagleye_area";
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, false},
      {PoseEstimatorName::yabloc, false},
      {PoseEstimatorName::eagleye, true},
      {PoseEstimatorName::artag, false},
    };
  }

  // (4) If AR marker exists around ego position, enable artag
  if (artag_is_available()) {
    debug_string_ = "enable Artag\nlandmark exists around the ego";
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, false},
      {PoseEstimatorName::yabloc, false},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, true},
    };
  }

  // (5) If yabloc is disabled, enable ndt
  if (!yabloc_is_available()) {
    debug_string_ = "enable Ndt\nonly ndt is available";
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, false},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false},
    };
  }

  // (6) If ndt is disabled, enable YabLoc
  if (!ndt_is_available()) {
    debug_string_ = "enable YabLoc\nonly yabloc is available";
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, false},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false},
    };
  }

  // (7) If PCD is enough occupied, enable ndt. Otherwise, enable yabloc
  if (ndt_is_more_suitable_than_yabloc(&debug_string_)) {
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, false},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false},
    };
  } else {
    RCLCPP_WARN_STREAM(get_logger(), debug_string_);
    return {
      {PoseEstimatorName::ndt, false},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false}};
  }
}

}  // namespace pose_estimator_manager::switch_rule
