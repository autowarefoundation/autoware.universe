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

#include "pose_estimator_arbiter/rule_helper/grid_info.hpp"
#include "pose_estimator_arbiter/switch_rule/map_based_rule.hpp"

namespace pose_estimator_arbiter::switch_rule
{
std::unordered_map<PoseEstimatorType, bool> MapBasedRule::update()
{
  auto ret = update_impl();
  RCLCPP_DEBUG(get_logger(), "%s", debug_string_.c_str());
  return ret;
}

std::unordered_map<PoseEstimatorType, bool> MapBasedRule::update_impl() const
{
  auto is_registered = [&list = std::as_const(running_estimator_list_)](
                         const PoseEstimatorType type) -> bool { return list.count(type) != 0; };

  // (1) If the localization state is not 'INITIALIZED'
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  if (shared_data_->initialization_state()->state != InitializationState::INITIALIZED) {
    debug_string_ = "Enable all: localization is not initialized";
    return {
      {PoseEstimatorType::ndt, true},
      {PoseEstimatorType::yabloc, true},
      {PoseEstimatorType::eagleye, true},
      {PoseEstimatorType::artag, true},
    };
  }

  // (2) If no pose are published, enable all;
  if (!shared_data_->localization_pose_cov.has_value()) {
    debug_string_ =
      "Enable all: estimated pose has not been published yet, so unable to determine which to use";
    return {
      {PoseEstimatorType::ndt, true},
      {PoseEstimatorType::yabloc, true},
      {PoseEstimatorType::eagleye, true},
      {PoseEstimatorType::artag, true},
    };
  }

  // (3) If eagleye is available, enable eagleye
  if (is_registered(PoseEstimatorType::eagleye)) {
    if (eagleye_is_available()) {
      debug_string_ = "Enable eagleye: the vehicle is within eagleye_area";
      return {
        {PoseEstimatorType::ndt, false},
        {PoseEstimatorType::yabloc, false},
        {PoseEstimatorType::eagleye, true},
        {PoseEstimatorType::artag, false},
      };
    } else {
      RCLCPP_DEBUG(get_logger(), "the vehicle is out of eagleye_area");
    }
  }

  // (4) If AR marker exists around ego position, enable artag
  if (is_registered(PoseEstimatorType::artag)) {
    if (artag_is_available()) {
      debug_string_ = "Enable artag: landmark exists around the ego";
      return {
        {PoseEstimatorType::ndt, false},
        {PoseEstimatorType::yabloc, false},
        {PoseEstimatorType::eagleye, false},
        {PoseEstimatorType::artag, true},
      };
    } else {
      RCLCPP_DEBUG(get_logger(), "there are no AR tags around the ego");
    }
  }

  // (5) If yabloc is disabled, enable ndt
  if (!is_registered(PoseEstimatorType::yabloc)) {
    debug_string_ = "Enable ndt: only ndt is available";
    return {
      {PoseEstimatorType::ndt, true},
      {PoseEstimatorType::yabloc, false},
      {PoseEstimatorType::eagleye, false},
      {PoseEstimatorType::artag, false},
    };
  }

  // (6) If ndt is disabled, enable YabLoc
  if (!is_registered(PoseEstimatorType::ndt)) {
    debug_string_ = "Enable yabloc: only yabloc is available";
    return {
      {PoseEstimatorType::ndt, false},
      {PoseEstimatorType::yabloc, true},
      {PoseEstimatorType::eagleye, false},
      {PoseEstimatorType::artag, false},
    };
  }

  // (7) If PCD is enough occupied, enable ndt. Otherwise, enable yabloc
  std::string ref_debug_string;
  if (ndt_is_more_suitable_than_yabloc(&ref_debug_string)) {
    debug_string_ = "Enable ndt: " + ref_debug_string;
    return {
      {PoseEstimatorType::ndt, true},
      {PoseEstimatorType::yabloc, false},
      {PoseEstimatorType::eagleye, false},
      {PoseEstimatorType::artag, false},
    };
  } else {
    debug_string_ = "Enable yabloc: " + ref_debug_string;
    return {
      {PoseEstimatorType::ndt, false},
      {PoseEstimatorType::yabloc, true},
      {PoseEstimatorType::eagleye, false},
      {PoseEstimatorType::artag, false}};
  }
}

}  // namespace pose_estimator_arbiter::switch_rule
