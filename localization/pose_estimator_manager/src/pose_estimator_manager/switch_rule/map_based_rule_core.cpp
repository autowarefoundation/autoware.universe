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
  // clear debug string
  debug_string_dictionary_.clear();

  const auto result = update_impl();
  RCLCPP_DEBUG_STREAM(get_logger(), debug_string_dictionary_["summary"]);
  return result;
}

std::unordered_map<PoseEstimatorName, bool> MapBasedRule::update_impl() const
{
  auto is_registered = [&list = std::as_const(running_estimator_list_)](
                         const PoseEstimatorName name) -> bool { return list.count(name) != 0; };

  // (1) If the localization state is not 'INITIALIZED'
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  if (shared_data_->initialization_state()->state != InitializationState::INITIALIZED) {
    debug_string_dictionary_["summary"] = "Enable all: localization is not initialized";
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, true},
      {PoseEstimatorName::artag, true},
    };
  }

  // (2) If no pose are published, enable all;
  if (!shared_data_->localization_pose_cov.has_value()) {
    debug_string_dictionary_["summary"] =
      "Enable all: estimated pose has not been published yet, so unable to determine which to use";
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, true},
      {PoseEstimatorName::artag, true},
    };
  }

  // (3) If eagleye is available, enable eagleye
  if (is_registered(PoseEstimatorName::eagleye)) {
    if (eagleye_is_available()) {
      debug_string_dictionary_["summary"] = "Enable eagleye: the vehicle is within eagleye_area";
      return {
        {PoseEstimatorName::ndt, false},
        {PoseEstimatorName::yabloc, false},
        {PoseEstimatorName::eagleye, true},
        {PoseEstimatorName::artag, false},
      };
    } else {
      debug_string_dictionary_["eagleye"] = "the vehicle is out of eagleye_area";
    }
  }

  // (4) If AR marker exists around ego position, enable artag
  if (is_registered(PoseEstimatorName::artag)) {
    if (artag_is_available()) {
      debug_string_dictionary_["summary"] = "Enable artag: landmark exists around the ego";
      return {
        {PoseEstimatorName::ndt, false},
        {PoseEstimatorName::yabloc, false},
        {PoseEstimatorName::eagleye, false},
        {PoseEstimatorName::artag, true},
      };
    } else {
      debug_string_dictionary_["artag"] = "there are no landmarks around the ego";
    }
  }

  // (5) If yabloc is disabled, enable ndt
  if (!is_registered(PoseEstimatorName::yabloc)) {
    debug_string_dictionary_["summary"] = "Enable ndt: only ndt is available";
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, false},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false},
    };
  }

  // (6) If ndt is disabled, enable YabLoc
  if (!is_registered(PoseEstimatorName::ndt)) {
    debug_string_dictionary_["summary"] = "Enable yabloc: only yabloc is available";
    return {
      {PoseEstimatorName::ndt, false},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false},
    };
  }

  // (7) If PCD is enough occupied, enable ndt. Otherwise, enable yabloc
  std::string ref_debug_string;
  if (ndt_is_more_suitable_than_yabloc(&ref_debug_string)) {
    debug_string_dictionary_["summary"] = "Enable ndt: " + ref_debug_string;
    return {
      {PoseEstimatorName::ndt, true},
      {PoseEstimatorName::yabloc, false},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false},
    };
  } else {
    debug_string_dictionary_["summary"] = "Enable yabloc: " + ref_debug_string;
    return {
      {PoseEstimatorName::ndt, false},
      {PoseEstimatorName::yabloc, true},
      {PoseEstimatorName::eagleye, false},
      {PoseEstimatorName::artag, false}};
  }
}

}  // namespace pose_estimator_manager::switch_rule
