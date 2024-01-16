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

#include "pose_estimator_arbiter/switch_rule/vector_map_based_rule.hpp"

#include <magic_enum.hpp>

namespace pose_estimator_arbiter::switch_rule
{
VectorMapBasedRule::VectorMapBasedRule(
  rclcpp::Node & node, const std::unordered_set<PoseEstimatorType> & running_estimator_list,
  const std::shared_ptr<const SharedData> shared_data)
: BaseSwitchRule(node),
  ar_marker_available_distance_(
    node.declare_parameter<int>("ar_marker_rule/ar_marker_available_distance")),
  running_estimator_list_(running_estimator_list),
  shared_data_(shared_data)
{
  pose_estimator_area_ = std::make_unique<rule_helper::PoseEstimatorArea>(node.get_logger());

  // Register callback
  shared_data_->vector_map.register_callback(
    [this](autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg) -> void {
      pose_estimator_area_->init(msg);
    });

  RCLCPP_INFO_STREAM(get_logger(), "VectorMapBasedRule is initialized successfully");
}

VectorMapBasedRule::MarkerArray VectorMapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;

  if (pose_estimator_area_) {
    const auto & additional = pose_estimator_area_->debug_marker_array().markers;
    array_msg.markers.insert(array_msg.markers.end(), additional.begin(), additional.end());
  }

  return array_msg;
}

std::string VectorMapBasedRule::debug_string()
{
  return debug_string_;
}

std::unordered_map<PoseEstimatorType, bool> VectorMapBasedRule::update()
{
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

  const auto ego_position = shared_data_->localization_pose_cov()->pose.pose.position;

  // (3)
  std::unordered_map<PoseEstimatorType, bool> enable_list;
  for (const auto & estimator_type : running_estimator_list_) {
    debug_string_ =
      "Enable all: estimated pose has not been published yet, so unable to determine which to use";
    const std::string estimator_name{magic_enum::enum_name(estimator_type)};
    const bool result = pose_estimator_area_->within(ego_position, estimator_name);
    enable_list.emplace(estimator_type, result);
  }

  return enable_list;
}

}  // namespace pose_estimator_arbiter::switch_rule
