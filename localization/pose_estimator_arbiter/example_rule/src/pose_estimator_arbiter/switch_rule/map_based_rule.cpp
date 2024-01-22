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

#include "pose_estimator_arbiter/switch_rule/map_based_rule.hpp"

namespace pose_estimator_arbiter::switch_rule
{
MapBasedRule::MapBasedRule(
  rclcpp::Node & node, const std::unordered_set<PoseEstimatorType> & running_estimator_list,
  const std::shared_ptr<const SharedData> shared_data)
: BaseSwitchRule(node), running_estimator_list_(running_estimator_list), shared_data_(shared_data)
{
  if (running_estimator_list.count(PoseEstimatorType::ndt)) {
    pcd_occupancy_ = std::make_unique<rule_helper::PcdOccupancy>(&node);

    // Register callback
    shared_data_->point_cloud_map.register_callback(
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
        pcd_occupancy_->init(msg);
      });
  }
}

MapBasedRule::MarkerArray MapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;

  if (pcd_occupancy_) {
    const auto & additional = pcd_occupancy_->debug_marker_array().markers;
    array_msg.markers.insert(array_msg.markers.end(), additional.begin(), additional.end());
  }

  return array_msg;
}

bool MapBasedRule::ndt_is_more_suitable_than_yabloc(std::string * optional_message) const
{
  if (!pcd_occupancy_) {
    throw std::runtime_error("pcd_occupancy is not initialized");
  }

  if (!shared_data_->localization_pose_cov.has_value()) {
    return false;
  }

  const auto position = shared_data_->localization_pose_cov()->pose.pose.position;
  return pcd_occupancy_->ndt_can_operate(position, optional_message);
}

std::unordered_map<PoseEstimatorType, bool> MapBasedRule::update()
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

  // (3) If PCD is enough occupied, enable ndt. Otherwise, enable yabloc
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
