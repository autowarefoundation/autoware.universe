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

#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

namespace pose_estimator_manager::switch_rule
{
MapBasedRule::MapBasedRule(
  rclcpp::Node & node, const std::unordered_set<PoseEstimatorName> & running_estimator_list,
  const std::shared_ptr<const SharedData> shared_data)
: BaseSwitchRule(node),
  ar_marker_available_distance_(
    node.declare_parameter<int>("ar_marker_rule/ar_marker_available_distance")),
  running_estimator_list_(running_estimator_list),
  shared_data_(shared_data)
{
  RCLCPP_INFO_STREAM(get_logger(), "MapBasedRule is initialized");

  if (running_estimator_list.count(PoseEstimatorName::ndt)) {
    pcd_occupancy_ = std::make_unique<rule_helper::PcdOccupancy>(&node);
  }
  if (running_estimator_list.count(PoseEstimatorName::artag)) {
    ar_tag_position_ = std::make_unique<rule_helper::ArTagPosition>(&node);
  }
  if (running_estimator_list.count(PoseEstimatorName::eagleye)) {
    eagleye_area_ = std::make_unique<rule_helper::EagleyeArea>(&node);
  }
}

bool MapBasedRule::eagleye_is_available() const
{
  if (shared_data_->vector_map.has_value()) {
    if (!eagleye_area_->vector_map_initialized()) {
      eagleye_area_->init(shared_data_->vector_map());
    }
  }

  if (running_estimator_list_.count(PoseEstimatorName::eagleye) == 0) {
    return false;
  }

  if (!shared_data_->eagleye_output_pose_cov.has_value()) {
    return false;
  }

  if (!shared_data_->localization_pose_cov.has_value()) {
    return false;
  }

  if (!eagleye_area_) {
    throw std::runtime_error("eagleye_area_ is not initialized");
  }

  return eagleye_area_->within(shared_data_->localization_pose_cov()->pose.pose.position);
}
bool MapBasedRule::yabloc_is_available() const
{
  return running_estimator_list_.count(PoseEstimatorName::yabloc) != 0;
}

bool MapBasedRule::ndt_is_available() const
{
  return running_estimator_list_.count(PoseEstimatorName::ndt) != 0;
}

std::string MapBasedRule::debug_string()
{
  return debug_string_;
}

MapBasedRule::MarkerArray MapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;

  if (pcd_occupancy_) {
    const auto & additional = pcd_occupancy_->debug_marker_array().markers;
    array_msg.markers.insert(array_msg.markers.end(), additional.begin(), additional.end());
  }

  if (eagleye_area_) {
    const auto & additional = eagleye_area_->debug_marker_array().markers;
    array_msg.markers.insert(array_msg.markers.end(), additional.begin(), additional.end());
  }

  return array_msg;
}

bool MapBasedRule::artag_is_available() const
{
  if (shared_data_->vector_map.has_value()) {
    if (!ar_tag_position_->vector_map_initialized()) {
      ar_tag_position_->init(shared_data_->vector_map());
    }
  }

  if (running_estimator_list_.count(PoseEstimatorName::artag) == 0) {
    return false;
  }

  if (!shared_data_->localization_pose_cov.has_value()) {
    return false;
  }

  const auto position = shared_data_->localization_pose_cov()->pose.pose.position;
  const double distance_to_marker =
    ar_tag_position_->distance_to_nearest_ar_tag_around_ego(position);
  return distance_to_marker < ar_marker_available_distance_;
}

bool MapBasedRule::ndt_is_more_suitable_than_yabloc(std::string * optional_message) const
{
  if (!pcd_occupancy_) {
    throw std::runtime_error("pcd_occupancy is not initialized");
  }

  if (!shared_data_->localization_pose_cov.has_value()) {
    return false;
  }

  if (shared_data_->point_cloud_map.has_value()) {
    pcd_occupancy_->init(shared_data_->point_cloud_map());
  }

  const auto position = shared_data_->localization_pose_cov()->pose.pose.position;
  return pcd_occupancy_->ndt_can_operate(position, optional_message);
}

}  // namespace pose_estimator_manager::switch_rule
