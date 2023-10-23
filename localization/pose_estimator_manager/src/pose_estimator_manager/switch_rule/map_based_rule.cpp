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
  rclcpp::Node & node, const std::unordered_set<PoseEstimatorName> & running_estimator_list)
: BaseSwitchRule(node),
  ar_marker_available_distance_(
    node.declare_parameter<int>("ar_marker_rule/ar_marker_available_distance")),
  running_estimator_list_(running_estimator_list)
{
  RCLCPP_INFO_STREAM(get_logger(), "MapBasedRule is initialized");

  using std::placeholders::_1;
  const auto latch_qos = rclcpp::QoS(1).transient_local().reliable();

  auto on_vector_map = std::bind(&MapBasedRule::on_vector_map, this, _1);
  auto on_point_cloud_map = std::bind(&MapBasedRule::on_point_cloud_map, this, _1);
  auto on_pose_cov = std::bind(&MapBasedRule::on_pose_cov, this, _1);
  auto on_initialization_state = [this](InitializationState::ConstSharedPtr msg) -> void {
    initialization_state_ = *msg;
  };
  auto on_eagleye_fix = [this](NavSatFix::ConstSharedPtr) -> void {
    eagleye_is_initialized = true;
  };

  sub_vector_map_ =
    node.create_subscription<HADMapBin>("~/input/vector_map", latch_qos, on_vector_map);
  sub_point_cloud_map_ =
    node.create_subscription<PointCloud2>("~/input/pointcloud_map", latch_qos, on_point_cloud_map);
  sub_pose_cov_ =
    node.create_subscription<PoseCovStamped>("~/input/pose_with_covariance", 10, on_pose_cov);
  sub_initialization_state_ = node.create_subscription<InitializationState>(
    "~/input/initialization_state", latch_qos, on_initialization_state);
  sub_eagleye_fix_ = node.create_subscription<NavSatFix>("~/input/eagleye/fix", 10, on_eagleye_fix);

  if (running_estimator_list.count(PoseEstimatorName::ndt)) {
    pcd_occupancy_ = std::make_unique<rule_helper::PcdOccupancy>(&node);
  }
  if (running_estimator_list.count(PoseEstimatorName::artag)) {
    ar_tag_position_ = std::make_unique<rule_helper::ArTagPosition>(&node);
  }
  if (running_estimator_list.count(PoseEstimatorName::eagleye)) {
    eagleye_area_ = std::make_unique<rule_helper::EagleyeArea>(&node);
  }

  //
  initialization_state_.state = InitializationState::UNINITIALIZED;
}

bool MapBasedRule::eagleye_is_available() const
{
  if (running_estimator_list_.count(PoseEstimatorName::eagleye) == 0) {
    return false;
  }

  if (!eagleye_is_initialized) {
    return false;
  }

  if (!latest_pose_.has_value()) {
    return false;
  }

  if (!eagleye_area_) {
    throw std::runtime_error("eagleye_area_ is not initialized");
  }

  return eagleye_area_->within(latest_pose_->pose.pose.position);
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
  return debug_string_msg_;
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

void MapBasedRule::on_vector_map(HADMapBin::ConstSharedPtr msg)
{
  if (eagleye_area_) {
    eagleye_area_->init(msg);
  }
}

void MapBasedRule::on_point_cloud_map(PointCloud2::ConstSharedPtr msg)
{
  if (pcd_occupancy_) {
    pcd_occupancy_->on_point_cloud_map(msg);
  }
}

void MapBasedRule::on_pose_cov(PoseCovStamped::ConstSharedPtr msg)
{
  latest_pose_ = *msg;
}

bool MapBasedRule::artag_is_available() const
{
  if (running_estimator_list_.count(PoseEstimatorName::artag) == 0) {
    return false;
  }

  if (!latest_pose_.has_value()) {
    return false;
  }

  const double distance_to_marker =
    ar_tag_position_->distance_to_nearest_ar_tag_around_ego(latest_pose_->pose.pose.position);
  return distance_to_marker < ar_marker_available_distance_;
}

bool MapBasedRule::ndt_is_more_suitable_than_yabloc(std::string * optional_message) const
{
  if (!pcd_occupancy_) {
    throw std::runtime_error("pcd_occupancy is not initialized");
  }

  if (!latest_pose_) {
    return false;
  }

  const auto position = latest_pose_->pose.pose.position;
  return pcd_occupancy_->ndt_can_operate(position, optional_message);
}

}  // namespace pose_estimator_manager::switch_rule
