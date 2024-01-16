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

std::string VectorMapBasedRule::debug_string()
{
  return debug_string_;
}

VectorMapBasedRule::MarkerArray VectorMapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;

  return array_msg;
}

std::unordered_map<PoseEstimatorType, bool> VectorMapBasedRule::update()
{
  // TODO:
  debug_string_ = "Enable all: localization is not initialized";
  return {
    {PoseEstimatorType::ndt, true},
    {PoseEstimatorType::yabloc, true},
    {PoseEstimatorType::eagleye, true},
    {PoseEstimatorType::artag, true},
  };
}

}  // namespace pose_estimator_arbiter::switch_rule
