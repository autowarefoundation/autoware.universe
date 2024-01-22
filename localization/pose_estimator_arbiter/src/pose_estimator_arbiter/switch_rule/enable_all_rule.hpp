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

#ifndef POSE_ESTIMATOR_ARBITER__SWITCH_RULE__ENABLE_ALL_RULE_HPP_
#define POSE_ESTIMATOR_ARBITER__SWITCH_RULE__ENABLE_ALL_RULE_HPP_

#include "pose_estimator_arbiter/shared_data.hpp"
#include "pose_estimator_arbiter/switch_rule/base_switch_rule.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pose_estimator_arbiter::switch_rule
{
class EnableAllRule : public BaseSwitchRule
{
protected:
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  EnableAllRule(
    rclcpp::Node & node, const std::unordered_set<PoseEstimatorType> & running_estimator_list,
    const std::shared_ptr<const SharedData> shared_data);

  virtual ~EnableAllRule() = default;

  std::unordered_map<PoseEstimatorType, bool> update() override;

  std::string debug_string() override { return std::string{}; }

  MarkerArray debug_marker_array() override { return MarkerArray{}; }

protected:
  const std::unordered_set<PoseEstimatorType> running_estimator_list_;

  rclcpp::Logger get_logger() const { return *logger_ptr_; }

  std::shared_ptr<rclcpp::Logger> logger_ptr_{nullptr};
};

}  // namespace pose_estimator_arbiter::switch_rule

#endif  // POSE_ESTIMATOR_ARBITER__SWITCH_RULE__ENABLE_ALL_RULE_HPP_
