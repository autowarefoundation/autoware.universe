// Copyright 2020 TIER IV, Inc.
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

#include "ad_api_adapter/motion_factor_aggregator.hpp"

#include <memory>
#include <vector>

namespace autoware_api
{

bool compareFactorsByDistance(
  tier4_planning_msgs::msg::MotionFactor & a, tier4_planning_msgs::msg::MotionFactor & b)
{
  return a.dist_to_stop_pose < b.dist_to_stop_pose;
}

AutowareIvMotionFactorAggregator::AutowareIvMotionFactorAggregator(
  rclcpp::Node & node, const double timeout)
: logger_(node.get_logger().get_child("awapi_awiv_motion_factor_aggregator")),
  clock_(node.get_clock()),
  timeout_(timeout)
{
}

void AutowareIvMotionFactorAggregator::updateSceneModuleMotionFactorArray(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
{
  applyUpdate(msg_ptr);
  applyTimeOut();
}

void AutowareIvMotionFactorAggregator::updateObstacleStopMotionFactorArray(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
{
  if (!obstacle_stop_factor_.motion_factors.empty()) {
    obstacle_stop_factor_.motion_factors.clear();
  }
  obstacle_stop_factor_ = *msg_ptr;
}

void AutowareIvMotionFactorAggregator::updateSurroundObstacleMotionFactorArray(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
{
  if (!surround_obstacle_factor_.motion_factors.empty()) {
    surround_obstacle_factor_.motion_factors.clear();
  }
  surround_obstacle_factor_ = *msg_ptr;
}

void AutowareIvMotionFactorAggregator::applyUpdate(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
{
  /* remove old motion factor that matches reason with received msg */
  // make reason-matching msg list

  std::vector<size_t> remove_idx;
  if (!motion_factor_array_vec_.empty()) {
    for (int i = motion_factor_array_vec_.size() - 1; i >= 0; i--) {
      if (checkMatchingReason(msg_ptr, motion_factor_array_vec_.at(i))) {
        remove_idx.emplace_back(i);
      }
    }
  }

  // remove reason matching msg
  for (const auto idx : remove_idx) {
    motion_factor_array_vec_.erase(motion_factor_array_vec_.begin() + idx);
  }

  // add new reason msg
  motion_factor_array_vec_.emplace_back(*msg_ptr);
}

bool AutowareIvMotionFactorAggregator::checkMatchingReason(
  const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_motion_factor_array,
  const tier4_planning_msgs::msg::MotionFactorArray & motion_factor_array)
{
  for (const auto & msg_motion_factor : msg_motion_factor_array->motion_factors) {
    for (const auto & motion_factor : motion_factor_array.motion_factors) {
      if (msg_motion_factor.reason == motion_factor.reason) {
        // find matching reason
        return true;
      }
    }
  }
  // cannot find matching reason
  return false;
}

void AutowareIvMotionFactorAggregator::applyTimeOut()
{
  const auto current_time = clock_->now();

  // make timeout-msg list
  std::vector<size_t> remove_idx;
  if (!motion_factor_array_vec_.empty()) {
    for (int i = motion_factor_array_vec_.size() - 1; i >= 0; i--) {
      if (
        (current_time - rclcpp::Time(motion_factor_array_vec_.at(i).header.stamp)).seconds() >
        timeout_) {
        remove_idx.emplace_back(i);
      }
    }
  }
  // remove timeout-msg
  for (const auto idx : remove_idx) {
    motion_factor_array_vec_.erase(motion_factor_array_vec_.begin() + idx);
  }
}

tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr
AutowareIvMotionFactorAggregator::makeMotionFactorArray(const AutowareInfo & aw_info)
{
  tier4_planning_msgs::msg::MotionFactorArray motion_factor_array_msg;
  // input header
  motion_factor_array_msg.header.frame_id = "map";
  motion_factor_array_msg.header.stamp = clock_->now();

  // input motion factor
  for (const auto & motion_factor_array : motion_factor_array_vec_) {
    for (const auto & motion_factor : motion_factor_array.motion_factors) {
      appendMotionFactorToArray(motion_factor, &motion_factor_array_msg, aw_info);
    }
  }

  for (const auto & motion_factor : obstacle_stop_factor_.motion_factors) {
    appendMotionFactorToArray(motion_factor, &motion_factor_array_msg, aw_info);
  }

  for (const auto & motion_factor : surround_obstacle_factor_.motion_factors) {
    appendMotionFactorToArray(motion_factor, &motion_factor_array_msg, aw_info);
  }

  // sort factors in ascending order
  std::sort(
    motion_factor_array_msg.motion_factors.begin(), motion_factor_array_msg.motion_factors.end(),
    compareFactorsByDistance);

  return std::make_shared<tier4_planning_msgs::msg::MotionFactorArray>(motion_factor_array_msg);
}

void AutowareIvMotionFactorAggregator::appendMotionFactorToArray(
  const tier4_planning_msgs::msg::MotionFactor & motion_factor,
  tier4_planning_msgs::msg::MotionFactorArray * motion_factor_array, const AutowareInfo & aw_info)
{
  // calculate dist_to_stop_pose
  const auto motion_factor_with_dist = inputStopDistToMotionFactor(motion_factor, aw_info);

  // if not exist same reason msg, append new stop reason
  motion_factor_array->motion_factors.emplace_back(motion_factor_with_dist);
}

tier4_planning_msgs::msg::MotionFactor
AutowareIvMotionFactorAggregator::inputStopDistToMotionFactor(
  const tier4_planning_msgs::msg::MotionFactor & motion_factor, const AutowareInfo & aw_info)
{
  if (!aw_info.autoware_planning_traj_ptr || !aw_info.current_pose_ptr) {
    // pass through all stop reason
    return motion_factor;
  }

  auto motion_factor_with_dist = motion_factor;
  const auto & trajectory = *aw_info.autoware_planning_traj_ptr;
  const auto & current_pose = aw_info.current_pose_ptr->pose;
  motion_factor_with_dist.distance = planning_util::calcDistanceAlongTrajectory(
    trajectory, current_pose, motion_factor_with_dist.pose);

  return motion_factor_with_dist;
}

}  // namespace autoware_api
