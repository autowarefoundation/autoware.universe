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

#include "motion_factor_aggregator/motion_factor_aggregator.hpp"

#include <memory>
#include <vector>

namespace default_ad_api
{

bool compareFactorsByDistance(
  autoware_ad_api_msgs::msg::MotionFactor & a, autoware_ad_api_msgs::msg::MotionFactor & b)
{
  return a.distance < b.distance;
}

MotionFactorAggregator::MotionFactorAggregator(const rclcpp::NodeOptions & node_options)
: Node("motion_factor_aggregator", node_options),
  clock_(this->get_clock()),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
// : logger_(node.get_logger().get_child("motion_factor_aggregator")),
//   clock_(node.get_clock()),
//   tf_buffer_(this->get_clock()),
//   tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  status_pub_hz_ = this->declare_parameter("status_pub_hz", 10.0);
  timeout_ = this->declare_parameter("time_out", 0.5);

  // Publisher
  pub_motion_factor_ = this->create_publisher<autoware_ad_api_msgs::msg::MotionFactorArray>(
    "~/output/motion_factors", 100);

  // Subscriber
  sub_scene_module_motion_factor_ =
    this->create_subscription<autoware_ad_api_msgs::msg::MotionFactorArray>(
      "input/scene_module/motion_factor", 100,
      std::bind(&MotionFactorAggregator::callbackSceneModuleMotionFactor, this, _1));
  sub_obstacle_stop_motion_factor_ =
    this->create_subscription<autoware_ad_api_msgs::msg::MotionFactorArray>(
      "input/obstacle_stop/motion_factor", 100,
      std::bind(&MotionFactorAggregator::callbackObstacleStopMotionFactor, this, _1));
  sub_surround_obstacle_motion_factor_ =
    this->create_subscription<autoware_ad_api_msgs::msg::MotionFactorArray>(
      "input/surround_obstacle/motion_factor", 100,
      std::bind(&MotionFactorAggregator::callbackSurroundObstacleMotionFactor, this, _1));
  sub_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/autoware_trajectory", 1,
    std::bind(&MotionFactorAggregator::callbackAutowareTrajectory, this, _1));

  // timer
  auto timer_callback = std::bind(&MotionFactorAggregator::callbackTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_pub_hz_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void MotionFactorAggregator::callbackSceneModuleMotionFactor(
  const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr msg_ptr)
{
  applyUpdate(msg_ptr);
  applyTimeOut();
}

void MotionFactorAggregator::callbackObstacleStopMotionFactor(
  const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr msg_ptr)
{
  if (!obstacle_stop_factor_.motion_factors.empty()) {
    obstacle_stop_factor_.motion_factors.clear();
  }
  obstacle_stop_factor_ = *msg_ptr;
}

void MotionFactorAggregator::callbackSurroundObstacleMotionFactor(
  const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr msg_ptr)
{
  if (!surround_obstacle_factor_.motion_factors.empty()) {
    surround_obstacle_factor_.motion_factors.clear();
  }
  surround_obstacle_factor_ = *msg_ptr;
}

void MotionFactorAggregator::callbackAutowareTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr)
{
  autoware_planning_traj_ptr_ = msg_ptr;
}

void MotionFactorAggregator::callbackTimer()
{
  getCurrentPose();
  autoware_ad_api_msgs::msg::MotionFactorArray motion_factor_array_msg = makeMotionFactorArray();
  pub_motion_factor_->publish(motion_factor_array_msg);
}

void MotionFactorAggregator::applyUpdate(
  const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr)
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

bool MotionFactorAggregator::checkMatchingReason(
  const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_motion_factor_array,
  const autoware_ad_api_msgs::msg::MotionFactorArray & motion_factor_array)
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

void MotionFactorAggregator::applyTimeOut()
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

autoware_ad_api_msgs::msg::MotionFactorArray MotionFactorAggregator::makeMotionFactorArray()
{
  autoware_ad_api_msgs::msg::MotionFactorArray motion_factor_array_msg;
  // input header
  motion_factor_array_msg.header.frame_id = "map";
  motion_factor_array_msg.header.stamp = clock_->now();

  // input motion factor
  for (const auto & motion_factor_array : motion_factor_array_vec_) {
    for (const auto & motion_factor : motion_factor_array.motion_factors) {
      appendMotionFactorToArray(motion_factor, &motion_factor_array_msg);
    }
  }

  for (const auto & motion_factor : obstacle_stop_factor_.motion_factors) {
    appendMotionFactorToArray(motion_factor, &motion_factor_array_msg);
  }

  for (const auto & motion_factor : surround_obstacle_factor_.motion_factors) {
    appendMotionFactorToArray(motion_factor, &motion_factor_array_msg);
  }

  // sort factors in ascending order
  std::sort(
    motion_factor_array_msg.motion_factors.begin(), motion_factor_array_msg.motion_factors.end(),
    compareFactorsByDistance);

  return motion_factor_array_msg;
}

void MotionFactorAggregator::appendMotionFactorToArray(
  const autoware_ad_api_msgs::msg::MotionFactor & motion_factor,
  autoware_ad_api_msgs::msg::MotionFactorArray * motion_factor_array)
{
  // calculate distance to stop pose
  const auto motion_factor_with_dist = inputStopDistToMotionFactor(motion_factor);

  // if not exist same reason msg, append new stop reason
  motion_factor_array->motion_factors.emplace_back(motion_factor_with_dist);
}

autoware_ad_api_msgs::msg::MotionFactor MotionFactorAggregator::inputStopDistToMotionFactor(
  const autoware_ad_api_msgs::msg::MotionFactor & motion_factor)
{
  if (!autoware_planning_traj_ptr_ || !current_pose_ptr_) {
    // pass through all stop reason
    return motion_factor;
  }

  auto motion_factor_with_dist = motion_factor;
  const auto & trajectory = *autoware_planning_traj_ptr_;
  const auto & current_pose = current_pose_ptr_->pose;
  motion_factor_with_dist.distance = planning_util::calcDistanceAlongTrajectory(
    trajectory, current_pose, motion_factor_with_dist.pose);

  return motion_factor_with_dist;
}

void MotionFactorAggregator::getCurrentPose()
{
  try {
    auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = transform.header;
    ps.pose.position.x = transform.transform.translation.x;
    ps.pose.position.y = transform.transform.translation.y;
    ps.pose.position.z = transform.transform.translation.z;
    ps.pose.orientation = transform.transform.rotation;
    current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 2000 /* ms */, "cannot get self pose");
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::MotionFactorAggregator)
