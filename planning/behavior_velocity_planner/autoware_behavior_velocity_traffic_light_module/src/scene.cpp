// Copyright 2020 Tier IV, Inc.
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

#include "scene.hpp"

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <tf2/utils.h>

#include <memory>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
TrafficLightModule::TrafficLightModule(
  const int64_t lane_id, const lanelet::TrafficLight & traffic_light_reg_elem,
  lanelet::ConstLanelet lane, const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<universe_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(lane_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  traffic_light_reg_elem_(traffic_light_reg_elem),
  lane_(lane),
  state_(State::APPROACH),
  debug_data_(),
  is_prev_state_stop_(false)
{
  planner_param_ = planner_param;
}

bool TrafficLightModule::modifyPathVelocity(PathWithLaneId * path)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  first_ref_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;

  const auto input_path = *path;

  const auto & self_pose = planner_data_->current_odometry;

  // Get lanelet2 stop lines.
  lanelet::ConstLineString3d lanelet_stop_lines = *(traffic_light_reg_elem_.stopLine());

  // Calculate stop pose and insert index
  const auto stop_line = calcStopPointAndInsertIndex(
    input_path, lanelet_stop_lines,
    planner_param_.stop_margin + planner_data_->vehicle_info_.max_longitudinal_offset_m,
    planner_data_->stop_line_extend_length);

  if (!stop_line.has_value()) {
    RCLCPP_WARN_STREAM_ONCE(
      logger_, "Failed to calculate stop point and insert index for regulatory element id "
                 << traffic_light_reg_elem_.id());
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }

  // Calculate dist to stop pose
  geometry_msgs::msg::Point stop_line_point_msg;
  stop_line_point_msg.x = stop_line.value().second.x();
  stop_line_point_msg.y = stop_line.value().second.y();
  const double signed_arc_length_to_stop_point = autoware::motion_utils::calcSignedArcLength(
    input_path.points, self_pose->pose.position, stop_line_point_msg);
  setDistance(signed_arc_length_to_stop_point);

  // Check state
  if (state_ == State::APPROACH) {
    // Move to go out state if ego vehicle over deadline.
    constexpr double signed_deadline_length = -2.0;
    if (signed_arc_length_to_stop_point < signed_deadline_length) {
      RCLCPP_DEBUG(logger_, "APPROACH -> GO_OUT");
      state_ = State::GO_OUT;
      stop_signal_received_time_ptr_.reset();
      return true;
    }

    first_ref_stop_path_point_index_ = stop_line.value().first;

    // Check if stop is coming.
    const bool is_stop_signal = isStopSignal();

    // Update stop signal received time
    if (is_stop_signal) {
      if (!stop_signal_received_time_ptr_) {
        stop_signal_received_time_ptr_ = std::make_unique<Time>(clock_->now());
      }
    } else {
      stop_signal_received_time_ptr_.reset();
    }

    // Check hysteresis
    const double time_diff =
      stop_signal_received_time_ptr_
        ? std::max((clock_->now() - *stop_signal_received_time_ptr_).seconds(), 0.0)
        : 0.0;
    const bool to_be_stopped =
      is_stop_signal && (is_prev_state_stop_ || time_diff > planner_param_.stop_time_hysteresis);

    setSafe(!to_be_stopped);
    if (isActivated()) {
      is_prev_state_stop_ = false;
      return true;
    }

    // Decide whether to stop or pass even if a stop signal is received.
    if (!isPassthrough(signed_arc_length_to_stop_point)) {
      *path = insertStopPose(input_path, stop_line.value().first, stop_line.value().second);
      is_prev_state_stop_ = true;
    }
    return true;
  } else if (state_ == State::GO_OUT) {
    // Initialize if vehicle is far from stop_line
    constexpr bool use_initialization_after_start = true;
    constexpr double restart_length = 1.0;
    if (use_initialization_after_start) {
      if (signed_arc_length_to_stop_point > restart_length) {
        RCLCPP_DEBUG(logger_, "GO_OUT(RESTART) -> APPROACH");
        state_ = State::APPROACH;
      }
    }
    stop_signal_received_time_ptr_.reset();
    return true;
  }

  return false;
}

bool TrafficLightModule::isStopSignal()
{
  updateTrafficSignal();

  // If there is no upcoming traffic signal information,
  //   SIMULATION: it will PASS to prevent stopping on the planning simulator
  //   or scenario simulator.
  //   REAL ENVIRONMENT: it will STOP for safety in cases such that traffic light
  //   recognition is not working properly or the map is incorrect.
  if (!traffic_signal_stamp_) {
    if (planner_data_->is_simulation) {
      return false;
    }
    return true;
  }

  // Stop if the traffic signal information has timed out
  if (isTrafficSignalTimedOut()) {
    return true;
  }

  // Check if the current traffic signal state requires stopping
  return autoware::traffic_light_utils::isTrafficSignalStop(lane_, looking_tl_state_);
}

void TrafficLightModule::updateTrafficSignal()
{
  TrafficSignalStamped signal;
  if (!findValidTrafficSignal(signal)) {
    // Don't stop if it never receives traffic light topic.
    return;
  }

  traffic_signal_stamp_ = signal.stamp;

  // Found signal associated with the lanelet
  looking_tl_state_ = signal.signal;
  return;
}

bool TrafficLightModule::isPassthrough(const double & signed_arc_length) const
{
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double max_jerk = planner_data_->max_stop_jerk_threshold;
  const double delay_response_time = planner_data_->delay_response_time;

  const double reachable_distance =
    planner_data_->current_velocity->twist.linear.x * planner_param_.yellow_lamp_period;

  // Calculate distance until ego vehicle decide not to stop,
  // taking into account the jerk and acceleration.
  const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    planner_data_->current_velocity->twist.linear.x,
    planner_data_->current_acceleration->accel.accel.linear.x, max_acc, max_jerk,
    delay_response_time);

  const bool distance_stoppable = pass_judge_line_distance < signed_arc_length;
  const bool slow_velocity = planner_data_->current_velocity->twist.linear.x < 2.0;
  const bool stoppable = distance_stoppable || slow_velocity;
  const bool reachable = signed_arc_length < reachable_distance;

  const auto & enable_pass_judge = planner_param_.enable_pass_judge;

  if (enable_pass_judge && !stoppable && !is_prev_state_stop_) {
    // Cannot stop under acceleration and jerk limits.
    // However, ego vehicle can't enter the intersection while the light is yellow.
    // It is called dilemma zone. Make a stop decision to be safe.
    if (!reachable) {
      // dilemma zone: emergency stop
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[traffic_light] cannot pass through intersection during yellow lamp!");
      return false;
    } else {
      // pass through
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[traffic_light] can pass through intersection during yellow lamp");
      return true;
    }
  } else {
    return false;
  }
}

bool TrafficLightModule::findValidTrafficSignal(TrafficSignalStamped & valid_traffic_signal) const
{
  // get traffic signal associated with the regulatory element id
  const auto traffic_signal_stamped_opt = planner_data_->getTrafficSignal(
    traffic_light_reg_elem_.id(), false /* traffic light module does not keep last observation */);
  if (!traffic_signal_stamped_opt) {
    RCLCPP_WARN_STREAM_ONCE(
      logger_, "the traffic signal data associated with regulatory element id "
                 << traffic_light_reg_elem_.id() << " is not received");
    return false;
  }
  valid_traffic_signal = traffic_signal_stamped_opt.value();
  return true;
}

bool TrafficLightModule::isTrafficSignalTimedOut() const
{
  if (!traffic_signal_stamp_) {
    return false;
  }

  const auto is_traffic_signal_timeout =
    (clock_->now() - *traffic_signal_stamp_).seconds() > planner_param_.tl_state_timeout;
  if (is_traffic_signal_timeout) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "the received traffic signal data is outdated");
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */,
      "time diff: " << (clock_->now() - *traffic_signal_stamp_).seconds());
    return true;
  }
  return false;
}

tier4_planning_msgs::msg::PathWithLaneId TrafficLightModule::insertStopPose(
  const tier4_planning_msgs::msg::PathWithLaneId & input, const size_t & insert_target_point_idx,
  const Eigen::Vector2d & target_point)
{
  tier4_planning_msgs::msg::PathWithLaneId modified_path;
  modified_path = input;

  // Create stop pose
  const int target_velocity_point_idx = std::max(static_cast<int>(insert_target_point_idx - 1), 0);
  auto target_point_with_lane_id = modified_path.points.at(target_velocity_point_idx);
  target_point_with_lane_id.point.pose.position.x = target_point.x();
  target_point_with_lane_id.point.pose.position.y = target_point.y();
  target_point_with_lane_id.point.longitudinal_velocity_mps = 0.0;
  debug_data_.stop_poses.push_back(target_point_with_lane_id.point.pose);

  // Insert stop pose into path or replace with zero velocity
  size_t insert_index = insert_target_point_idx;
  planning_utils::insertVelocity(modified_path, target_point_with_lane_id, 0.0, insert_index);

  planning_factor_interface_->add(
    modified_path.points, planner_data_->current_odometry->pose,
    target_point_with_lane_id.point.pose, target_point_with_lane_id.point.pose,
    tier4_planning_msgs::msg::PlanningFactor::STOP, tier4_planning_msgs::msg::SafetyFactorArray{},
    true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "");

  return modified_path;
}

}  // namespace autoware::behavior_velocity_planner
