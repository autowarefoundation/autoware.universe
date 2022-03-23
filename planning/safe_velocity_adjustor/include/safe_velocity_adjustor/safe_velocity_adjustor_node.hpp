// Copyright 2022 Tier IV, Inc.
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

#ifndef SAFE_VELOCITY_ADJUSTOR__SAFE_VELOCITY_ADJUSTOR_NODE_HPP_
#define SAFE_VELOCITY_ADJUSTOR__SAFE_VELOCITY_ADJUSTOR_NODE_HPP_

#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace safe_velocity_adjustor
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using Float = decltype(TrajectoryPoint::longitudinal_velocity_mps);

class SafeVelocityAdjustorNode : public rclcpp::Node
{
public:
  explicit SafeVelocityAdjustorNode(const rclcpp::NodeOptions & node_options)
  : rclcpp::Node("safe_velocity_adjustor", node_options)
  {
    time_safety_buffer_ = static_cast<Float>(declare_parameter<Float>("time_safety_buffer"));
    dist_safety_buffer_ = static_cast<Float>(declare_parameter<Float>("dist_safety_buffer"));

    pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
    pub_debug_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_markers", 1);
    sub_trajectory_ = create_subscription<Trajectory>(
      "~/input/trajectory", 1, [this](const Trajectory::ConstSharedPtr msg) { onTrajectory(msg); });

    set_param_res_ =
      add_on_set_parameters_callback([this](const auto & params) { return onParameter(params); });
  }

private:
  rclcpp::Publisher<Trajectory>::SharedPtr
    pub_trajectory_;  //!< @brief publisher for output trajectory
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_debug_markers_;  //!< @brief publisher for debug markers
  rclcpp::Subscription<Trajectory>::SharedPtr
    sub_trajectory_;  //!< @brief subscriber for reference trajectory

  // parameters
  Float time_safety_buffer_;
  Float dist_safety_buffer_;

  // parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "time_safety_buffer") {
        time_safety_buffer_ = static_cast<Float>(parameter.as_double());
      } else if (parameter.get_name() == "dist_safety_buffer") {
        time_safety_buffer_ = static_cast<Float>(parameter.as_double());
        result.successful = true;
      } else {
        RCLCPP_WARN(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
      }
    }
    return result;
  }

  // cached inputs
  Trajectory::ConstSharedPtr prev_input_trajectory_;

  // topic callback
  void onTrajectory(const Trajectory::ConstSharedPtr msg)
  {
    Trajectory safe_trajectory = *msg;
    for (auto & trajectory_point : safe_trajectory.points) {
      const auto closest_collision_point = distanceToClosestCollision(trajectory_point);
      if (closest_collision_point)
        trajectory_point.longitudinal_velocity_mps =
          calculateSafeVelocity(trajectory_point, *closest_collision_point);
    }
    safe_trajectory.header.stamp = now();
    pub_trajectory_->publish(safe_trajectory);
    publishDebug(*msg, safe_trajectory);
    prev_input_trajectory_ = msg;
  }

  // publish methods
  std::optional<Float> distanceToClosestCollision(const TrajectoryPoint & trajectory_point)
  {
    (void)trajectory_point;
    return {};
  }

  Float calculateSafeVelocity(
    const TrajectoryPoint & trajectory_point, const Float & dist_to_collision) const
  {
    return std::min(
      trajectory_point.longitudinal_velocity_mps,
      static_cast<Float>(dist_to_collision / time_safety_buffer_));
  }

  static visualization_msgs::msg::Marker makeEnvelopeMarker(
    const Trajectory & trajectory, const Float time_safety_buffer, const Float dist_safety_buffer)
  {
    visualization_msgs::msg::Marker envelope;
    envelope.header = trajectory.header;
    envelope.type = visualization_msgs::msg::Marker::LINE_STRIP;
    envelope.scale.x = 0.1;
    envelope.color.a = 1.0;
    for (const auto & point : trajectory.points) {
      const auto heading = tf2::getYaw(point.pose.orientation);
      auto p = point.pose.position;
      p.x += static_cast<Float>(
               point.longitudinal_velocity_mps * std::cos(heading) * time_safety_buffer) +
             dist_safety_buffer;
      p.y += static_cast<Float>(
               point.longitudinal_velocity_mps * std::sin(heading) * time_safety_buffer) +
             dist_safety_buffer;
      envelope.points.push_back(p);
    }
    return envelope;
  }

  void publishDebug(
    const Trajectory & original_trajectory, const Trajectory & safe_trajectory) const
  {
    visualization_msgs::msg::MarkerArray debug_markers;
    auto unsafe_envelope =
      makeEnvelopeMarker(original_trajectory, time_safety_buffer_, dist_safety_buffer_);
    unsafe_envelope.color.r = 1.0;
    unsafe_envelope.ns = "unsafe";
    debug_markers.markers.push_back(unsafe_envelope);
    auto safe_envelope =
      makeEnvelopeMarker(safe_trajectory, time_safety_buffer_, dist_safety_buffer_);
    safe_envelope.color.g = 1.0;
    safe_envelope.ns = "safe";
    debug_markers.markers.push_back(safe_envelope);
    pub_debug_markers_->publish(debug_markers);
  }

  // debug
};
}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__SAFE_VELOCITY_ADJUSTOR_NODE_HPP_
