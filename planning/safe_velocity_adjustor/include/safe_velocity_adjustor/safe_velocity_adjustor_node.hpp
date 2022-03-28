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

#include "safe_velocity_adjustor/collision_distance.hpp"
#include "safe_velocity_adjustor/pointcloud_processing.hpp"

#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
using sensor_msgs::msg::PointCloud2;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using Float = decltype(TrajectoryPoint::longitudinal_velocity_mps);

class SafeVelocityAdjustorNode : public rclcpp::Node
{
public:
  explicit SafeVelocityAdjustorNode(const rclcpp::NodeOptions & node_options)
  : rclcpp::Node("safe_velocity_adjustor", node_options), transform_listener_(this)
  {
    time_safety_buffer_ = static_cast<Float>(declare_parameter<Float>("time_safety_buffer"));
    dist_safety_buffer_ = static_cast<Float>(declare_parameter<Float>("dist_safety_buffer"));

    sub_trajectory_ = create_subscription<Trajectory>(
      "~/input/trajectory", 1, [this](const Trajectory::ConstSharedPtr msg) { onTrajectory(msg); });
    sub_obstacle_pointcloud_ = create_subscription<PointCloud2>(
      "~/input/obstacle_pointcloud", rclcpp::QoS(1).best_effort().transient_local(),
      [this](const PointCloud2::ConstSharedPtr msg) { obstacle_pointcloud_ptr_ = msg; });

    pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
    pub_debug_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_markers", 1);

    set_param_res_ =
      add_on_set_parameters_callback([this](const auto & params) { return onParameter(params); });
  }

private:
  tier4_autoware_utils::TransformListener transform_listener_;
  rclcpp::Publisher<Trajectory>::SharedPtr
    pub_trajectory_;  //!< @brief publisher for output trajectory
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_debug_markers_;  //!< @brief publisher for debug markers
  rclcpp::Subscription<Trajectory>::SharedPtr
    sub_trajectory_;  //!< @brief subscriber for reference trajectory
  rclcpp::Subscription<PointCloud2>::SharedPtr
    sub_obstacle_pointcloud_;  //!< @brief subscriber for obstacle pointcloud

  // cached inputs
  PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr_;

  // parameters
  Float time_safety_buffer_;
  Float dist_safety_buffer_;
  // TODO(Maxime CLEMENT): get vehicle width and length from vehicle parameters
  Float vehicle_width_ = 2.0;
  Float vehicle_length_ = 4.0;

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
        dist_safety_buffer_ = static_cast<Float>(parameter.as_double());
        result.successful = true;
      } else {
        RCLCPP_WARN(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
      }
    }
    return result;
  }

  void onTrajectory(const Trajectory::ConstSharedPtr msg)
  {
    if (!obstacle_pointcloud_ptr_) {
      RCLCPP_WARN(get_logger(), "Obstable pointcloud not yet received");
      return;
    }
    std::vector<std::vector<geometry_msgs::msg::Point>> footprints;
    Trajectory safe_trajectory = *msg;
    const auto max_vel = std::max_element(
                           safe_trajectory.points.cbegin(), safe_trajectory.points.cend(),
                           [](const auto & p1, const auto & p2) {
                             return p1.longitudinal_velocity_mps < p2.longitudinal_velocity_mps;
                           })
                           ->longitudinal_velocity_mps;
    const auto filtered_obstacle_pointcloud = transformAndFilterPointCloud(
      safe_trajectory, *obstacle_pointcloud_ptr_, transform_listener_,
      max_vel * time_safety_buffer_ + dist_safety_buffer_);

    for (auto & trajectory_point : safe_trajectory.points) {
      const auto forward_simulated_vector = forwardSimulatedVector(
        trajectory_point, time_safety_buffer_, dist_safety_buffer_, vehicle_length_);
      const auto forward_simulated_footprint =
        forwardSimulatedFootprint(forward_simulated_vector, vehicle_width_);
      const auto dist_to_collision = distanceToClosestCollision(
        trajectory_point, forward_simulated_footprint, filtered_obstacle_pointcloud);
      if (dist_to_collision) {
        trajectory_point.longitudinal_velocity_mps = calculateSafeVelocity(
          trajectory_point,
          std::max(
            {}, static_cast<Float>(*dist_to_collision - vehicle_length_ - dist_safety_buffer_)));
      }
    }
    safe_trajectory.header.stamp = now();
    pub_trajectory_->publish(safe_trajectory);
    publishDebug(*msg, safe_trajectory);
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
    const Trajectory & original_trajectory, const Trajectory & adjusted_trajectory) const
  {
    visualization_msgs::msg::MarkerArray debug_markers;
    auto original_envelope =
      makeEnvelopeMarker(original_trajectory, time_safety_buffer_, dist_safety_buffer_);
    original_envelope.color.r = 1.0;
    original_envelope.ns = "original";
    debug_markers.markers.push_back(original_envelope);
    auto adjusted_envelope =
      makeEnvelopeMarker(adjusted_trajectory, time_safety_buffer_, dist_safety_buffer_);
    adjusted_envelope.color.g = 1.0;
    adjusted_envelope.ns = "adjusted";
    debug_markers.markers.push_back(adjusted_envelope);

    pub_debug_markers_->publish(debug_markers);
  }
};
}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__SAFE_VELOCITY_ADJUSTOR_NODE_HPP_
